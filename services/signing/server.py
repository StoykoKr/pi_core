"""
Re4Life Signing Server — Raspberry Pi
=====================================
Linux/Pi version of server_windows.py.

- Paths come from the RE4LIFE_CERT_DIR env var (default /home/pi/re4life/certificates),
  so nothing is hardcoded to a Windows desktop.
- Emits PKCS#8 private keys (-----BEGIN PRIVATE KEY-----). DO NOT change this back to
  TraditionalOpenSSL/PKCS#1 — the Android client cannot load PKCS#1 and the TLS
  handshake will silently die ("unexpected eof while reading").
- Generates a keypair + signed client cert on /provision when a valid single-use
  token is presented. Tokens are issued by /tokens and expire after TOKEN_TTL.

Run via systemd (see the deployment guide) so it auto-starts on boot.
"""

import datetime
import os
import sys
import json
import time
import secrets

import uvicorn
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ── Config (override with env vars) ─────────────────────────────────────────────

CERT_DIR     = os.environ.get("RE4LIFE_CERT_DIR", "/home/s/pi/re4life/certificates")
CA_CERT_PATH = os.path.join(CERT_DIR, "root_ca.crt")
CA_KEY_PATH  = os.path.join(CERT_DIR, "root_ca.key")
CLIENTS_DIR  = os.path.join(CERT_DIR, "clients")
TOKENS_FILE  = os.path.join(CERT_DIR, "tokens.json")

# The signing server presents the same cert/key as the broker (same SAN covers both).
SERVER_CERT  = os.path.join(CERT_DIR, "mqtt_cert.crt")
SERVER_KEY   = os.path.join(CERT_DIR, "mqtt_key.key")

PORT      = int(os.environ.get("RE4LIFE_SIGNING_PORT", "8443"))
TOKEN_TTL = int(os.environ.get("RE4LIFE_TOKEN_TTL", "600"))  # seconds (10 min default)

os.makedirs(CLIENTS_DIR, exist_ok=True)

# ── Token store helpers ─────────────────────────────────────────────────────────

def load_tokens() -> dict:
    if os.path.exists(TOKENS_FILE):
        with open(TOKENS_FILE, "r") as f:
            return json.load(f)
    return {}

def save_tokens(tokens: dict) -> None:
    with open(TOKENS_FILE, "w") as f:
        json.dump(tokens, f)

def cleanup_tokens(tokens: dict) -> dict:
    now = time.time()
    return {k: v for k, v in tokens.items() if v > now}

# ── Load CA ──────────────────────────────────────────────────────────────────────

def load_ca():
    try:
        with open(CA_CERT_PATH, "rb") as f:
            ca_cert = x509.load_pem_x509_certificate(f.read())
        with open(CA_KEY_PATH, "rb") as f:
            ca_key = serialization.load_pem_private_key(f.read(), password=None)
        return ca_cert, ca_key
    except Exception as e:
        print(f"[x] Failed to load CA from {CERT_DIR}: {e}")
        print("    Did you run setup_certs.sh on the Pi first?")
        sys.exit(1)

ca_cert, ca_key = load_ca()

print("=" * 55)
print("  Re4Life Signing Server — Raspberry Pi")
print("=" * 55)
print(f"  Cert dir   : {CERT_DIR}")
print(f"  Tokens file: {TOKENS_FILE}")
print(f"  Token TTL  : {TOKEN_TTL}s")
print(f"  Listening  : https://0.0.0.0:{PORT}")
print("=" * 55)

# ── App ────────────────────────────────────────────────────────────────────────

app = FastAPI(title="Re4Life Signing Server")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class ProvisionRequest(BaseModel):
    client_id: str
    token: str

# ── Routes ───────────────────────────────────────────────────────────────────────

@app.get("/health")
def health():
    return {"ok": True}

@app.post("/tokens")
def create_token():
    """Generate a secure, single-use token valid for TOKEN_TTL seconds."""
    tokens = cleanup_tokens(load_tokens())
    new_token = secrets.token_hex(16)
    tokens[new_token] = time.time() + TOKEN_TTL
    save_tokens(tokens)
    print(f"[+] Generated token: {new_token}")
    return {"token": new_token}

@app.post("/provision")
def provision(req: ProvisionRequest):
    """Validate token, generate keypair + signed cert, revoke token (single-use)."""
    try:
        client_id = req.client_id.strip()
        if not client_id:
            raise HTTPException(status_code=400, detail="client_id is required")

        # 1. Validate + consume the token
        tokens = cleanup_tokens(load_tokens())
        if req.token not in tokens:
            save_tokens(tokens)
            print(f"[x] Provisioning failed: invalid/expired token ({req.token})")
            raise HTTPException(status_code=401, detail="Invalid or expired token")
        del tokens[req.token]
        save_tokens(tokens)

        # 2. Generate client keypair
        client_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

        now     = datetime.datetime.now(datetime.timezone.utc)
        expires = now + datetime.timedelta(days=730)

        subject = x509.Name([
            x509.NameAttribute(NameOID.COMMON_NAME,       client_id),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Re4Life"),
            x509.NameAttribute(NameOID.COUNTRY_NAME,      "BG"),
        ])

        cert = (
            x509.CertificateBuilder()
            .subject_name(subject)
            .issuer_name(ca_cert.subject)
            .public_key(client_key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now)
            .not_valid_after(expires)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .sign(ca_key, hashes.SHA256())
        )

        # 3. PKCS#8 key — REQUIRED for the Android client
        client_key_pem = client_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        ).decode()

        client_cert_pem = cert.public_bytes(serialization.Encoding.PEM).decode()

        with open(CA_CERT_PATH, "r") as f:
            ca_cert_pem = f.read()

        # 4. Save a copy for debugging
        client_dir = os.path.join(CLIENTS_DIR, client_id)
        os.makedirs(client_dir, exist_ok=True)
        for name, data in (
            ("client.crt", client_cert_pem),
            ("client.key", client_key_pem),
            ("ca.crt",     ca_cert_pem),
        ):
            with open(os.path.join(client_dir, name), "w") as f:
                f.write(data)

        print(f"[+] Provisioned cert for CN={client_id}, valid until {expires.date()}")
        return {
            "client_key_pem":  client_key_pem,
            "client_cert_pem": client_cert_pem,
            "ca_cert_pem":     ca_cert_pem,
        }

    except HTTPException:
        raise
    except Exception as e:
        print(f"[x] Error provisioning for {req.client_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ── Run ──────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=PORT,
        ssl_certfile=SERVER_CERT,
        ssl_keyfile=SERVER_KEY,
        log_level="info",
    )
