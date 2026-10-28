#!/usr/bin/env bash
# ============================================================================
# One-time trust-root ceremony — adapted from the legacy setup_certs.sh.
# Generates the Root CA + broker/signing-server cert into /robot_data/certs.
# SAN pins the hotspot IP (192.168.4.1): the QR generator, the mobile app's
# pinning check, and the broker all depend on it.
#
# Idempotent: refuses to overwrite an existing CA (regenerating would orphan
# every already-provisioned phone). Delete the dir manually to start over.
# ============================================================================
set -euo pipefail

CERT_DIR="${RE4LIFE_CERT_DIR:-/robot_data/certs}"
AP_IP="${RE4LIFE_AP_IP:-192.168.4.1}"

# Re-asserting the broker-key permissions runs on EVERY invocation (not just
# first creation): install.sh step 5 chowns /robot_data recursively to the
# user, which would otherwise silently revert the key away from mosquitto's
# uid on a re-run and crash-loop the broker.
fix_key_perms() {
    sudo chmod 640 "$CERT_DIR/mqtt_key.key"
    sudo chown 1883:1883 "$CERT_DIR/mqtt_key.key"
    sudo chmod 600 "$CERT_DIR/root_ca.key"
}

if [ -f "$CERT_DIR/root_ca.crt" ]; then
    echo "[=] CA already exists in $CERT_DIR — keeping it (delete manually to regenerate)."
    fix_key_perms
    exit 0
fi

mkdir -p "$CERT_DIR/clients"
cd "$CERT_DIR"

# Root CA. keyUsage marked explicitly: strict verifiers (Python 3.13+ STRICT
# chain building) reject CA certs without it.
openssl genrsa -out root_ca.key 4096
openssl req -new -x509 -key root_ca.key -out root_ca.crt -days 3650 \
  -subj "/CN=Re4Life-RootCA/O=Re4Life/C=BG" \
  -addext "basicConstraints=critical,CA:TRUE" \
  -addext "keyUsage=critical,keyCertSign,cRLSign"

# SAN config — the hotspot IP MUST be here
cat > ext.cnf <<EOF
[req_ext]
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1  = 127.0.0.1
IP.2  = $AP_IP
EOF

# Broker cert (shared by the signing server — same SAN covers both)
openssl genrsa -out mqtt_key.key 2048
openssl req -new -key mqtt_key.key -out mqtt.csr -subj "/CN=$AP_IP/O=Re4Life/C=BG"
openssl x509 -req -in mqtt.csr -CA root_ca.crt -CAkey root_ca.key -CAcreateserial \
  -out mqtt_cert.crt -days 825 -sha256 -extfile ext.cnf -extensions req_ext

rm -f mqtt.csr ext.cnf root_ca.srl

# Append the root to the served cert -> mqtt_cert.crt is a CHAIN file.
# REQUIRED: qr_generator.py pins this file (requests verify=). On Python 3.12
# (the ros2_app image) a pinned NON-self-signed leaf alone fails verification
# ("unable to get local issuer certificate") — only 3.13+ sets
# VERIFY_X509_PARTIAL_CHAIN. Leaf+root verifies on both. Mosquitto's certfile
# and uvicorn's ssl_certfile both accept a chain file, and the QR fingerprint
# code reads only the FIRST PEM (the leaf), so nothing else changes.
cat root_ca.crt >> mqtt_cert.crt

# Permissions: the mosquitto container runs as uid/gid 1883 and mounts the dir
# read-only; it must be able to read the broker key. Other consumers (signing
# server, provisioning) run as root in their containers and read regardless.
# NOTE: chmod BEFORE chown (after the chown, the plain user may no longer
# touch the file) — and both via the helper so re-runs stay correct.
fix_key_perms

echo "[+] Done. Certs in $CERT_DIR (SAN includes $AP_IP)."
echo "    NEVER commit this directory — it is gitignored, keep it that way."
