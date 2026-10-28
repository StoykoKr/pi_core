"""
Re4Life QR Generator — auto-detecting
=====================================
Generates the pairing QR with NOTHING hardcoded:

  - On the Pi: reads the hotspot SSID + passphrase from /etc/hostapd/hostapd.conf
    and the AP IP from the wlan0 interface.
  - On Windows (dev): reads the SSID you're currently connected to via netsh and
    auto-detects your machine's LAN IP. (Windows can't read the saved Wi-Fi password,
    so set WIFI_PASSWORD_FALLBACK below for dev.)

Must be run AFTER the signing server is up — it fetches a live single-use token from it.
The token is single-use and short-lived, so generate a fresh QR per pairing.
"""

import json
import logging
import os
import platform
import re
import socket
import subprocess

import requests
import qrcode
from cryptography import x509
from cryptography.hazmat.primitives import hashes
from PIL import Image, ImageDraw, ImageFont

# ── Config ────────────────────────────────────────────────────────────────────
CERT_DIR              = os.environ.get("RE4LIFE_CERT_DIR", "/home/s/pi/re4life/certificates")
AP_IFACE              = os.environ.get("RE4LIFE_AP_IFACE", "wlan0")
HOSTAPD_CONF          = os.environ.get("RE4LIFE_HOSTAPD", "/etc/hostapd/hostapd.conf")
SIGNING_PORT          = int(os.environ.get("RE4LIFE_SIGNING_PORT", "8443"))
BROKER_PORT           = int(os.environ.get("RE4LIFE_BROKER_PORT", "8883"))
CAMERA_USER           = os.environ.get("RE4LIFE_CAMERA_USER", "admin")
CAMERA_PASS           = os.environ.get("RE4LIFE_CAMERA_PASS", "CHANGE_ME")
WIFI_PASSWORD_FALLBACK = os.environ.get("RE4LIFE_WIFI_PASS", "")  # dev only (Windows)
OUTPUT_PATH           = os.environ.get("RE4LIFE_QR_OUT", "current_qr.png")


# ── Network auto-detection ──────────────────────────────────────────────────────

def detect_ip() -> str:
    """Pi: the AP interface IP. Dev: the primary LAN IP."""
    if platform.system() != "Windows":
        try:
            out = subprocess.check_output(["ip", "-4", "addr", "show", AP_IFACE]).decode()
            m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
            if m:
                return m.group(1)
        except Exception:
            pass
    # Fallback (works on any OS): source IP of the route to an external host.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()


def detect_wifi() -> tuple[str, str]:
    """Pi: SSID + passphrase from hostapd. Dev (Windows): current SSID (no password)."""
    if platform.system() != "Windows":
        ssid = pw = None
        with open(HOSTAPD_CONF) as f:
            for line in f:
                if line.startswith("ssid="):
                    ssid = line.split("=", 1)[1].strip()
                if line.startswith("wpa_passphrase="):
                    pw = line.split("=", 1)[1].strip()
        if not ssid:
            raise RuntimeError(f"Could not read ssid= from {HOSTAPD_CONF}")
        return ssid, (pw or "")
    # Windows dev
    out = subprocess.check_output(["netsh", "wlan", "show", "interfaces"]).decode(errors="ignore")
    m = re.search(r"^\s*SSID\s*:\s*(.+)$", out, re.MULTILINE)
    ssid = m.group(1).strip() if m else None
    if not ssid:
        raise RuntimeError("Could not detect current Wi-Fi SSID (are you connected?)")
    return ssid, WIFI_PASSWORD_FALLBACK


# ── QR generation ────────────────────────────────────────────────────────────────

class QRGenerator:
    def __init__(self, qr_size: int = 600):
        self.qr_size = qr_size
        logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
        self.logger = logging.getLogger(__name__)

    def generate(
        self,
        wifi_ssid: str,
        wifi_password: str,
        wifi_security: str,
        provisioning_url: str,
        server_cert_path: str,
        broker_ip: str,
        broker_port: int,
        camera_user: str,
        camera_pass: str,
    ) -> tuple[Image.Image, str]:
        # 1. Fetch a live token (verify pins us to OUR server cert -> no MitM)
        self.logger.info(f"Requesting token from {provisioning_url}/tokens ...")
        resp = requests.post(f"{provisioning_url}/tokens", verify=server_cert_path, timeout=5)
        resp.raise_for_status()
        token = resp.json().get("token")
        if not token:
            raise ValueError("Server did not return a token.")
        self.logger.info(f"Token: {token}")

        # 2. Fingerprint of the server cert (for the app's pinning check)
        fingerprint = self._fingerprint(server_cert_path)
        self.logger.info(f"Cert fingerprint: {fingerprint}")

        # 3. Build payload
        payload = {
            "wifi": {"ssid": wifi_ssid, "password": wifi_password, "security": wifi_security},
            "signing_server_url": provisioning_url,
            "token": token,
            "server_cert_fingerprint": fingerprint,
            "broker_ip": broker_ip,
            "broker_port": broker_port,
            "camera_user": camera_user,
            "camera_pass": camera_pass,
        }
        payload_json = json.dumps(payload, separators=(",", ":"))
        self.logger.info(f"Payload size: {len(payload_json)} bytes")

        # 4. Render
        qr = qrcode.QRCode(version=None, error_correction=qrcode.constants.ERROR_CORRECT_M,
                           box_size=10, border=4)
        qr.add_data(payload_json)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
        img = img.resize((self.qr_size, self.qr_size), Image.Resampling.NEAREST)
        return self._add_header(img), payload_json

    def _fingerprint(self, cert_path: str) -> str:
        with open(cert_path, "rb") as f:
            cert = x509.load_pem_x509_certificate(f.read())
        return ":".join(f"{b:02X}" for b in cert.fingerprint(hashes.SHA256()))

    def _add_header(self, qr_img: Image.Image) -> Image.Image:
        header_h = 120
        full = Image.new("RGB", (qr_img.width, qr_img.height + header_h), "white")
        draw = ImageDraw.Draw(full)
        try:
            title_font = ImageFont.truetype("DejaVuSans-Bold.ttf", 36)
            sub_font   = ImageFont.truetype("DejaVuSans.ttf", 24)
        except OSError:
            title_font = ImageFont.load_default()
            sub_font   = title_font
        for text, font, y, color in (
            ("Re4Life Robot Pairing", title_font, 20, "black"),
            ("Scan with the mobile app to enroll", sub_font, 70, "gray"),
        ):
            bbox = draw.textbbox((0, 0), text, font=font)
            draw.text(((qr_img.width - (bbox[2] - bbox[0])) // 2, y), text, fill=color, font=font)
        full.paste(qr_img, (0, header_h))
        return full

    def save(self, img: Image.Image, path: str) -> None:
        img.save(path, "PNG")
        self.logger.info(f"QR saved to {path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    
    # Force all standard logging outputs to stderr so they don't pollute the C# reading pipe
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", stream=sys.stderr)
    
    ip = detect_ip()
    ssid, wifi_pw = detect_wifi()

    server_cert = os.path.join(CERT_DIR, "mqtt_cert.crt")
    gen = QRGenerator(qr_size=600)
    try:
        img, payload = gen.generate(
            wifi_ssid=ssid,
            wifi_password=wifi_pw,
            wifi_security="WPA2-Personal",
            provisioning_url=f"https://{ip}:{SIGNING_PORT}",
            server_cert_path=server_cert,
            broker_ip=ip,
            broker_port=BROKER_PORT,
            camera_user=CAMERA_USER,
            camera_pass=CAMERA_PASS,
        )
        
        # Save the PNG file copy locally on the Pi filesystem as a fallback
        gen.save(img, OUTPUT_PATH)
        
        # CRITICAL: Print the exact signature string to standard out for C# to read
        print(f"QR_PAYLOAD:{payload}", flush=True)
        
    except Exception as e:
        print(f"Failed to generate QR: {e}", file=sys.stderr)
        sys.exit(1)
