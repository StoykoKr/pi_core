#!/usr/bin/env python3
"""
Re4Life provisioning wrapper node — the ONLY new code in the provisioning
chain (contract §1). qr_generator.py stays untouched and is subprocessed
per request, exactly like the legacy C# ProvisioningManager did.

Flow:
  /ui/request_qr (std_msgs/Empty)  <- esp_sensors publishes on CYD touch
      -> subprocess qr_generator.py (fetches a live single-use token from the
         signing server, builds the payload)
      -> stdout line "QR_PAYLOAD:{json}"
  /ui/qr_payload (std_msgs/String) -> esp_sensors forwards over UART to the CYD

Failure policy (contract §5): on ANY failure publish NOTHING on qr_payload
(never feed the CYD garbage), log loudly, and report on
/ui/provisioning_status so the bridge can surface it in the app
(robot/status/system.provisioning_ok).

QoS: Reliable, volatile, NOT latched — the token inside is single-use with a
short TTL, so replaying a stale payload would render a dead QR.
"""

import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty, String

# esp_sensors allocates a fixed 600 B buffer for this message (contract §2);
# screen firmware caps at QR version 12. Refuse to send anything bigger.
MAX_PAYLOAD_BYTES = 600

GENERATOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "qr_generator.py")
TIMEOUT_S = 30


class ProvisioningNode(Node):
    def __init__(self):
        super().__init__("provisioning")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # NOT latched, by design
        )
        self._pub_payload = self.create_publisher(String, "/ui/qr_payload", qos)
        self._pub_status = self.create_publisher(String, "/ui/provisioning_status", qos)
        self.create_subscription(Empty, "/ui/request_qr", self._on_request, qos)

        # Serialize generations (mirrors the legacy process lock): a touch-happy
        # user should queue, not spawn parallel generators fighting over tokens.
        self._lock = threading.Lock()

        self.get_logger().info(f"Provisioning ready (generator: {GENERATOR})")
        self._status("ok", "provisioning node started")

    # ------------------------------------------------------------------
    def _on_request(self, _msg: Empty):
        # Run off the executor thread so a slow generation never blocks the sub.
        threading.Thread(target=self._generate, daemon=True).start()

    def _generate(self):
        if not self._lock.acquire(blocking=False):
            self.get_logger().warn("QR generation already in progress — request ignored")
            return
        try:
            self.get_logger().info("QR requested — running generator...")
            env = os.environ.copy()
            # The generator writes a PNG copy next to its cwd; our mount is
            # read-only, so point both at /tmp.
            env.setdefault("RE4LIFE_QR_OUT", "/tmp/current_qr.png")

            try:
                proc = subprocess.run(
                    ["python3", GENERATOR],
                    capture_output=True, text=True, timeout=TIMEOUT_S,
                    cwd="/tmp", env=env,
                )
            except subprocess.TimeoutExpired:
                self._fail(f"generator timed out after {TIMEOUT_S}s (signing server down?)")
                return
            except Exception as e:  # noqa: BLE001 — anything here is a hard fail
                self._fail(f"could not run generator: {e}")
                return

            if proc.returncode != 0:
                self._fail(f"generator exited {proc.returncode}: {proc.stderr.strip()[-400:]}")
                return

            payload = None
            for line in proc.stdout.splitlines():
                if line.startswith("QR_PAYLOAD:"):
                    payload = line[len("QR_PAYLOAD:"):].strip()
                    break
            if not payload:
                self._fail(f"no QR_PAYLOAD in generator output: {proc.stdout.strip()[-400:]}")
                return

            size = len(payload.encode("utf-8"))
            if size > MAX_PAYLOAD_BYTES:
                self._fail(f"payload {size} B exceeds the {MAX_PAYLOAD_BYTES} B "
                           f"esp_sensors buffer — NOT sent (shorten SSID/passphrase?)")
                return

            msg = String()
            msg.data = payload
            self._pub_payload.publish(msg)
            self.get_logger().info(f"QR payload published ({size} B)")
            self._status("ok", f"qr delivered ({size} B)")
        finally:
            self._lock.release()

    # ------------------------------------------------------------------
    def _fail(self, reason: str):
        # Contract: nothing on qr_payload, loud log, status for the app.
        self.get_logger().error(f"PROVISIONING FAILED: {reason}")
        self._status("error", reason)

    def _status(self, level: str, detail: str):
        msg = String()
        msg.data = f"{level}|{detail}"
        self._pub_status.publish(msg)


def main():
    rclpy.init()
    node = ProvisioningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
