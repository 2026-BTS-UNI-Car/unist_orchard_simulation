"""Private Unix-socket client. Never opens motor devices or writes command files."""
import json
import os
import socket


def request(payload):
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(0.6)
            client.connect(os.environ.get("ROVER_CONTROL_SOCKET", "/run/ubts-rover/control.sock"))
            client.sendall(json.dumps(payload, allow_nan=False).encode() + b"\n")
            data = b""
            while b"\n" not in data and len(data) < 32768:
                part = client.recv(4096)
                if not part:
                    break
                data += part
            return json.loads(data)
    except (OSError, ValueError) as exc:
        return {"ok": False, "connected": False, "state": "unavailable", "error": "motor_bridge_unavailable"}
