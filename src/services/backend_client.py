import json
import socket
import time
from pathlib import Path
from urllib import error, request


DEFAULT_CONFIG = {
    "backend": {
        "host": "auto",
        "protocol": "http",
        "http_port": 8000,
        "tcp_port": 9000,
        "mission_upload_path": "/api/mission/upload_txt",
        "mission_start_path": "/api/mission/start",
        "pixhawk_arm_path": "/api/pixhawk/arm",
        "pixhawk_mode_path": "/api/pixhawk/set_mode",
        "emergency_stop_path": "/api/mission/stop",
        "timeout_s": 5,
    },
    "mission": {
        "default_name": "UPLOADED_WAYPOINTS",
        "upload_to_pixhawk": True,
    },
}


class BackendClientError(RuntimeError):
    pass


class BackendClient:
    def __init__(self, config_path=None):
        repo_root = Path(__file__).resolve().parents[2]
        self.config_path = Path(config_path) if config_path else repo_root / "config" / "settings.yaml"
        self.config = self._load_config()

    def upload_mission_waypoints(self, waypoints_path, jetson_ip=None, mission_name=None):
        waypoints_path = Path(waypoints_path)
        if waypoints_path.suffix.lower() != ".waypoints":
            raise BackendClientError("Mission file must have a .waypoints extension.")
        if not waypoints_path.exists():
            raise BackendClientError(f"Mission file not found: {waypoints_path}")

        host = self._backend_host(jetson_ip)
        if not host:
            raise BackendClientError("Jetson backend host is not configured and Jetson IP is not available.")

        upload_filename = self._mission_filename(mission_name, waypoints_path.name)
        payload = {
            "type": "mission_waypoints_upload",
            "mission_name": mission_name or self.config["mission"]["default_name"],
            "filename": upload_filename,
            "content": self._read_text(waypoints_path),
            "upload_to_pixhawk": bool(self.config["mission"].get("upload_to_pixhawk", True)),
            "client_time": time.time(),
        }

        protocol = str(self.config["backend"].get("protocol", "http")).lower()
        if protocol == "tcp":
            return self._post_tcp(host, payload)
        return self._post_http(host, self.config["backend"].get("mission_upload_path"), payload)

    @staticmethod
    def _mission_filename(mission_name, original_filename):
        if not mission_name:
            return original_filename
        name = Path(str(mission_name)).name
        return name if name.lower().endswith(".waypoints") else f"{name}.waypoints"

    def upload_mission_txt(self, txt_path, jetson_ip=None, mission_name=None):
        """Geriye dönük uyumluluk için eski yükleyici adı."""
        return self.upload_mission_waypoints(txt_path, jetson_ip, mission_name)

    def start_mission(self, mission_name, mission_id=None, jetson_ip=None):
        host = self._backend_host(jetson_ip)
        if not host:
            raise BackendClientError("Jetson backend host is not configured and Jetson IP is not available.")

        payload = {
            "type": "mission_start",
            "mission_name": mission_name,
            "mission_id": mission_id,
            "client_time": time.time(),
        }

        protocol = str(self.config["backend"].get("protocol", "http")).lower()
        if protocol == "tcp":
            return self._post_tcp(host, payload)
        return self._post_http(host, self.config["backend"].get("mission_start_path"), payload)

    def set_arm(self, armed, jetson_ip=None):
        payload = {
            "type": "pixhawk_arm",
            "armed": bool(armed),
            "client_time": time.time(),
        }
        return self._send_backend_command(
            jetson_ip,
            self.config["backend"].get("pixhawk_arm_path"),
            payload,
        )

    def set_mode(self, mode, jetson_ip=None):
        payload = {
            "type": "pixhawk_set_mode",
            "mode": str(mode).upper(),
            "client_time": time.time(),
        }
        return self._send_backend_command(
            jetson_ip,
            self.config["backend"].get("pixhawk_mode_path"),
            payload,
        )

    def emergency_stop(self, jetson_ip=None):
        payload = {
            "type": "emergency_stop",
            "client_time": time.time(),
        }
        return self._send_backend_command(
            jetson_ip,
            self.config["backend"].get("emergency_stop_path"),
            payload,
        )

    def _send_backend_command(self, jetson_ip, path, payload):
        host = self._backend_host(jetson_ip)
        if not host:
            raise BackendClientError("Jetson backend host is not configured and Jetson IP is not available.")

        protocol = str(self.config["backend"].get("protocol", "http")).lower()
        if protocol == "tcp":
            return self._post_tcp(host, payload)
        return self._post_http(host, path, payload)

    def _backend_host(self, jetson_ip):
        host = str(self.config["backend"].get("host", "auto")).strip()
        if not host or host.lower() == "auto":
            return jetson_ip
        return host

    def _post_http(self, host, path, payload):
        port = int(self.config["backend"].get("http_port", 8000))
        path = str(path or "/")
        timeout = float(self.config["backend"].get("timeout_s", 5))
        url = f"http://{host}:{port}{path}"
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with request.urlopen(req, timeout=timeout) as response:
                body = response.read().decode("utf-8", errors="replace")
        except error.URLError as exc:
            raise BackendClientError(f"Backend HTTP upload failed: {url} ({exc})") from exc

        return self._parse_response(body)

    def _post_tcp(self, host, payload):
        port = int(self.config["backend"].get("tcp_port", 9000))
        timeout = float(self.config["backend"].get("timeout_s", 5))
        message = json.dumps(payload).encode("utf-8") + b"\n"

        try:
            with socket.create_connection((host, port), timeout=timeout) as sock:
                sock.settimeout(timeout)
                sock.sendall(message)
                chunks = []
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    chunks.append(chunk)
        except OSError as exc:
            raise BackendClientError(f"Backend TCP upload failed: {host}:{port} ({exc})") from exc

        body = b"".join(chunks).decode("utf-8", errors="replace")
        return self._parse_response(body)

    def _parse_response(self, body):
        if not body.strip():
            raise BackendClientError("Backend returned an empty response.")
        try:
            return json.loads(body)
        except json.JSONDecodeError as exc:
            raise BackendClientError(f"Backend returned non-JSON response: {body[:200]}") from exc

    def _read_text(self, path):
        for encoding in ("utf-8", "cp1254", "latin-1"):
            try:
                return path.read_text(encoding=encoding)
            except UnicodeDecodeError:
                continue
        return path.read_text(errors="replace")

    def _load_config(self):
        config = json.loads(json.dumps(DEFAULT_CONFIG))
        if not self.config_path.exists():
            return config

        current_section = None
        for raw_line in self.config_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].rstrip()
            if not line:
                continue
            if not line.startswith(" ") and line.endswith(":"):
                current_section = line[:-1].strip()
                config.setdefault(current_section, {})
                continue
            if current_section and ":" in line:
                key, value = line.strip().split(":", 1)
                config[current_section][key.strip()] = self._coerce_value(value.strip())

        return config

    def _coerce_value(self, value):
        if value.startswith('"') and value.endswith('"'):
            return value[1:-1]
        if value.startswith("'") and value.endswith("'"):
            return value[1:-1]
        lowered = value.lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
        try:
            if "." in value:
                return float(value)
            return int(value)
        except ValueError:
            return value
