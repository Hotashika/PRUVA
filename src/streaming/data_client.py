import json

import requests

from .network import get_jetson_ip

PORT = 5001


def main(jetson_ip=None):
    host = jetson_ip or get_jetson_ip()
    url = f"http://{host}:{PORT}/data/stream"
    print(f"Bağlandı → {url}")

    try:
        response = requests.get(url, stream=True, timeout=10)
        response.raise_for_status()
        for line in response.iter_lines():
            if not line or not line.startswith(b"data:"):
                continue
            payload = json.loads(line[5:])
            ts = payload["timestamp"]
            imu = payload["imu"]
            depth = payload["center_depth"]
            print(
                f"[{ts}] pitch={imu['pitch']:.2f} yaw={imu['yaw']:.2f} "
                f"roll={imu['roll']:.2f} | depth={depth}m"
            )
    except KeyboardInterrupt:
        print("Durduruluyor...")
    except requests.RequestException as exc:
        print(f"Jetson veri akışına bağlanılamadı: {exc}")


if __name__ == "__main__":
    main()
