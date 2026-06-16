import requests
import json
from fetch_data.utils import get_jetson_ip

JETSON_IP = get_jetson_ip()
PORT = 5001

url = f"http://{JETSON_IP}:{PORT}/data/stream"

print(f"Bağlandı → {url}")

try:
    response = requests.get(url, stream=True)
    for line in response.iter_lines():
        if not line:
            continue
        if line.startswith(b"data:"):
            payload = json.loads(line[5:])
            ts  = payload["timestamp"]
            imu = payload["imu"]
            cd  = payload["center_depth"]
            print(f"[{ts}] pitch={imu['pitch']:.2f} yaw={imu['yaw']:.2f} roll={imu['roll']:.2f} | depth={cd}m")

except KeyboardInterrupt:
    print("Durduruluyor...")
except requests.exceptions.ConnectionError:
    print("Jetson'a bağlanılamadı")