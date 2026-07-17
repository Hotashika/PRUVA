import subprocess
import time
import re

import cv2
from PyQt5.QtGui import QImage

JETSON_MAC = "8c:b8:7e:04:20:a9".lower()
PORT = 5000


def get_ip_by_mac(target_mac: str) -> str | None:
    normalized_mac = target_mac.lower().replace("-", ":")
    commands = (
        ["arp", "-a"],
        ["ip", "neigh"],
        ["ip", "neighbor"],
        ["netsh", "interface", "ip", "show", "neighbors"],
    )
    for command in commands:
        try:
            result = subprocess.run(command, capture_output=True, text=True, check=False)
        except FileNotFoundError:
            continue
        for line in result.stdout.splitlines():
            normalized_line = line.lower().replace("-", ":")
            if normalized_mac in normalized_line:
                ip_match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
                if ip_match:
                    return ip_match.group()

    return None


# noinspection D
def main(jetson_ip: str, frame_callback=None, log_callback=None, stop_callback=None):
    url = f"http://{jetson_ip}:{PORT}/video_feed"

    cap = None

    while True:
        if stop_callback is not None and not stop_callback():
            return

        cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if cap.isOpened():
            break

        cap.release()
        if log_callback:
            log_callback("ZED stream failed to open, retrying...")
        else:
            print("ZED stream failed to open, retrying...")
        time.sleep(1.0)

    if log_callback:
        log_callback(f"ZED stream connected: {url}")
    else:
        print(f"ZED stream connected: {url}")

    try:
        while True:
            if stop_callback is not None and not stop_callback():
                break

            ret, frame = cap.read()
            if not ret:
                if log_callback:
                    log_callback("Frame was not received, reconnecting...")
                cap.release()
                cap = cv2.VideoCapture(url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue

            if frame_callback is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channels = rgb.shape
                qimg = QImage(
                    rgb.tobytes(),
                    width,
                    height,
                    channels * width,
                    QImage.Format_RGB888,
                ).copy()
                frame_callback(qimg)
            else:
                cv2.imshow("ZED Stream", frame)
                if cv2.waitKey(1) == 27:
                    break
    finally:
        cap.release()
        if frame_callback is None:
            cv2.destroyAllWindows()


def start(jetson_ip: str | None = None, frame_callback=None, log_callback=None, stop_callback=None):
    ip = jetson_ip or get_ip_by_mac(JETSON_MAC)
    while ip is None:
        if stop_callback is not None and not stop_callback():
            return
        message = "Jetson IP was not found, retrying..."
        if log_callback:
            log_callback(message)
        else:
            print(message)
        time.sleep(1.0)
        ip = jetson_ip or get_ip_by_mac(JETSON_MAC)

    if ip is None:
        message = "Jetson was not found. Check that it is on the same network."
        if log_callback:
            log_callback(message)
        else:
            print(message)
        return

    if log_callback:
        log_callback(f"Jetson IP: {ip}")
    else:
        print(f"Jetson IP: {ip}")

    main(
        ip,
        frame_callback=frame_callback,
        log_callback=log_callback,
        stop_callback=stop_callback,
    )


if __name__ == "__main__":
    start()
