import subprocess
import time
import cv2
import platform
from PyQt5.QtGui import QImage
import concurrent.futures

JETSON_MAC = "8c:b8:7e:04:20:a9".lower()
PORT = 5000


def _ping(ip: str):
    """Tek bir IP'ye ping at (sessizce)."""
    if platform.system() == "Windows":
        # -n 1 paket, -w 500ms timeout
        args = ["ping", "-n", "1", "-w", "500", ip]
    else:
        # -c 1 paket, -W 1sn response timeout
        args = ["ping", "-c", "1", "-W", "1", ip]

    subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _get_local_subnet() -> str:
    """Aktif interface'in /24 subnet'ini döndür."""
    try:
        import netifaces
        gws = netifaces.gateways()
        iface = gws["default"][netifaces.AF_INET][1]
        ip = netifaces.ifaddresses(iface)[netifaces.AF_INET][0]["addr"]
        return ".".join(ip.split(".")[:3]) + ".{}"
    except Exception:
        return "192.168.1.{}"  # fallback


def _parse_arp_cache(target_mac: str) -> str | None:
    """ARP cache'den MAC'e karşılık gelen IP'yi bul."""
    result = subprocess.run(
        ["arp", "-a"] if platform.system() == "Windows" else ["arp", "-n"],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        parts = line.split()
        if platform.system() == "Windows":
            # Windows: "  192.168.1.5    8c-b8-7e-04-20-a9    dynamic"
            if len(parts) >= 2:
                mac = parts[1].replace("-", ":").lower()
                if mac == target_mac:
                    return parts[0]
        else:
            # Linux: "192.168.1.5 ether 8c:b8:7e:04:20:a9 ..."
            if len(parts) >= 3 and parts[2].lower() == target_mac:
                return parts[0]
    return None


def get_ip_by_mac(target_mac: str) -> str | None:
    # 1. Önce ARP cache'e bak (hızlı)
    ip = _parse_arp_cache(target_mac)
    if ip:
        return ip

    # 2. Cache'de yoksa ping sweep ile ARP cache'i doldur
    subnet = _get_local_subnet()
    hosts = [subnet.format(i) for i in range(1, 255)]

    with concurrent.futures.ThreadPoolExecutor(max_workers=64) as ex:
        ex.map(_ping, hosts)

    # 3. Tekrar cache'e bak
    return _parse_arp_cache(target_mac)


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
                    log_callback("Frame alınamadı, yeniden bağlanılıyor...")
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
        message = "Jetson IP bulunamadı, tekrar deneniyor..."
        if log_callback:
            log_callback(message)
        else:
            print(message)
        time.sleep(1.0)
        ip = jetson_ip or get_ip_by_mac(JETSON_MAC)

    if ip is None:
        message = "Jetson bulunamadı, aynı ağda mısın?"
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