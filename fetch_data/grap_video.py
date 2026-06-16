import subprocess
import time
import cv2
import platform
from PyQt5.QtGui import QImage

JETSON_MAC = "8c:b8:7e:04:20:a9".lower()
PORT = 5000


def get_ip_by_mac(target_mac: str) -> str | None:
    # Önce ARP cache'e bak (platform-aware)
    if platform.system() == "Windows":
        result = subprocess.run(["arp", "-a"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            parts = line.split()
            # Windows formatı: IP  MAC  type
            if len(parts) >= 2:
                mac = parts[1].replace("-", ":").lower()
                if mac == target_mac:
                    return parts[0]
    else:
        result = subprocess.run(["arp", "-n"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[2].lower() == target_mac:
                return parts[0]

    # ARP cache'de yoksa Scapy ile tara
    try:
        from scapy.layers.l2 import ARP, Ether
        from scapy.sendrecv import srp
        import netifaces

        # Aktif interface'in subnet'ini bul
        gws = netifaces.gateways()
        iface = gws['default'][netifaces.AF_INET][1]
        addrs = netifaces.ifaddresses(iface)[netifaces.AF_INET][0]
        ip = addrs['addr']
        netmask = addrs['netmask']
        # Basit /24 varsayımı yerine gerçek subnet
        prefix = ".".join(ip.split(".")[:3]) + ".0/24"

        pkt = Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(pdst=prefix)
        answered, _ = srp(pkt, timeout=3, verbose=False)
        for _, rcv in answered:
            if rcv[ARP].hwsrc.lower() == target_mac:
                return rcv[ARP].psrc
    except Exception as e:
        print(f"Scapy tarama hatası: {e}")

    return None


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
