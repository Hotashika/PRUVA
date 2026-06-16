import ipaddress
import platform
import re
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

import cv2
from PyQt5.QtGui import QImage

try:
    from scapy.layers.l2 import ARP, Ether
    from scapy.sendrecv import srp
except Exception:
    ARP = None
    Ether = None
    srp = None


JETSON_MAC = "8c:b8:7e:04:20:a9".lower()
PORT = 5000


def _subprocess_flags():
    if platform.system().lower() == "windows":
        return getattr(subprocess, "CREATE_NO_WINDOW", 0)
    return 0


def _arp_cache_ip(target_mac: str) -> str | None:
    is_windows = platform.system().lower() == "windows"
    formatted_mac = target_mac.lower().replace(":", "-" if is_windows else ":")

    try:
        result = subprocess.run(
            ["arp", "-a"],
            capture_output=True,
            text=True,
            creationflags=_subprocess_flags(),
        )
    except Exception:
        return None

    for line in result.stdout.splitlines():
        if formatted_mac in line.lower():
            match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
            if match:
                return match.group()

    return None


def _local_ipv4_networks():
    try:
        result = subprocess.check_output(
            ["ipconfig"],
            text=True,
            stderr=subprocess.DEVNULL,
            creationflags=_subprocess_flags(),
        )
    except Exception:
        return []

    networks = []
    current_ip = None
    for line in result.splitlines():
        ipv4_match = re.search(r"IPv4.*?:\s*([0-9.]+)", line)
        mask_match = re.search(r"Subnet Mask.*?:\s*([0-9.]+)", line)

        if ipv4_match:
            current_ip = ipv4_match.group(1)
        elif current_ip and mask_match:
            try:
                network = ipaddress.IPv4Network(
                    f"{current_ip}/{mask_match.group(1)}",
                    strict=False,
                )
                if not network.is_loopback:
                    networks.append(network)
            except Exception:
                pass
            current_ip = None

    return networks


def _ping_ip(ip: str) -> bool:
    if platform.system().lower() == "windows":
        command = ["ping", "-n", "1", "-w", "120", ip]
    else:
        command = ["ping", "-c", "1", "-W", "1", ip]

    try:
        result = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=_subprocess_flags(),
        )
        return result.returncode == 0
    except Exception:
        return False


def _scan_network_for_mac(target_mac: str) -> str | None:
    for network in _local_ipv4_networks():
        hosts = [str(ip) for ip in network.hosts()]
        if len(hosts) > 4096:
            continue

        with ThreadPoolExecutor(max_workers=64) as executor:
            futures = [executor.submit(_ping_ip, ip) for ip in hosts]
            for future in as_completed(futures):
                future.result()

        ip = _arp_cache_ip(target_mac)
        if ip:
            return ip

    return None


def get_ip_by_mac(target_mac: str) -> str | None:
    ip = _arp_cache_ip(target_mac)
    if ip:
        return ip

    ip = _scan_network_for_mac(target_mac)
    if ip:
        return ip

    if platform.system().lower() == "windows" or srp is None:
        return None

    pkt = Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(pdst="192.168.1.0/24")
    answered, _ = srp(pkt, timeout=3, verbose=False)
    for _, rcv in answered:
        if rcv[ARP].hwsrc.lower() == target_mac.lower():
            return rcv[ARP].psrc

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
                    log_callback("Frame alinamadi, yeniden baglaniliyor...")
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
        if cap is not None:
            cap.release()
        if frame_callback is None:
            cv2.destroyAllWindows()


def start(jetson_ip: str | None = None, frame_callback=None, log_callback=None, stop_callback=None):
    ip = jetson_ip or get_ip_by_mac(JETSON_MAC)
    while ip is None:
        if stop_callback is not None and not stop_callback():
            return

        message = "Jetson IP bulunamadi, tekrar deneniyor..."
        if log_callback:
            log_callback(message)
        else:
            print(message)

        time.sleep(1.0)
        ip = jetson_ip or get_ip_by_mac(JETSON_MAC)

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
