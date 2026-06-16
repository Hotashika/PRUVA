import subprocess
from scapy.layers.l2 import Ether, ARP
from scapy.sendrecv import srp

JETSON_MAC = "8c:b8:7e:04:20:a9"

def get_jetson_ip() -> str:
    mac = JETSON_MAC.lower()

    # Önce ARP cache'e bak (hızlı)
    result = subprocess.run(["arp", "-n"], capture_output=True, text=True)
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2].lower() == mac:
            print(f"ARP cache'den bulundu → {parts[0]}")
            return parts[0]

    # Cache'de yoksa ağı tara
    print("ARP cache'de bulunamadı, ağ taranıyor...")
    pkt = Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(pdst="192.168.1.0/24")
    answered, _ = srp(pkt, timeout=3, verbose=False)
    for _, rcv in answered:
        if rcv[ARP].hwsrc.lower() == mac:
            print(f"Bulundu → {rcv[ARP].psrc}")
            return rcv[ARP].psrc

    raise RuntimeError("Jetson bulunamadı, aynı ağda mısın?")