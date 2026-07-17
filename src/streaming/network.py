import subprocess
import re

JETSON_MAC = "8c:b8:7e:04:20:a9"


def get_jetson_ip() -> str:
    mac = JETSON_MAC.lower()

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
            if mac in normalized_line:
                ip_match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
                if ip_match:
                    print(f"Found in neighbor cache -> {ip_match.group()}")
                    return ip_match.group()

    raise RuntimeError("Jetson was not found in the ARP/neighbor cache.")
