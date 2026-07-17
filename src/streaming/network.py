import subprocess

JETSON_MAC = "8c:b8:7e:04:20:a9"


def get_jetson_ip() -> str:
    mac = JETSON_MAC.lower()

    try:
        result = subprocess.run(["arp", "-a"], capture_output=True, text=True, check=False)
    except FileNotFoundError as exc:
        raise RuntimeError("ARP command is not available on this system.") from exc
    for line in result.stdout.splitlines():
        normalized_line = line.lower().replace("-", ":")
        if mac in normalized_line:
            for part in line.split():
                if part.count(".") == 3:
                    print(f"Found in ARP cache -> {part}")
                    return part

    raise RuntimeError("Jetson was not found in ARP cache.")
