"""Read this bench device's recorded run and check it against the Mac replay."""
import math
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import termios
import time
import tty

root = Path(__file__).resolve().parent
registry = subprocess.check_output(
    ["ioreg", "-r", "-n", "STM32 Tilt Bench", "-l", "-w0"], text=True)
ports = set(re.findall(r'"IOCalloutDevice" = "(/dev/cu\.usbmodem[^"]+)"', registry))
if len(ports) != 1:
    sys.exit("Connect the STM32 Tilt Bench and allow the USB accessory on the Mac.")

def request(fd, command, terminator):
    termios.tcflush(fd, termios.TCIFLUSH)
    data = b""
    deadline = time.monotonic() + 6
    retry = 0
    while terminator not in data and time.monotonic() < deadline:
        if not data and time.monotonic() >= retry:
            os.write(fd, command)
            retry = time.monotonic() + 1
        if select.select([fd], [], [], 0.2)[0]:
            data += os.read(fd, 2048)
    return data


fd = os.open(ports.pop(), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    previous = termios.tcgetattr(fd)
    tty.setraw(fd)
    if sys.argv[1:] == ["--barometer"]:
        for index in range(3):
            line = request(fd, b"b", b"\n").decode().strip()
            if not re.fullmatch(r"BAROMETER,BMP(?:388|390),0x7[67],-?[0-9]+\.[0-9]{2} hPa,-?[0-9]+\.[0-9]{2} C", line):
                sys.exit(line or "No barometer response from the board.")
            print(line, flush=True)
            if index < 2:
                time.sleep(1)
        sys.exit(0)
    data = request(fd, b"r", b"\nEND\n")
finally:
    if 'previous' in locals():
        termios.tcsetattr(fd, termios.TCSANOW, previous)
    os.close(fd)

if not data.startswith(b"TILT-BENCH-1\n") or b"\nEND\n" not in data:
    sys.exit("The board did not send a complete tilt report.")
report = data.decode().split("END\n")[0] + "END\n"
print(report, end="")
values = {r.split(",")[0]: r.split(",")[1:] for r in report.splitlines() if "," in r}
reference = (root / "build/mac.csv").read_text().splitlines()
for line in reference:
    name, *want = line.split(",")
    got = values.get(name, [])
    if name == "fault_mask":
        if got != want or want != ["255"]:
            sys.exit("A bench fault case failed.")
        continue
    tolerances = [0.05, 0.05, 0.002, 0.002, 0.002, 0.002]
    if len(got) != 6 or len(want) != 6 or not all(
        math.isclose(float(a), float(b), abs_tol=t, rel_tol=0)
        for a, b, t in zip(want, got, tolerances)
    ):
        sys.exit(f"Mac and board differ at {name}.")
if "\nPASS\n" not in report:
    sys.exit("Board reported a failed replay or timing check.")
(root / "build/board-report.txt").write_text(report)
print("All three checkpoints match the Mac. All eight bench fault checks passed.")
