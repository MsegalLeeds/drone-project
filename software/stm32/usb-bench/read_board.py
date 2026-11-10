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
    return data if terminator in data else b""


def run_check(fd, args):
    if args == ["--imu-id"]:
        line = request(fd, b"i", b"\n").decode().strip()
        if line not in {"IMU,LSM6DSO,0x6B,0x6C", "IMU,LSM6DSO,0x6A,0x6C"}:
            sys.exit(line or "No IMU reply. Load the firmware built with --imu-id support.")
        print(line)
        return
    if args == ["--barometer"]:
        for index in range(3):
            line = request(fd, b"b", b"\n").decode().strip()
            match = re.fullmatch(r"BAROMETER,BMP(?:388|390),0x7[67],([0-9]+\.[0-9]{2}) hPa,(-?[0-9]+\.[0-9]{2}) C", line)
            if not match:
                sys.exit(line or "No barometer response from the board.")
            pressure, temperature = map(float, match.groups())
            if not (300 <= pressure <= 1250 and -40 <= temperature <= 85):
                sys.exit(f"Barometer sample outside operating range: {line}")
            print(line, flush=True)
            if index < 2:
                time.sleep(1)
        return
    data = request(fd, b"r", b"\nEND\n")
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
    timing = [values.get(key, []) for key in ("period_us", "peak_step_us", "max_start_late_us")]
    if not all(len(field) == 1 and field[0].isascii() and field[0].isdigit() for field in timing):
        sys.exit("Board timing fields are missing or invalid.")
    period, peak, late = (int(field[0]) for field in timing)
    if period != 10000 or peak + late >= period:
        sys.exit("Board timing exceeded the replay budget.")
    if "\nPASS\n" not in report:
        sys.exit("Board reported a failed replay or timing check.")
    (root / "build/board-report.txt").write_text(report)
    print("All three checkpoints match the Mac. All eight bench fault checks passed.")


if __name__ == "__main__":
    registry = subprocess.check_output(
        ["ioreg", "-r", "-n", "STM32 Tilt Bench", "-l", "-w0"], text=True)
    ports = set(re.findall(r'"IOCalloutDevice" = "(/dev/cu\.usbmodem[^"]+)"', registry))
    if len(ports) != 1:
        sys.exit("Connect the STM32 Tilt Bench and allow the USB accessory on the Mac.")

    fd = os.open(ports.pop(), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        previous = termios.tcgetattr(fd)
        tty.setraw(fd)
        run_check(fd, sys.argv[1:])
    finally:
        if 'previous' in locals():
            termios.tcsetattr(fd, termios.TCSANOW, previous)
        os.close(fd)
