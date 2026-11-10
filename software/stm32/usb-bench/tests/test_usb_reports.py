"""Exercise the same reader used in the CLion terminal with known USB replies."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

bench = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(bench))
import read_board

saved = bench.parents[2] / "experiments/stm32-usb"
tilt = (saved / "board-report.txt").read_bytes()
baro = b"BAROMETER,BMP388,0x76,1000.96 hPa,21.63 C\n"


class USBReports(unittest.TestCase):
    def run_reader(self, args, command, replies):
        def exchange(fd, sent, terminator):
            self.assertEqual(sent, command)
            self.assertEqual(terminator, b"\nEND\n" if sent == b"r" else b"\n")
            return next(messages)

        messages = iter(replies)
        with tempfile.TemporaryDirectory() as tmp:
            build = Path(tmp) / "build"
            build.mkdir()
            (build / "mac.csv").write_bytes((saved / "mac.csv").read_bytes())
            with patch.object(read_board, "root", Path(tmp)), patch.object(
                read_board, "request", side_effect=exchange
            ), patch.object(read_board.time, "sleep"), contextlib.redirect_stdout(io.StringIO()) as out:
                read_board.run_check(None, args)
                self.assertEqual(list(messages), [])
                return out.getvalue()

    def test_saved_tilt_and_motor_checkpoints(self):
        self.assertIn("All eight", self.run_reader([], b"r", [tilt]))

    def test_invalid_tilt_reports(self):
        for key, replacement in [(b"END\n", b""), (b"PASS", b"FAIL"), (b"fault_mask,255", b"fault_mask,127"),
                                 (b"0.75000", b"0.25000"), (b"peak_step_us,32", b"peak_step_us,9999")]:
            with self.subTest(key=key), self.assertRaises(SystemExit):
                self.run_reader([], b"r", [tilt.replace(key, replacement)])

    def test_three_barometer_samples(self):
        self.assertEqual(self.run_reader(["--barometer"], b"b", [baro] * 3).splitlines(),
                         [baro.decode().strip()] * 3)

    def test_intermittent_barometer_and_bad_measurements(self):
        errors = [b"BAROMETER,NO_ACK,weak_pulldown_B8=1,B9=0\n", b"", baro.replace(b"hPa", b"Pa"),
                  baro.replace(b"1000.96", b"0.00"), baro.replace(b"21.63", b"99.00")]
        for reply in errors:
            with self.subTest(reply=reply), self.assertRaises(SystemExit):
                self.run_reader(["--barometer"], b"b", [baro, reply])

    def test_imu_identity(self):
        for address in ("0x6A", "0x6B"):
            line = f"IMU,LSM6DSO,{address},0x6C"
            self.assertEqual(self.run_reader(["--imu-id"], b"i", [line.encode() + b"\n"]).strip(), line)
        for reply in (b"IMU,UNKNOWN_ID,0x6B,0x00\n", b"IMU,NO_ACK\n", b""):
            with self.subTest(reply=reply), self.assertRaises(SystemExit):
                self.run_reader(["--imu-id"], b"i", [reply])


if __name__ == "__main__":
    unittest.main()
