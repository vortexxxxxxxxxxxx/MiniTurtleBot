import argparse
import csv
import json
import os
import select
import socket
import sys
import threading
import time
from datetime import datetime
from typing import Optional, Tuple

from pynput import keyboard

from odometry import DifferentialOdometry


DEFAULT_CMD_PORT = 5601
DEFAULT_TELEM_PORT = 5602


class TelemetryReceiver(threading.Thread):
    """Receives encoder telemetry over UDP and feeds odometry updater.

    Expected message formats (any of these):
      - JSON: {"A":1234, "B":1230} or keys "a"/"b", "encA"/"encB", "encoderCountA"/"encoderCountB"
      - Plain: "A=1234 B=1230" or "A:1234 B:1230"
      - CSV-like: "1234,1230" (assumed A,B order)
    """

    def __init__(self, listen_port: int, odom: DifferentialOdometry):
        super().__init__(daemon=True)
        self.listen_port = listen_port
        self.odom = odom
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self.last_counts: Optional[Tuple[int, int]] = None

    def stop(self):
        self._stop.set()
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", self.listen_port))
        s.setblocking(False)
        self._sock = s
        while not self._stop.is_set():
            r, _, _ = select.select([s], [], [], 0.1)
            if not r:
                continue
            try:
                data, _addr = s.recvfrom(2048)
            except OSError:
                break
            if not data:
                continue
            msg = data.decode(errors="ignore").strip()
            counts = self._parse_counts(msg)
            if counts is None:
                continue
            self.last_counts = counts
            self.odom.update_from_counts(counts[0], counts[1])

    @staticmethod
    def _parse_counts(s: str) -> Optional[Tuple[int, int]]:
        # Try JSON
        try:
            j = json.loads(s)
            for ka, kb in (("A","B"),("a","b"),("encA","encB"),("encoderCountA","encoderCountB")):
                if ka in j and kb in j:
                    return int(j[ka]), int(j[kb])
        except Exception:
            pass
        # Try key-value plain text
        try:
            parts = s.replace(",", " ").replace(":", " ").replace("=", " ").split()
            kv = {}
            lastk = None
            for p in parts:
                if p.upper() in ("A","B") or p in ("encA","encB","encoderCountA","encoderCountB"):
                    lastk = p
                elif lastk is not None:
                    kv[lastk] = int(p)
                    lastk = None
            for ka, kb in (("A","B"),("a","b"),("encA","encB"),("encoderCountA","encoderCountB")):
                if ka in kv and kb in kv:
                    return int(kv[ka]), int(kv[kb])
        except Exception:
            pass
        # Try two integers assumed A,B
        try:
            parts = [p for p in s.replace(","," ").split() if p.strip()]
            if len(parts) >= 2:
                a = int(parts[0]); b = int(parts[1])
                return a, b
        except Exception:
            pass
        return None


class Logger(threading.Thread):
    def __init__(self, odom: DifferentialOdometry, path: str, period_s: float = 0.2):
        super().__init__(daemon=True)
        self.odom = odom
        self.path = path
        self.period = period_s
        self._stop = threading.Event()
        self._last_logged: Optional[Tuple[float,float,float]] = None
        # Create directory
        os.makedirs(os.path.dirname(path), exist_ok=True)
        # Open file and write header if new
        self._file_lock = threading.Lock()
        self._fh = open(self.path, "a", newline="")
        self._csv = csv.writer(self._fh)
        if os.stat(self.path).st_size == 0:
            self._csv.writerow(["timestamp","x_m","y_m","theta_deg"])
            self._fh.flush()

    def stop(self):
        self._stop.set()
        try:
            with self._file_lock:
                self._fh.flush()
                self._fh.close()
        except Exception:
            pass

    def run(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.05, next_t - now))
                continue
            next_t += self.period
            pose = self.odom.get_pose()
            tup = (pose.x, pose.y, pose.theta)
            if self._last_logged is not None:
                # Skip if unchanged to reduce duplicates
                if all(abs(a-b) < 1e-9 for a,b in zip(tup, self._last_logged)):
                    continue
            self._last_logged = tup
            ts = datetime.utcnow().isoformat() + "Z"
            with self._file_lock:
                self._csv.writerow([ts, f"{pose.x:.6f}", f"{pose.y:.6f}", f"{(pose.theta*180.0/3.1415926535):.2f}"])
                self._fh.flush()


class Controller:
    def __init__(self, esp_ip: str, cmd_port: int):
        self.esp_ip = esp_ip
        self.cmd_port = cmd_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pressed = set()
        self._lock = threading.Lock()

    def send_cmd(self, msg: str):
        try:
            self.sock.sendto(msg.encode(), (self.esp_ip, self.cmd_port))
            print(f"Sent: {msg}")
        except Exception as e:
            print(f"Send failed: {e}")

    def on_press(self, key):
        with self._lock:
            if key == keyboard.Key.left and 'left' not in self._pressed:
                self._pressed.add('left'); self.send_cmd("L1")
            elif key == keyboard.Key.right and 'right' not in self._pressed:
                self._pressed.add('right'); self.send_cmd("R1")
            elif key == keyboard.Key.up and 'up' not in self._pressed:
                self._pressed.add('up'); self.send_cmd("F1")
            elif key == keyboard.Key.down and 'down' not in self._pressed:
                self._pressed.add('down'); self.send_cmd("B1")

    def on_release(self, key):
        with self._lock:
            if key == keyboard.Key.left:
                self._pressed.discard('left'); self.send_cmd("L0")
            elif key == keyboard.Key.right:
                self._pressed.discard('right'); self.send_cmd("R0")
            elif key == keyboard.Key.up:
                self._pressed.discard('up'); self.send_cmd("F0")
            elif key == keyboard.Key.down:
                self._pressed.discard('down'); self.send_cmd("B0")


def main():
    ap = argparse.ArgumentParser(description="Arrow-key control with UDP odometry logging")
    ap.add_argument("--esp-ip", dest="esp_ip", required=True, help="ESP device IP address")
    ap.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT, help="UDP port for commands (default 5601)")
    ap.add_argument("--telemetry-port", type=int, default=DEFAULT_TELEM_PORT, help="UDP port to receive encoder telemetry (default 5602)")
    ap.add_argument("--log", dest="log_path", default=os.path.join(os.path.dirname(__file__), "odometry_log.csv"), help="Path to CSV log file")
    ap.add_argument("--log-period", type=float, default=0.2, help="Log period in seconds (default 0.2s)")
    args = ap.parse_args()

    odom = DifferentialOdometry()
    telem = TelemetryReceiver(args.telemetry_port, odom)
    logger = Logger(odom, args.log_path, args.log_period)
    ctrl = Controller(args.esp_ip, args.cmd_port)

    telem.start(); logger.start()

    print("Controls ready. Use arrow keys; ESC to exit.")
    print(f"Logging to {args.log_path} at {args.log_period}s intervals.")
    print(f"Listening for telemetry on UDP :{args.telemetry_port} (A/B encoder counts)...")

    # Print pose periodically to console
    def print_pose_loop():
        try:
            while True:
                p = odom.get_pose()
                sys.stdout.write(f"\rx={p.x:.3f} m  y={p.y:.3f} m  th={p.theta*180.0/3.1415926535:.1f} deg   ")
                sys.stdout.flush()
                time.sleep(0.2)
        except Exception:
            pass
    printer = threading.Thread(target=print_pose_loop, daemon=True)
    printer.start()

    def on_release(key):
        if key == keyboard.Key.esc:
            return False
        ctrl.on_release(key)

    with keyboard.Listener(on_press=ctrl.on_press, on_release=on_release) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            pass

    # Cleanup
    telem.stop(); logger.stop()
    print("\nStopped.")


if __name__ == "__main__":
    main()
