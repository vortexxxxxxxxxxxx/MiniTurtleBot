import argparse
import socket
import time
import json

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5602)
    ap.add_argument('--rate', type=float, default=50.0, help='Hz')
    ap.add_argument('--speed-a', type=int, default=60, help='ticks per interval synthetic')
    ap.add_argument('--speed-b', type=int, default=60, help='ticks per interval synthetic')
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    a = 0; b = 0
    dt = 1.0/max(1e-3, args.rate)
    print(f"Sending synthetic telemetry to {args.host}:{args.port} at {args.rate} Hz...")
    try:
        while True:
            a += args.speed_a
            b += args.speed_b
            msg = json.dumps({'A': a, 'B': b}).encode()
            s.sendto(msg, (args.host, args.port))
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
