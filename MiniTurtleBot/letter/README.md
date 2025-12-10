MiniTurtleBot Letter Odometry

This adds keyboard control with UDP-based odometry logging from encoder telemetry.

Files
- `letter.py`: original arrow-key UDP control (no odometry)
- `odometry.py`: differential-drive odometry core (x, y, theta)
- `letter_odometry.py`: control + telemetry receiver + periodic CSV logging

Requirements
- Python 3.8+
- `pynput` installed (`pip install pynput`)
- ESP firmware should broadcast encoder counts over UDP to the host on a known port (default: 5602) in one of the following formats per packet:
  - JSON: `{ "A":12345, "B":12340 }` (or `a/b`, `encA/encB`, `encoderCountA/encoderCountB`)
  - Key-value: `A=12345 B=12340` or `A:12345 B:12340`
  - CSV: `12345,12340` (assumed `A,B` order)

Run (lin)
```
pip install pynput matplotlib
python3 letter_odometry.py --esp-ip <ESP_IP> \
  --cmd-port 5601 \
  --telemetry-port 5602 \
  --log ./odometry_log.csv \
  --log-period 0.2
```
Run (win)
```
python MiniTurtleBot\letter\letter_odometry.py --esp-ip 192.168.1.14 --cmd-port 5601 --telemetry-port 5602 --log MiniTurtleBot\letter\odometry_log.csv --log-period 0.2

python MiniTurtleBot\letter\plot_path.py --csv MiniTurtleBot\letter\odometry_log.csv --out MiniTurtleBot\letter\odometry_path.png --show
```

- Use arrow keys to drive; press ESC to exit.
- Logs append to `odometry_log.csv` with columns: `timestamp,x_m,y_m,theta_deg`.
- Live pose prints to the console at ~5 Hz.

Notes
- Geometry and encoder constants are aligned with `base_setup/MotorEncoder_code.ino` and `square/follow_line.ino`:
  - `ticks_per_rev = 12 * 4`, `gear_ratio = 30.0`
  - `wheel_radius = 0.0325 m` (65 mm), `track_width = 0.10 m`
- If your encoder direction is inverted per wheel, adjust `ENC_A_DIR` / `ENC_B_DIR` in `odometry.py`.
- If you prefer on-board odometry, the `square/follow_line.ino` firmware exposes `/pose` via its web UI; this Python solution computes odometry off-board from encoder counts.

Plot the path
```
python3 plot_path.py --csv ./odometry_log.csv --out ./odometry_path.png --show
```
This generates a PNG and optionally shows an interactive window.

Interactive controls in the plot window
- Press `C` to clear the drawing (keeps axes/grid).
- Press `R` to redraw the last loaded trajectory.
- Press `Q` or `Esc` to close the window.

Firmware telemetry
- The firmware in `letter/letter.ino` now sends encoder counts as JSON `{"A":<encA>,"B":<encB>}` to the last command sender's IP on UDP port `5602` at ~20 Hz. This enables `letter_odometry.py` to log pose without extra setup.

Finding the ESP32 serial port (Linux)
- List boards via Arduino CLI:
```
arduino-cli board list
```
- Or list candidate device files:
```
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | grep -i tty | tail -n 20
```
Use the reported port (e.g., `/dev/ttyUSB0`) when uploading.

Finding the ESP32 IP and UDP ports
- The OLED displays the IP after Wi‑Fi connects.
- Command UDP port is `5601` (PC → ESP). Telemetry UDP port is `5602` (ESP → PC, to the last sender IP).

Heatmap scan (SPACE key)
- New script in `heatmap/heatmap.py` mirrors controls and odometry, and adds a SPACE-triggered 360° scan. It expects the ESP to stream LiDAR samples over UDP port `5603` as JSON per sample: `{ "ang_deg": <angle>, "dist_cm": <distance_cm> }` while spinning.
- Run:
```
pip install pynput matplotlib
python MiniTurtleBot\heatmap\heatmap.py --esp-ip 192.168.1.14 --cmd-port 5601 --enc-port 5602 --lidar-port 5603
```
- Press SPACE to start scan; on exit, a heatmap plot of distances is rendered at the robot’s current pose.
