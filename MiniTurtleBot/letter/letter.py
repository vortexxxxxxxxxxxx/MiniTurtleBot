import socket
from pynput import keyboard

ESP_IP = "192.168.1.14"
UDP_PORT = 5601
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(msg):
    sock.sendto(msg.encode(), (ESP_IP, UDP_PORT))
    print(f"Sent: {msg}")

pressed_keys = set()

def on_press(key):
    if key == keyboard.Key.left and 'left' not in pressed_keys:
        pressed_keys.add('left')
        send_cmd("L1")
    elif key == keyboard.Key.right and 'right' not in pressed_keys:
        pressed_keys.add('right')
        send_cmd("R1")
    elif key == keyboard.Key.up and 'up' not in pressed_keys:
        pressed_keys.add('up')
        send_cmd("F1")
    elif key == keyboard.Key.down and 'down' not in pressed_keys:
        pressed_keys.add('down')
        send_cmd("B1")
    elif key == keyboard.Key.space and 'space' not in pressed_keys:
        # Trigger 360° scan (handled by heatmap firmware or extended sketch)
        pressed_keys.add('space')
        send_cmd("S1")

def on_release(key):
    if key == keyboard.Key.left:
        pressed_keys.discard('left')
        send_cmd("L0")
    elif key == keyboard.Key.right:
        pressed_keys.discard('right')
        send_cmd("R0")
    elif key == keyboard.Key.up:
        pressed_keys.discard('up')
        send_cmd('F0')
    elif key == keyboard.Key.down:
        pressed_keys.discard('down')
        send_cmd('B0')
    elif key == keyboard.Key.esc:
        # ESC exits the program
        return False
    elif key == keyboard.Key.space:
        # optional: mark space released (no S0 needed)
        pressed_keys.discard('space')

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("Use arrow keys to drive; SPACE to scan; ESC to exit.")
    listener.join()