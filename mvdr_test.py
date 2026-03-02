import socket
import sys

SOCKET_PATH = "/tmp/mvdr_ipc.sock"

def send_cmd(cmd):
    try:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        client.sendto(cmd.encode('utf-8'), SOCKET_PATH)
        client.close()
        print(f"  -> Sent: {cmd}")
    except FileNotFoundError:
        print(f"  -> Error: C++ Engine not running (Socket {SOCKET_PATH} not found).")

print("=== MVDR Manual Pilot ===")
print("Act as the neural network. Send commands to the C++ engine.")
print("Commands:")
print("  1 - Send 'VAD 1'  (Starts DOA Steering)")
print("  0 - Send 'VAD 0'  (Stops Steering, learns background noise)")
print("  L - Send 'LOCK'   (Evaluates the 1.6s history buffer and locks)")
print("  R - Send 'RESET'  (Unlocks and returns to AUTO)")
print("  Q - Quit")
print("-" * 25)

while True:
    cmd = input("Pilot Command [1, 0, L, R, Q]: ").strip().upper()
    
    if cmd == '1':
        send_cmd("VAD 1")
    elif cmd == '0':
        send_cmd("VAD 0")
    elif cmd == 'L':
        send_cmd("LOCK")
    elif cmd == 'R':
        send_cmd("RESET")
    elif cmd == 'Q':
        print("Exiting pilot.")
        sys.exit(0)
    else:
        print("  -> Invalid command.")
