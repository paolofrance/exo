import socket
import threading
import time

# Configuration
UDP_IP = "127.0.0.1"  # Change this if needed
UDP_PORT = 9090
SEND_INTERVAL = 0.1   # in seconds

# Shared state
detection_status = ["not detected"]
running = [True]

# Set up UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def sender():
    while running[0]:
        timestamp = time.time()
        message = f"{timestamp} : {detection_status[0]}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"[SENT] {message}")
        time.sleep(SEND_INTERVAL)

def user_input():
    print("Live UDP status publisher started.")
    print("Type:")
    print("  1 → detected")
    print("  0 → not detected")
    print("  q → quit")

    while running[0]:
        user_in = input("Enter command (1/0/q): ").strip()
        if user_in == "1":
            detection_status[0] = "detected"
        elif user_in == "0":
            detection_status[0] = "not detected"
        elif user_in.lower() == "q":
            running[0] = False
            print("Exiting...")
        else:
            print("Invalid input. Use 1, 0, or q.")

# Start threads
input_thread = threading.Thread(target=user_input)
sender_thread = threading.Thread(target=sender)

input_thread.start()
sender_thread.start()

input_thread.join()
sender_thread.join()
