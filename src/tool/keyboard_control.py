#!/usr/bin/env python3
"""Keyboard control test for the boot UART bridge.

Run ./boot first, then run this script on the EdgeBoard. It sends the same
control frames as include/com/client.hpp to 127.0.0.1:8899.
"""

import select
import socket
import struct
import sys
import termios
import time
import tty


HOST = "127.0.0.1"
PORT = 8899
FRAME_HEAD = 0xAA
FRAME_TAIL = 0xDD

SERVO_MID = 1500
SERVO_MIN = 1100
SERVO_MAX = 1900
SERVO_STEP = 40

SPEED_STEP = 0.05
SPEED_MAX = 0.35
SPEED_MIN = -0.20


def checksum(data):
    return sum(data) & 0xFF


def car_frame(speed, servo):
    speed_raw = int(speed * 1000.0)
    return struct.pack("<B h H B", FRAME_HEAD, speed_raw, int(servo), FRAME_TAIL)


def buzzer_frame(sound=1):
    return car_frame(0.0, SERVO_MID)


def send_car(sock, speed, servo):
    sock.sendall(car_frame(speed, servo))


def send_stop(sock):
    send_car(sock, 0.0, SERVO_MID)


def read_key(timeout=0.08):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return None


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    print("Keyboard control test")
    print("Make sure wheels are off the ground.")
    print("Keys: W/S speed +/- | A/D steer | X/Space stop | C center | B beep | Q quit")

    speed = 0.0
    servo = SERVO_MID

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        with socket.create_connection((HOST, PORT), timeout=5) as sock:
            sock.settimeout(1)
            print("Connected to boot at %s:%d" % (HOST, PORT))
            sock.sendall(buzzer_frame(1))
            send_stop(sock)

            last_send = 0.0
            running = True
            while running:
                key = read_key()
                if key:
                    key = key.lower()
                    if key == "w":
                        speed = clamp(speed + SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    elif key == "s":
                        speed = clamp(speed - SPEED_STEP, SPEED_MIN, SPEED_MAX)
                    elif key == "a":
                        servo = clamp(servo + SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    elif key == "d":
                        servo = clamp(servo - SERVO_STEP, SERVO_MIN, SERVO_MAX)
                    elif key == "c":
                        servo = SERVO_MID
                    elif key in ("x", " "):
                        speed = 0.0
                        servo = SERVO_MID
                    elif key == "b":
                        sock.sendall(buzzer_frame(1))
                    elif key == "q":
                        running = False

                    print("\rspeed=%+.2f m/s  servo=%d   " % (speed, servo), end="", flush=True)

                now = time.time()
                if now - last_send >= 0.05:
                    send_car(sock, speed, servo)
                    last_send = now

            print("\nStopping...")
            for _ in range(10):
                send_stop(sock)
                time.sleep(0.05)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
