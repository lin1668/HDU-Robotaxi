#!/usr/bin/env python3
"""Direct UART keyboard control test.

This bypasses ./boot and writes the official car-control frames directly to
/dev/ttyUSB*. Use it only with the wheels off the ground.
"""

import glob
import os
import select
import struct
import sys
import termios
import time
import tty


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


def clamp(value, low, high):
    return max(low, min(high, value))


def read_key(timeout=0.08):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return None


def choose_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        raise SystemExit("No /dev/ttyUSB* or /dev/ttyACM* found")
    return ports[0]


def configure_port(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def write_all(fd, data):
    os.write(fd, data)
    termios.tcdrain(fd)


def main():
    port = choose_port()
    print("Direct serial keyboard control")
    print("Port:", port)
    print("Make sure ./boot and ./icar are stopped, and wheels are off the ground.")
    print("Keys: W/S speed +/- | A/D steer | X/Space stop | C center | B beep | Q quit")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_port(fd)
    old_settings = termios.tcgetattr(sys.stdin)

    speed = 0.0
    servo = SERVO_MID
    try:
        tty.setcbreak(sys.stdin.fileno())
        write_all(fd, buzzer_frame(1))
        write_all(fd, car_frame(0.0, SERVO_MID))
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
                    write_all(fd, buzzer_frame(1))
                elif key == "q":
                    running = False
                print("\rspeed=%+.2f m/s  servo=%d   " % (speed, servo), end="", flush=True)

            now = time.time()
            if now - last_send >= 0.05:
                write_all(fd, car_frame(speed, servo))
                last_send = now

        print("\nStopping...")
        for _ in range(10):
            write_all(fd, car_frame(0.0, SERVO_MID))
            time.sleep(0.05)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        os.close(fd)


if __name__ == "__main__":
    main()
