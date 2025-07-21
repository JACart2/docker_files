"""
    wasd_teleop.py - Used to send keyboard-commands to cart_teleop.ino for manual control of the cart

    Author: Brandon Parr
    Version: 1.1 (Refined)
"""

import curses
import sys
import math
import serial  # pySerial package
import time
import bitstruct
import numpy as np
from serial.tools.list_ports import comports
from sys import platform


def auto_detect_port():
    """
    Automatically detect the serial port for the Arduino on all platforms.
    Returns the port name as a string, or None if not found.
    """
    # Try to find a port with 'usbserial', 'USB', 'ACM', or 'COM' in the name
    candidates = []
    for port in comports():
        port_name = port.device
        desc = port.description.lower()
        if (
            'usbserial' in port_name.lower() or
            'usb' in port_name.lower() or
            'acm' in port_name.lower() or
            'arduino' in desc or
            'ch340' in desc or
            'cp210' in desc or
            'com' in port_name.lower()
        ):
            candidates.append(port_name)
    if candidates:
        return candidates[0]  # Return the first likely candidate
    return None


class Teleop:
    """
    Teleoperation class for manual cart control via keyboard.
    """
    MAX_SPEED = 255
    MIN_ANGLE = 0
    MAX_ANGLE = 100

    def __init__(self, port):
        self.cur_vel = 0  # (0 - 255)
        self.cur_angle = 50  # 50 is middle, (0 - 100)
        self.prev_key = 1
        self.cart_ser = None
        try:
            self.cart_ser = serial.Serial(port, 57600, write_timeout=0)
            print(f"Connected to Arduino on port: {port}")
        except Exception as e:
            print(f"ERROR: Could not connect to Arduino on port {port}: {e}")
            sys.exit(1)
        curses.wrapper(self.get_input)

    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, min(256, curses.COLORS)):
            curses.init_pair(i, i, -1)
        stdscr.nodelay(True)
        stdscr.addstr(0, 0, 'Move with WASD, Z for brake, X for hard stop and Y for centering the wheel.')
        stdscr.addstr(1, 0, 'CTRL-C to exit')
        stdscr.addstr(7, 0, 'Throttle val:')
        stdscr.addstr(8, 0, 'Brake val:')
        stdscr.addstr(9, 0, 'Steering val:')
        while True:
            try:
                keyval = stdscr.getch()
                if keyval == ord('w'):
                    self.cur_vel = min(self.MAX_SPEED, self.cur_vel + 15)
                elif keyval == ord('a'):
                    self.cur_angle = max(self.MIN_ANGLE, self.cur_angle - 5)
                elif keyval == ord('s'):
                    self.cur_vel = max(0, self.cur_vel - 15)
                elif keyval == ord('d'):
                    self.cur_angle = min(self.MAX_ANGLE, self.cur_angle + 5)
                elif keyval == ord('y'):
                    self.cur_angle = 50
                elif keyval == ord('x'):
                    self.cur_vel = 0
                elif keyval == ord('z'):
                    self.brake(self.cur_vel / 255.0 * 3, stdscr)
                    self.cur_vel = 0
                self.send_cmd(self.cur_vel, 0, self.cur_angle, stdscr)
                self.prev_key = keyval
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nExiting teleop...")
                break
            except Exception as e:
                stdscr.addstr(11, 0, f"Error: {e}    ")
                time.sleep(0.5)

    def brake(self, delay, stdscr):
        rate = 10.
        steps = max(1, int(delay * rate))
        for brake in np.linspace(0, 255, steps, endpoint=True):
            self.send_cmd(0, int(brake), self.cur_angle, stdscr)
            time.sleep(1. / rate)
            stdscr.getch()

    def send_cmd(self, throttle, brake, steering, stdscr):
        data = bytearray(5)
        bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, throttle, brake, steering)
        try:
            self.cart_ser.write(data)
        except Exception as e:
            stdscr.addstr(12, 0, f"Serial write error: {e}    ")
        stdscr.addstr(7, 0, f'Throttle val: {throttle}   ')
        stdscr.addstr(8, 0, f'Brake val:    {brake}   ')
        stdscr.addstr(9, 0, f'Steering val: {steering}   ')


def main():
    port = auto_detect_port()
    if not port:
        print("ERROR: Could not auto-detect Arduino serial port. Please check your connection.")
        sys.exit(1)
    print(f"Using serial port: {port}")
    Teleop(port)


if __name__ == "__main__":
    main()
