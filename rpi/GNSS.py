#!/usr/bin/env python3

import time
import serial
import pynmea2
import re
from sys import argv


class AT:

    def __init__(self, port='/dev/ttyAMA0', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            raise Exception(f"Error opening serial connection: {e}")

    def send_at_command(self, command, wait_for_response=True):
        try:
            encoded_command = command.encode('utf-8') + b'\r\n'
            self.ser.write(encoded_command)
            print(f"Sent command: {command}")
            if wait_for_response:
                response = self.ser.readlines()[1].decode('utf-8').strip()
                print(f"Response: {response}")
                return response
            else:
                return None
        except serial.SerialException as e:
            raise Exception(f"Error sending AT command: {e}")

    def initialize_gnss(self):
        try:
            self.send_at_command('AT+CGNSSPWR=1')
            self.send_at_command('AT+CGNSSTST=1')
        except serial.SerialException as e:
            raise Exception(f"Error initializing GNSS: {e}")

    def gnss_params(self):
        try:
            response = str(self.ser.readline(), encoding='utf-8')
            if response.startswith("$GNRMC"):
                rmc = pynmea2.parse(response)
                return rmc
            else:
                return None
        except serial.SerialException as e:
            raise Exception(f"Error reading GNSS params: {e}")

    def gnss_latlong(self):
        pass

    def close(self):
        return self.ser.close()


if __name__ == '__main__':

    try:
        cmds =  {'init', 'echo', 'stream', 'lat,long'}
        arg = argv[1]
        if arg not in cmds:
            raise Exception('invalid command')
    except Exception:
        print("usage: GNSS <command>")
        print(f"commands: {cmds}")
        exit()

    try:

        if arg == 'init':
            instance = AT(port='/dev/ttyUSB2')
            instance.initialize_gnss()
            instance.ser.close()

        elif arg == 'echo' or arg == 'stream':
            instance = AT(port='/dev/ttyUSB3')
            while True:
                response = instance.gnss_params()
                if response:
                    if re.match(r"^\d+?\.\d+?$", response.lat) is not None:
                        print(f"Latitude: {response.latitude:.8f}, Longitude: {response.longitude:.8f}")
                    else:
                        print(f'[WARNING: could not parse]: {response}')
                    if arg == 'echo':
                        break

    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Cleaning up before exiting.")
        instance.close()

    except Exception as e:
        print(f"An error occurred: {e}")
