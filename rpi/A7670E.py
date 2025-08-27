#!/usr/bin/env python3

import serial
import pynmea2
import re
import math
import sys


def latlong_displacement_to_xy_meter(latlong1:tuple[float,float],
                                     latlong2:tuple[float,float]
                                     ) -> tuple[float,float]:
    """
    Converts latitude and longitude displacement to
    x (East-West) and y (North-South) displacement in meters.
    """
    lat1, long1 = latlong1
    lat2, long2 = latlong2
    R = 6378137 # Earth radius in meters
    delta_lat = math.radians(lat2 - lat1)
    delta_long = math.radians(long2 - long1)
    lat_avg = math.radians((lat1 + lat2) / 2)
    delta_y = delta_lat * R
    delta_x = delta_long * R * math.cos(lat_avg)
    return delta_x, delta_y


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

    def gnss_up(self):
        try:
            self.send_at_command('AT+CGNSSPWR=1')
            self.send_at_command('AT+CGNSSTST=1')
        except serial.SerialException as e:
            raise Exception(f"Error initializing GNSS: {e}")

    def gnss_down(self):
        try:
            self.send_at_command('AT+CGNSSPWR=0')
            self.send_at_command('AT+CGNSSTST=0')
        except serial.SerialException as e:
            raise Exception(f"Error stopping GNSS: {e}")

    def gnss_params(self):
        try:
            response = str(self.ser.readline(), encoding='utf-8')
            if response.startswith("$GNRMC"):
                return pynmea2.parse(response)
            else:
                return None
        except serial.SerialException as e:
            raise Exception(f"Error reading GNSS params: {e}")

    def gnss_latlong(self):
        response = self.gnss_params()
        if response and re.match(r"^\d+?\.\d+?$", response.lat):
            return float(response.latitude), float(response.longitude)
        else:
            return None

    def close(self):
        return self.ser.close()


if __name__ == '__main__':

    cmds =  {'up', 'echo', 'stream', 'down'}

    try:
        arg = sys.argv[1]
        if arg not in cmds:
            raise Exception('invalid command')
    except Exception:
        print("usage: GNSS <command>")
        print(f"commands: {cmds}")
        exit()

    instance = None
    tmp = None

    try:

        if arg == 'up':
            instance = AT(port='/dev/ttyUSB2')
            instance.gnss_up()

        elif arg in {'echo', 'stream'}:
            instance = AT(port='/dev/ttyUSB3')
            while True:
                latlong = instance.gnss_latlong()
                if latlong:
                    lat, long = latlong
                    print(f"Latitude: {lat:.8f}, Longitude: {long:.8f}")
                    if arg == 'echo':
                        break
                    else:
                        if not tmp:
                            tmp = latlong
                        dlat, dlong = latlong_displacement_to_xy_meter(latlong, tmp)
                        print(f"Displacement X (East-West):   {dlat:.8f}")
                        print(f"Displacement Y (North-South): {dlong:.8f}")
                        tmp = latlong

        elif arg == 'down':
            instance = AT(port='/dev/ttyUSB2')
            instance.gnss_down()

    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Cleaning up before exiting.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if instance:
            instance.close()
