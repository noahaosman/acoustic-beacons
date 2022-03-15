#!/usr/bin/env python3
from threading import Thread
import itertools
import sys
import serial
import time
import yaml
import numpy as np
import pynmea2
import requests
from datetime import datetime
from classes.mlat_solver import Mlat

# Parse config file
config_file = "/home/pi/nav/config.yaml"
config = yaml.safe_load(open(config_file))

# Get general settings
settings = {
    'sound_speed': float(config['settings']['sound_speed']),
    'repeat_rate': float(config['settings']['repeat_rate']),
    'range_rate': float(config['settings']['range_rate']),
    'broadcast_rate': float(config['settings']['broadcast_rate']),
    'reply_timeout': float(config['settings']['reply_timeout']),
    'randomize': float(config['settings']['randomize']),
    'pressure_rate': float(config['settings']['pressure_rate']),
}


class Modem:
    """A class for communications with Delphis Subsea Modems"""

    # ======================================================
    # Process config file
    # ======================================================
    def __init__(self, mode=None, args=None):

        # UNTESTED
        self.port = '/dev/ttyBeacon' or config['modems'][self.address]['serial_beacon']

        # Open serial connection to modem
        self.ser = serial.Serial(
            port=self.port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        # Verify modem status
        status_msg = self.status()
        self.address = status_msg['src']
        print("Connected to modem %d, voltage: %.2fV" % (self.address, status_msg['voltage']))

        # Fall back to mode in config file if mode not set
        self.mode = mode or config['modems'][self.address]['mode']
        print("Starting in %s mode" % self.mode)
        self.args = args

        # Open GPS serial port if configured
        self.has_gps = False
        if 'serial_gps' in config['modems'][self.address]:
            print("Opening GPS serial port")
            self.ser_gps = serial.Serial(
                port=config['modems'][self.address]['serial_gps'],
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            if self.ser_gps.is_open:
                self.has_gps = True

        # Open pressure serial port if configured
        # UNTESTED
        self.has_pressure = False
        if 'serial_pressure' in config['modems'][self.address]:
            print("Opening pressure serial port")
            self.ser_pressure = serial.Serial(
                port=config['modems'][self.address]['serial_pressure'],
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            if self.ser_pressure.is_open:
                self.has_pressure = True

        # Define passive beacons
        self.passive_beacons = [int(m) for m in config['modems'].keys()
                                if config['modems'][m]['mode'] == 'passive']

        # Initialize multilateration solver
        self.mlat = Mlat(config)

        # If stations were configured with local coordinates, convert them to a
        # local lat,lon coordinate system
        if config['settings']['coords'] == 'local':
            for m in self.passive_beacons:
                x = config['modems'][m]['x']
                y = config['modems'][m]['y']
                lat, lon = self.mlat.local2gps(x, y)
                config['modems'][m]['lat'] = lat
                config['modems'][m]['lon'] = lon

        # Initialize dictionaries for passive beacon locations & distances
        self.locs = {m: {'lat': config['modems'][m]['lat'],
                         'lon': config['modems'][m]['lon'],
                         'z':   config['modems'][m]['z']}
                     for m in self.passive_beacons}
        self.dists = {m: None for m in self.passive_beacons}

        # Get initial position from config file
        self.lat = None
        self.lon = None
        self.z = None
        if self.mode == "passive":
            self.lat = config['modems'][self.address]['lat']
            self.lon = config['modems'][self.address]['lon']
            self.z = config['modems'][self.address]['z']

    # ======================================================
    # Low-level modem commands
    # ======================================================
    def send(self, cmd=None, wait=False, n=1, prefix=None):
        "Send a command, optionally wait for the Nth response"

        # If a command is specified and wait==False, send the command and
        # immediately return.
        if cmd and not wait:
            self.ser.write(cmd.encode())
            return None

        # If wait==True, then:
        # - Send the command (if a command is given)
        # - Wait for the nth response with the desired prefix(es)
        #   - If no prefix(es) is specified,
        #     wait for nth response with any prefix.
        elif wait:
            t0 = time.time()
            c = 0
            while c < n and time.time() - t0 < settings['reply_timeout']:
                # Send command until at least 1 response is recieved,
                # then keep listening until nth response.
                if cmd and c == 0:
                    self.ser.write(cmd.encode())

                # If we get a response with the right prefix, increment the counter
                response = self.ser.readline().decode().strip()
                if response and ((not prefix) or (response[1] in prefix)):
                    c += 1
                time.sleep(settings['repeat_rate'])
            return parse_message(response)

    def status(self):
        "Query node address and voltage"
        cmd = "$?"
        return self.send(cmd=cmd, wait=True, prefix="A")

    def set_address(self, address):
        "Set node address"
        cmd = "$A%03d" % (address)
        return self.send(cmd=cmd, wait=True, prefix="A")

    def broadcast(self, message, wait=False):
        "Send message to all units in range"
        cmd = "$B%02d%s" % (len(message), message)
        return self.send(cmd=cmd, wait=wait)

    def unicast(self, message, target, wait=False):
        "Send message to target unit (specified by 3-digit integer)"
        cmd = "$U%03d%02d%s" % (target, len(message), message)
        return self.send(cmd=cmd, wait=wait)

    def ping(self, target, wait=False):
        "Send range ping to target unit"
        cmd = "$P%03d" % (target)
        return self.send(cmd=cmd, prefix=["P", "R"], n=2, wait=wait)

    # ======================================================
    # Processing threads
    # ======================================================
    # We should be fine to run any number of threads as long as:
    # - No two threads try to write to the same serial port at the same time
    # - No two threads try to read from the same serial port at the same time
    #
    # I've tried to set this up so any given thread only reads or only writes,
    # and only to one serial port.

    def active_ping(self):
        "Cyclically loop over passive beacons and send ranging pings"
        # Writes to acoustic modem serial port
        t0 = time.time() - settings['range_rate']
        for target in itertools.cycle(self.passive_beacons):
            while time.time() - t0 <= settings['range_rate']:
                time.sleep(0.005)
            self.ping(target, wait=False)
            t0 = time.time() + rand()

    def active_listen(self):
        "Parse ranging returns and broadcasts, update positions & distances"
        while self.ser.is_open:
            msg_str = self.ser.readline().decode().strip()
            msg = parse_message(msg_str)

            # Update position or distance from passive beacon
            if msg:
                if msg['type'] == 'broadcast':
                    if is_hex(msg['str']):
                        # Decode the lat/lon message
                        lat, lon = decode_ll(msg['str'])
                        # Update the location entry for the source beacon
                        self.locs[msg['src']]['lat'] = lat
                        self.locs[msg['src']]['lon'] = lon
                        print("%d is at %.5fN,%.5fE" % (msg['src'], lat, lon), flush=True)
                elif msg['type'] == 'range':
                    # Update the distance entry for the source beacon
                    self.dists[msg['src']] = msg['range']
                    print("%.2f m from %d" % (msg['range'], msg['src']), flush=True)

                # Pass positions & distances to multilateration solver
                if len(self.passive_beacons) > 2:

                    # Use previous position as initial guess if possible
                    if not (self.lat and self.lon and self.z):
                        x0 = None
                    else:
                        x0 = np.array((self.lat, self.lon, self.z))

                    # Estimate position
                    [lat, lon, z] = self.mlat.solve(self.locs, self.dists, x0=x0)

                    # TODO: overwrite z with pressure
                    # if self.has_pressure:
                    #     self.z = scale*self.pressure
                    # TODO: Do something with this,
                    #    like sending to the ROV brain

    def monitor_gps(self):
        """Parse all incoming GPS messages and update position
        UNTESTED
        """
        # Reads from GPS serial port
        while self.ser_gps.is_open:

            # FIXME: This line sometimes fails with the following error, might
            # need to change something here:
            #
            #     Traceback (most recent call last):
            #       File "/usr/lib/python3.9/threading.py", line 954, in _bootstrap_inner
            #         self.run()
            #       File "/usr/lib/python3.9/threading.py", line 892, in run
            #         self._target(*self._args, **self._kwargs)
            #       File "/home/pi/nav/code/classes/modem.py", line 200, in monitor_gps
            #         msg_str = self.ser_gps.readline().decode().strip()
            #     UnicodeDecodeError: 'utf-8' codec can't decode byte 0xfc in position 0: invalid start byte
            #
            # CHANGED: added arguments to the decode to hopefully circumvent the above error
            msg_str = self.ser_gps.readline().decode('utf-8', 'ignore').strip()
            if msg_str:
                print(msg_str, flush=True)
                if len(msg_str) > 5:
                    if msg_str[0:6] == "$GPGGA":
                        try:
                            parsed = pynmea2.parse(msg_str)
                            if parsed.latitude & parsed.longitude:
                                self.lat = parsed.latitude
                                self.lon = parsed.longitude
                        except Exception as e:
                            print('Unable to parse GPS GGA message.')

    def monitor_rov_pressure(self):
        """ Request ROV pressure (hPa) from PixHawk
        UNTESTED
        """
        period = settings['pressure_rate']
        api_url = "http://192.168.2.2:4777/mavlink/SCALED_PRESSURE2/press_abs"
        while True:
            response = requests.get(api_url)
            try:
                output = response.json()
                print(output)
                self.pressure = output[‘press_abs’]
                self.z = pressure_2_depth(self.pressure)
                self.has_pressure = True
            except Exception as e:
                print('Unable to parse pressure from API.')
            time.sleep(period)

    def monitor_ser_pressure(self):
        """ Collect pressure data from serial data feed
        UNTESTED
        """
        while self.ser_pressure.is_open:
            msg_str = self.ser_pressure.readline().decode().strip()
            if msg_str:
                print(msg_str, flush=True)
                # TODO: parse pressure messages and assign pressure
                # self.pressure = ...
                # self.z = pressure_2_depth(self.pressure)

    def monitor_pressure(self):
        """ Wrapper for the various pressure data sources."""
        if config['modems'][self.address]['rov']:
            self.monitor_rov_pressure(self)
        elif self.ser_pressure.is_open:
            self.monitor_ser_pressure(self)
        else:
            print('No pressure available.')

    def passive_broadcast(self):
        "Periodically broadcast current position"
        while self.ser.is_open:
            msg = encode_ll(self.lat, self.lon)
            self.broadcast(msg)
            time.sleep(settings['broadcast_rate'] + rand())

    def debug_report(self):
        "Parse all incoming modem messages"
        # Reads from acoustic modem serial port
        while self.ser.is_open:
            msg_str = self.ser.readline().decode().strip()
            msg = parse_message(msg_str)
            if msg:
                print(msg)

    def debug_timer(self):
        "Periodically broadcast the current time"
        period = float(self.args[0]) - settings['rate']
        target = len(self.args) > 1 and int(self.args[1]) or None
        while self.ser.is_open:
            current_time = datetime.now().strftime("%H:%M:%S")
            if target:
                self.unicast(current_time, target)
            else:
                self.broadcast(current_time)
            time.sleep(period)

    # ======================================================
    # Main loop
    # ======================================================
    def run(self, mode):

        # Set address and exit if in "set" mode
        if mode == "set":
            address = int(self.args[0])
            self.set_address(address)

        # Define threads, but don't start any
        ping_thread = Thread(target=self.active_ping)
        listen_thread = Thread(target=self.active_listen)
        gps_thread = Thread(target=self.monitor_gps)
        broadcast_thread = Thread(target=self.passive_broadcast)
        pressure_thread = Thread(target=self.monitor_pressure)

        # Threads for debugging
        report_thread = Thread(target=self.debug_report)
        timer_thread = Thread(target=self.debug_timer)

        if mode == "active":
            ping_thread.start()
            listen_thread.start()
            pressure_thread.start()

        elif mode == "passive":
            broadcast_thread.start()
            gps_thread.start()
            pressure_thread.start()

        elif mode == "timer":
            timer_thread.start()
            report_thread.start()

        elif mode == "report":
            report_thread.start()

# =========================================================================
# Helper functions
# =========================================================================


def parse_message(msg_str):
    "Parse a raw message string and return a useful structure"
    if not msg_str:
        return None

    # Get message prefix and initialize output
    prefix = msg_str[1]
    msg = {}

    # Status: #AxxxVyyyyy in response to "$?" (query status)
    #      or #Axxx       in response to "$Axxx" (set address)
    if prefix == "A":
        msg['type'] = "status"
        msg['src'] = int(msg_str[2:5])  # xxx
        if len(msg_str) > 5:
            msg['voltage'] = float(msg_str[6:])*15/65536  # yyyyy...
        else:
            msg['voltage'] = None

    # Broadcast: #Bxxxnnddd... (broadcast recieved)
    #            #Bnn          (self broadcast acknowledge)
    elif prefix == "B":
        if len(msg_str) > 4:
            msg['type'] = "broadcast"
            msg['src'] = int(msg_str[2:5])  # xxx
            msg['str'] = msg_str[7:]  # ddd...
        else:
            msg['type'] = "broadcast_ack"
            msg['len'] = int(msg_str[2:])

    # Unicast: #Unnddd...
    elif prefix == "U":
        msg['type'] = "unicast"
        msg['src'] = None
        msg['str'] = msg_str[4:]  # ddd...

    # Range: RxxxTyyyyy
    elif prefix == "R":
        msg['type'] = "range"
        msg['src'] = int(msg_str[2:5])
        msg['range'] = settings['sound_speed'] * 3.125e-5 * float(msg_str[6:11])

    # Note: Other message types are possible, but we don't currently use any of
    #       them. Return None if we encounter these.
    else:
        return None

    # Return message structure
    return msg


def pressure_2_depth(P_hPa):
    """ Converts pressure (hPa) to depth (m)
    using the hydrostatic water pressure formula.
    UNTESTED
    """

    P = P_hPa * 100   # convert hPa to Pa

    g = 9.80665  # m/s2, acceleration of gravity

    medium = config['settings']['medium']
    if medium == 'saltwater':
        rho = 1023.6  # kg/m3, saltwater density
    else:
        rho = 997.0474  # kg/m3, freshwater density

    z = P / (rho * g)  # depth in meters
    return z


def encode_decimal_deg(deg):
    "Encode decimal lat or lon to hexidecimal degrees, minutes, seconds"
    # Output string looks like:  [DD] [MM] [SSS]  [N]
    #                              |    |    |     |
    #                           Degrees | Seconds  |
    #                                 Minutes    1 if negative
    neg = deg < 0
    deg = abs(deg)
    mins = (deg-np.floor(deg))*60
    secs = (mins-np.floor(mins))*int('fff', 16)
    return "%02x%02x%03x%1x" % (int(np.floor(deg)),
                                int(np.floor(mins)),
                                int(np.floor(secs)),
                                neg)


def decode_hex_dms(dms):
    "Decode hexidecimal degrees,mins,secs to decimal degrees"
    degs = int(dms[0:2], 16)
    mins = int(dms[2:4], 16)
    secs = int(dms[4:7], 16)*60/int('fff', 16)
    neg = bool(int(dms[7]))
    dec = degs + mins/60 + secs/60**2
    return neg and -1*dec or dec


def encode_ll(lat, lon):
    return encode_decimal_deg(lat) + encode_decimal_deg(lon)


def decode_ll(hex_str):
    return decode_hex_dms(hex_str[0:8]), decode_hex_dms(hex_str[8:])


def is_hex(s):
    try:
        int(s, 16)
        return True
    except ValueError:
        return False


def rand():
    return settings['randomize'] * np.random.random()
