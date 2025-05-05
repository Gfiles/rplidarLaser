from time import sleep
import math
import serial
import sys
import os
import json
from adafruit_rplidar import RPLidar #pip install Adafruit_CircuitPython_RPLIDAR
from pythonosc.udp_client import SimpleUDPClient # pip install python-osc
from tcpServer_part import TCPServer
import pynput  # pip install pynput

# ---------- Classes ------------
class Zone:
    def __init__(self, angle, distance, radius, in_event, on_event, out_event):
        self.angle = angle
        self.distance = distance
        self.radius = radius
        self.in_event = in_event
        self.on_event = on_event
        self.out_event = out_event

    def __str__(self):
        return f"Zone at ang: {self.angle}, dist: {self.distance} with radius {self.radius}mm"

# ---------- Functions ----------
def readConfig(settingsFile):
    if os.path.isfile(settingsFile):
        with open(settingsFile) as json_file:
            data = json.load(json_file)
    else:
        data = {
                "uartPort": "COM4",
                "uartSpeed": 19200,
                "debug": False,
                "minDist": 100,
                "maxDist": 4000,
                "minAng": -90,
                "maxAng": 90,
                "touchWidth": 4000,
                "touchHeight": 2250,
                "widthOffset": 0,
                "heightOffset": 0,
                "angleOffset": 0,
                "time2Scan": 0.1,
                "sendSpeed": 1,
                "minSize": 20,
                "maxSize": 300,
                "oscPort": 9000,
                "oscServer": "192.168.60.159",
                "oscAddress": "/zones",
                "outputType": "Keyboard",
                "inputType": "Zones",
                "manual_zones": [
                    {
                        "name": "Left",
                        "angle": -30.0,
                        "distance": 400.0,
                        "radius": 200.0,
                        "in_event": "a",
                        "on_event": "b",
                        "out_event": "c"
                    },
                    {
                        "name": "Right",
                        "angle": 30.0,
                        "distance": 1000.0,
                        "radius": 200.0,
                        "in_event": "d",
                        "on_event": "e",
                        "out_event": "f"
                    }
                ]
                
            }
        # Serializing json
        json_object = json.dumps(data, indent=4)

        # Writing to config.json
        with open(settingsFile, "w") as outfile:
            outfile.write(json_object)
    return data

def filter_scan_points_within_zones(scan, zone):
    """
    Filters the scan points to include only those within any of the given zones.
    :param scan: iterable of points, each point is a tuple (_, angle, distance)
    :param zones: list of Zone objects
    :return: dictionary with keys as point indices and values as point tuples within zones
    """
    filtered_points = {}
    for idx, point in enumerate(scan):
        _, ang, dist = point
        if ang > 180:
            ang = ang - 360
        distance_to_zone_center = distBetweenPolar(dist, ang, zone.distance, zone.angle)
        if distance_to_zone_center < zone.radius:
            filtered_points[idx] = point
            break
    return filtered_points


def connectRadar():
    #Clean Range Finder settings
    try:
        #print(laser.get_sensor_state())
        laserHealth = laser.health
        
        print(f"infor = {laser.info}")
        if laserHealth[0] == "Good":
            return True
        else:
            print(f"health = {laserHealth[0]}")
            return False
    except:
        laser.stop()
        laser.start_motor()
        laser.disconnect()
        print("Error Conecting to Laser, Reseting")
        return False

def distBetweenPolar(r1, theta1, r2, theta2):
    """
    Calculate distance between two points in polar coordinates (r, theta in degrees)
    """
    # Convert degrees to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    # Convert polar to cartesian
    x1 = r1 * math.cos(theta1_rad)
    y1 = r1 * math.sin(theta1_rad)
    x2 = r2 * math.cos(theta2_rad)
    y2 = r2 * math.sin(theta2_rad)
    # Calculate Euclidean distance
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# ---------- Main Program ----------
try:
    this_file = __file__
except NameError:
    this_file = sys.argv[0]
this_file = os.path.abspath(this_file)
if getattr(sys, 'frozen', False):
    cwd = os.path.dirname(sys.executable)
else:
    cwd = os.path.dirname(this_file)

# Get the current working
# directory (CWD)
#print("Current working directory:", cwd)

#Read Configuration File
settingsFile = os.path.join(cwd, "appconfig.json")
config = readConfig(settingsFile)
uartPort = config["uartPort"]
uartSpeed = config["uartSpeed"]
time2Scan = config["time2Scan"]
outputType = config["outputType"]
oscPort = int(config["oscPort"])
oscServer = config["oscServer"]
debug = config["debug"]
oscAddress = config["oscAddress"]

# Load manual zones from config if present
manual_zones = []
if "manual_zones" in config:
    for zone_data in config["manual_zones"]:
        zone = Zone(
            angle=zone_data.get("angle", 0),
            distance=zone_data.get("distance", 0),
            radius=zone_data.get("radius", 0),
            in_event=zone_data.get("in_event", ""),
            on_event=zone_data.get("on_event", ""),
            out_event=zone_data.get("out_event", "")
        )
        manual_zones.append(zone)
else:
    manual_zones = []

# Laser Settings
# Setup the RPLidar
laser = RPLidar(None, uartPort, timeout=3)
print(laser)
laserOn = False

# Connect to laser
while not laserOn: # Block Program if Not Conecting to Laser
    laserOn = connectRadar()

print(f"Output Type: {outputType}")
if outputType == "OSC":
    #OSC Client Connection
    oscClient = SimpleUDPClient(oscServer, oscPort)
elif outputType == "TCP":
    #TCP Server Connection
    tcpServer = TCPServer(port=oscPort)
    tcpServer.start()


if outputType == "Keyboard":
    print("Keyboard will be activatedin 20 seconds.")
    sleep(20)  # Give time to switch to the target application
    print("Keyboard mode activated. Ready to send keypresses.")
    keyboard = pynput.keyboard.Controller()

print("laser Ready")
#Create zone state list
last_zone_states = [None for _ in range(len(manual_zones))]
zone_states = [None for _ in range(len(manual_zones))]
try:
    while laserOn:
        # Read laser data
        if debug:
            print("zone_states")
            print(last_zone_states)
        
        for scan in laser.iter_scans():
            #print(scan)
            #print("-----Filteresd Scan-----")
            for i, zone in enumerate(manual_zones):
                filtered = filter_scan_points_within_zones(scan, zone)
                #print(f"Zone {i}: {filtered}")
                #print(f"{zone_states[i]} - {last_zone_states[i]}")
                if len(filtered) > 0:
                    #print(f"{zone.in_event} - {zone.on_event} - {zone.out_event}")
                    if last_zone_states[i] is None:
                        zone_states[i] = zone.in_event
                    elif last_zone_states[i] is not None:
                        zone_states[i] = zone.on_event
                else:
                    if last_zone_states[i] is zone.on_event:
                        zone_states[i] = zone.out_event
                    elif last_zone_states[i] is zone.out_event:
                        zone_states[i] = None
                last_zone_states[i] = zone_states[i]
                if zone_states[i] is not None:
                    if debug:
                        print(f"{zone} - {zone_states[i]}")
                    if outputType == "OSC":
                        print(f"Sending OSC message: {zone_states[i]}")
                        #print(f"{zone_states[i]} - {last_zone_states[i]}")
                        oscClient.send_message(oscAddress, zone_states[i])
                    elif outputType == "TCP":
                        print(f"Sending TCP message: {zone_states[i]}")
                        tcpServer.send(zone_states[i].encode())
                    elif outputType == "Keyboard":
                        # Simulate keypresses of the message string
                        if len(zone_states[i]) == 1:
                            keyboard.press(zone_states[i])
                            keyboard.release(zone_states[i])
                        else:
                            keyboard.write(zone_states[i])
            sleep(time2Scan)
except KeyboardInterrupt as error:
    print("An exception occurred:", error)
    laser.stop()
    laser.stop_motor()
    laser.disconnect()   
    print("Laser turned off and serial port closed.")
    if outputType == "TCP":
        tcpServer.stop()