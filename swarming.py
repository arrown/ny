import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print("connecting to Drone")
Drone = connect("/dev/serial0", baudrate = 57600, wait_ready = True)
Drone.airspeed = 1
Drone.groundspeed = 0.3
time.sleep(1)
print("conncting to Rover")
Rover = connect("/dev/", baudrate = 57600, wait_ready = True)
Rover.groundspeed = 1
time.sleep(1)
