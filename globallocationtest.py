import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print "connecting to Drone"
Drone = connect("/dev/serial0", baudrate = 57600, wait_ready = True) # telemetry line to raspberry pi
Drone.airspeed = 1
Drone.groundspeed = 0.3
time.sleep(1)

print "connecting to Rover"
Rover = connect("/dev/", baudrate = 57600, wait_ready = True) #telemetry receiver
Rover.groundspeed = 1
time.sleep(1)

# swarm code
tmp = y
while tmp == y:
    lat = Drone.location.global_relative_frame.lat # drone's latitude
    lon = Drone.location.global_relative_frame.lon # drone's longitude
    nlat = Rover.location.global_relative_frame.lat # rover's latitude
    nlon = Rover.location.global_relative_frame.lon # rover's longitude

dnorth = (nlat-lat)*rad*math.pi/180
deast = math.pi/180*(nlon-lon)*rad*math.cos(lat/180*math.pi)
print dnorth,deast
