import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print ("connecting to Drone")
drone = connect("udp:127.0.0.1:14550",baud = 57600, wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Drone connected")
drone.airspeed = 0.3
drone.groundspeed = 0.3
time.sleep(1)

print ("connecting to Rover")
rover = connect("/dev/ttyUSB0",baud = 57600, wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Rover connected")
rover.groundspeed = 0.7
time.sleep(1)

def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    drone.mode    = VehicleMode("GUIDED")
    drone.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not drone.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    drone.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", drone.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
        
def get_location_metres(original_location, dNorth, dEast):

    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

altitude = 4
arm_and_takeoff(altitude)
time.sleep(4)
start = time.perf_counter()
while True:
    lat = rover.location.global_relative_frame.lat # rover's latitude
    lon = rover.location.global_relative_frame.lon # rover's longitude
    location = LocationGlobalRelative(lat, lon, altitude)
    drone.simple_goto(location)
    end = time.perf_counter()
    tmp = end-start
    if tmp >=15:
        print("done")
        break
    time.sleep(2)
time.sleep(15)
drone.mode = VehicleMode("LAND")
while drone.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to land")
print("Landing..")
print("close drone")
drone.close()
print("close rover")
rover.close()

