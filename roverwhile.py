import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
print ("connecting to Drone")
drone = connect("udp:127.0.0.1:14551",baudrate = 57600, wait_ready = True)
drone.airspeed = 0.3
drone.groundspeed = 0.7
time.sleep(1)

print ("connecting to Rover")
rover = connect("/dev/ttyUSB0",baudrate = 57600, wait_ready = True)
rover.groundspeed = 0.7
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

while True:
    lat = drone.location.global_relative_frame.lat # drone's latitude
    lon = drone.location.global_relative_frame.lon
    location = LocationGlobalRelative(lat, lon,0)
    rover.simple_goto(location)
    time.sleep(0.5)
