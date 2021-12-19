import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

print ("connecting to Drone")
drone = connect("/dev/ttyACM0", wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Drone connected")

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
    marker_lat, marker_lon  = get_location_metres(drone.location.global_relative_frame, -0.01*marypos, 0.01*marxpos)
    print(marker_lat,marker_lon)
