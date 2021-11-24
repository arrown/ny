import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

print ("connecting to Rover")
rover = connect("/dev/ttyUSB0",baud = 57600, wait_ready = True,timeout = 120, heartbeat_timeout=120)
print("Rover connected")
rover.groundspeed = 0.7
time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):

    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth)Radius*math.com(math.pi*original_locaiton.lat/180))

    newlat = original_location.lat +(dLat*180/math.pi)
    newlon = original_location.lon + (dLon*180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation= LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat = aLocation1.lat
    dlong = aLocation2.lon = aLocation1.lon
    return math.sqrt((dlat*dlat)+(dlong*dlong)) * 1.113195e5

def goto_rover(dNorth, dEast, gotoFunction=rover.simple_goto):

    currentLocation = rover.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    while rover.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name

        remainingDistance=get_distance_metres(rover.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.2: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)


rover.mode = VehicleMode("GUIDED")

while rover.mode!='GUIDED':
	print("Waiting for change mode")
	time.sleep(1)
print("Rover in GUIDED MODE")

goto_rover(1,0)
time.sleep(2)
print("close vehicle")
rover.close()
