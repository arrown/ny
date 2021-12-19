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

def goto_position_target_local_ned(north, east, down):

    msg = rover.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    rover.send_mavlink(msg)
    

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        drone.send_mavlink(msg)
        time.sleep(1)
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
        
altitude = 4
arm_and_takeoff(altitude)
time.sleep(4)
goto_position_target_local_ned(2,0)
duration = round(end-start)
time.sleep(4)
vx = rx/tdu
vy = ry/tdu
vz = 0
time.sleep(1)
send_ned_velocity(vx, vy, vz, duration)
time.sleep(4)
goto_rover(rx,ry,rz)
time.sleep(4)
drone.mode = VehicleMode("LAND")
while drone.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to land")
print("Landing..")
print("close drone")
drone.close()
print("close rover")
rover.close()
