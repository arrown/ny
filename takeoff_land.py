import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
print("connecting to Vehicle")
vehicle= connect("/dev/ttyACM0",baudrate=57600, wait_ready=True)
vehicle.airspeed = 0.3
vehicle.groundspeed = 0.3
time.sleep(1)
def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: "), vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(4)
vehicle.airspeed = 0.3
vehicle.groundspeed = 0.3
time.sleep(3)
vehicle.mode = VehicleMode("LAND")
while vehicle.mode!='LAND':
    time.sleep(1)
    print("Waiting for drone to land")
print("Landing..")
print("close vehicle")
vehicle.close()
