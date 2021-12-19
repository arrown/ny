import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

print ("connecting to Drone")
drone = connect("/dev/ttyACM0", wait_ready = True, timeout = 120, heartbeat_timeout=120)
print ("Drone connected")
drone.airspeed = 0.3
drone.groundspeed = 0.3
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
  
id_to_find = 0
marker_size = 20
calib_path  = ""
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
horizontal_res = 320
vertical_res = 240
horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters  = aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
fps = 1


altitude = 4 # target altitude
arm_and_takeoff(altitude) # take off
time.sleep(4)

while True:

    ret, frame = cap.read()
    
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    
    if ids is not None and ids[0] == id_to_find:
        
        ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
        (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
        x = '{:.2f}'.format(tvec[0])
        y = '{:.2f}'.format(tvec[1])
        z = '{:.2f}'.format(tvec[2])
            
        y_sum = 0
        x_sum = 0
            
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
        x_avg = x_sum*.25
        y_avg = y_sum*.25
            
        x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
        y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
        
        marxpos = (x_avg-160)*tvec[2]*math.tan(horizontal_fov/2)/160
        marypos = (y_avg-120)*tvec[2]*math.tan(vertical_fov/2)/120
        location = drone.location.global_relative_frame

        marker_lat, marker_lon  = get_location_metres(location, -0.01*marypos, 0.01*marxpos)
        altitude = altitude*0.9
        location_marker = LocationGlobalRelative(marker_lat, marker_lon, altitude)
        drone.simple_goto(location_marker)
        if altitude <= 1:
            drone.mode = VehicleMode("LAND")
            while drone.mode!='LAND':
                time.sleep(1)
                print("Waiting for drone to land")
    elif ids is None:
        print("marker is not detected")
        iland = input("do you want to land?")
        if iland == "y":
            drone.mode = VehicleMode("LAND")
            while drone.mode!='LAND':
                time.sleep(1)
                print("Waiting for drone to land")

print("landing..")                
print("close drone")
drone.close()
