from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import keyboard
from serial import Serial
from pyubx2 import UBXReader


#vehicle = connect('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_320037000C51393239383638-if00', wait_ready=True, baud=115200)
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True, baud=57600)

#ground = connect('/dev/serial/by-id/usb-Hex_ProfiCNC_CubePurple_310045001651393437393433-if00', wait_ready=True, baud=115200)

print(" UAV alt AMSL : %s" % vehicle.location.global_frame.alt)
print(" UAV alt AGL : %s" % vehicle.location.global_relative_frame.alt)
vehicle.airspeed=10
vehicle.mode = VehicleMode("GUIDED")
while(True):
    #global lat, lon, alt
    ubr=UBXReader(Serial('/dev/ttyUSB0',9600))
    raw_data,parsed_data = ubr.read()
    if (parsed_data.identity == "GNGGA"):
        lat = float(parsed_data.lat)
        lon = float(parsed_data.lon)
        alt = vehicle.location.global_relative_frame.alt
        pos = LocationGlobalRelative(lat, lon, alt)
        print (pos)
        vehicle.simple_goto(pos)


    

