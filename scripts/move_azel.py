#!/usr/bin/env python3
"""
For test of telescope driving
"""
import sys
import time
sys.path.append("/home/exito/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/necst-core/scripts")
import core_controller
import rospy
import datetime

name = "move_azel"

rospy.init_node(name)

antenna = telescope_controller.antenna()

az = antenna.get_az()
el = antenna.get_el()
print("current position : az="+str(az)+" el="+str(el))
az_cmd = input("az = ")
el_cmd = input("el = ")

print("Moving az: "+str(az_cmd)+ ", el: "+str(el_cmd))
#anntena.initilize()
antenna.move_azel(float(az_cmd),float(el_cmd))
#antenna.tracking_check()
#anntena.finalize()
