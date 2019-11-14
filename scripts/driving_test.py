#!/usr/bin/env python3
"""
For test of telescope driving
"""
import sys
import ephem
import time
import numpy
import math
import os
import pylab
import datetime
import matplotlib.pyplot as plt
sys.path.append("/home/exito/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/necst-core/scripts")
import core_controller
import rospy

name = "driving_test"

rospy.init_node(name)

logger = core_controller.logger()
anntena = telescope_controller.antenna()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')

file_name = name + '/' + date + '.necstdb'
print(file_name)

az = anntena.get_az()
el = anntena.get_el()
print("current position : az="+str(az)+" el="+str(el))
az_cmd = input("az = ")
el_cmd = input("el = ")

logger.start(file_name)

time.sleep(3)
print("Moving az: "+str(az_cmd)+ ", el: "+str(az_cmd))
anntena.move_azel(float(az_cmd),float(el_cmd))
antenna.tracking_check()
time.sleep(3)

logger.stop()
