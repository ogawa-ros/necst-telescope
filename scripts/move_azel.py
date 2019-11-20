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

logger = core_controller.logger()
antenna = telescope_controller.antenna()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

az = antenna.get_az()
el = antenna.get_el()
print("current position : az="+str(az)+" el="+str(el))
az_cmd = input("az = ")
el_cmd = input("el = ")

logger.start(file_name)
time.sleep(1)
print("Moving az: "+str(az_cmd)+ ", el: "+str(el_cmd))
anntena.initilize()
antenna.move_azel(float(az_cmd),float(el_cmd))
anntena.finalize()
#antenna.tracking_check()
logger.stop()
