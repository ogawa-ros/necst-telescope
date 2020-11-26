#!/usr/bin/env python3
"""
For test of telescope driving
"""
import sys
import time
import numpy
import math
import os
import datetime
sys.path.append("/home/exito/ros/src/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller
import rospy

name = "tracking_moon_test"

rospy.init_node(name)

logger = core_controller.logger()
antenna = telescope_controller.antenna()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

#logger.start(file_name)

time.sleep(3)
print("Moving to moon")
antenna.move_planet("mars")
antenna.tracking_check()
time.sleep(100)
antenna.finalize()
#logger.stop()
