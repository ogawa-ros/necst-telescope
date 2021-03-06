#!/usr/bin/env python3

import sys
import time
import datetime
sys.path.append("/home/exito/ros/src/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller
import rospy

name = "camera_capture_moon"

savepath = "/home/m100raspi/data/moon/"

rospy.init_node("name")

logger = core_controller.logger()
camera = telescope_controller.camera()
antenna = telescope_controller.antenna()

###logger#
date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

###picture savename###
az = antenna.get_az()
el = antenna.get_el()
nowtimestamp = datetime.datetime.today()
timestr = nowtimestamp.strftime('%Y%m%d_%H.%M.%S')
savename = timestr +"_az_"+str(az) +"_el_"+str(el)+".JPG"
savep = savepath + savename

logger.start(file_name)
antenna.move_planet("moon")
antenna.tracking_check()
camera.capture(savep)
logger.stop()
