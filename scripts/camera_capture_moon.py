#!/usr/bin/env python3

import sys
import time
import datetime
sys.path.append("/home/exito/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/necst-core/scripts")
import core_controller
import rospy

name = "camera_capture_moon"

savepath = "/home/m100raspi/data/moon/"

rospy.init_node("name")

logger = core_controller.logger()
camera = controller.camera()
antenna = controller.antenna()

###logger#
date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

###picture savename###
nowtimestamp = datetime.datetime.today()
timestr = nowtimestamp.strftime('%Y%m%d_%H.%M.%S')
savename = timestr +".JPG"
savep = savepath + savename

logger.start(file_name)
antenna.move_planet("moon")
antenna.tracking_check()
camera.capture(savep)
logger.stop()
