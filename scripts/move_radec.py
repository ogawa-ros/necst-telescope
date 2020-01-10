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

name = "move_radec"

rospy.init_node(name)

antenna = telescope_controller.antenna()

ra_cmd = 15*(5+35/60+17.3/3600)
dec_cmd = -5-23/60-28/3600

print("Moving ra,dec "+str(ra_cmd)+", "+str(dec_cmd))
#anntena.initilize()
antenna.move_wcs(ra_cmd,dec_cmd)
antenna.tracking_check()
print("track ")
#anntena.finalize()
