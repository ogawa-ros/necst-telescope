#!/usr/bin/env python3

import time
import threading
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

node_name = 'tracking'

class tracking_check(object):
    tracking = False
    track_falseflag = False
    az = 0
    el = 0
    az_cmd = 0
    el_cmd = 0

    def __init__(self):
        self.trac_threshold = rospy.get_param("~trac_threshold")
        rospy.Subscriber("/necst/telescope/coordinate/apparent_az_cmd", Float64, self.recieve_az_cmd)
        rospy.Subscriber("/necst/telescope/coordinate/apparent_el_cmd", Float64, self.recieve_el_cmd)
        rospy.Subscriber("/1p85m/az", Float64, self.recieve_az)
        rospy.Subscriber("/1p85m/el", Float64, self.recieve_el)
        rospy.Subscriber("/necst/telescope/coordinate/planet_cmd", String, self.recieve_coord_cmd)
        rospy.Subscriber("/necst/telescope/coordinate/wcs_cmd"   , Float64MultiArray, self.recieve_coord_cmd)


    def recieve_az_cmd(self, q):
        self.az_cmd = q.data
        return

    def recieve_el_cmd(self, q):
        self.el_cmd = q.data
        return

    def recieve_az(self, q):
        self.az = q.data
        return

    def recieve_el(self, q):
        self.el = q.data
        return

    def recieve_coord_cmd(self, q):
        self.track_falseflag = True

    def check_track(self):
        track_count = 0
        time.sleep(1)
        while not rospy.is_shutdown():
            if self.track_falseflag:
                self.tracking = False
                time.sleep(3) # waiting until antenna moving
                self.track_falseflag = False
            command_az = self.az_cmd
            command_el = self.el_cmd

            """ start checking track """
            enc_az = self.az
            enc_el = self.el

            d_az = abs(command_az - enc_az) #deg
            d_el = abs(command_el - enc_el) #deg

            threshold = self.trac_threshold/3600 #arcsec->deg
            if d_az <= threshold and d_el <=threshold: #deg
                track_count += 1
            else:
                track_count = 0
            if track_count >= 5:# if tracking is True for 0.5[sec]
                self.tracking = True
            else:
                self.tracking = False
            #print(self.tracking)
            time.sleep(0.1)
        return self.tracking

    def pub_tracking(self):
        pub = rospy.Publisher('/necst/telescope/tracking_check', Bool, queue_size = 10, latch = True)
        track = Bool()
        while not rospy.is_shutdown():
            track.data = self.tracking
            pub.publish(track)
            time.sleep(0.01)

    def start_thread(self):
        th = threading.Thread(target = self.pub_tracking)
        th.setDaemon(True)
        th.start()
        check = threading.Thread(target = self.check_track)
        check.setDaemon(True)
        check.start()


if __name__ == '__main__':
    rospy.init_node(node_name)
    track = tracking_check()
    track.start_thread()
    rospy.spin()
