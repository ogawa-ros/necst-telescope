#!/usr/bin/env python3

import sys
import time
import rospy
import std_msgs.msg

class controller(object):
    def __init__(self):
        self.antenna = antenna()

class make_pub(object):
    def __init__(self):
        self.pub = {}
        pass

    def publish(self, topic_name, data_class, msg):
        if topic_name not in self.pub:
            self.set_publisher(topic_name = topic_name, data_class = data_class)
            pass
        self.pub[topic_name].publish(msg)
        return

    def set_publisher(self, topic_name, data_class):
        self.pub[topic_name] = rospy.Publisher(name = topic_name, data_class = data_class, queue_size = 1, latch = False)
        time.sleep(0.1)
        return

class make_sub(object):
    def __init__(self):
        self.sub = {}
        pass

    def subs(self, topic_name, data_class, msg):
        if topic_name not in self.pub:
            self.set_publisher(topic_name = topic_name, data_class = data_class)
            pass
        self.pub[topic_name].publish(msg)
        return

    def set_publisher(self, topic_name, data_class):
        self.pub[topic_name] = rospy.Publisher(name = topic_name, data_class = data_class, queue_size = 1, latch = False)
        time.sleep(0.1)
        return


class antenna(object):
    def __init__(self):
        self.make_pub = make_pub()

    def move_azel(self,az,el):
        """
        msg
        - type : list
        - unit : az [deg]
               : el [deg]
        """
        topic_name = '/necst_telescope/coordinate/refracted_azel_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [az,el]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_planet(self,planet):
        """
        msg
        - type : String
        - cmd : 'earth','sun','moon','mercury','venus','earth-moon-barycenter','mars','jupiter','saturn','uranus','neptune'
        """
        topic_name = '/necst_telescope/coordinate/planet_cmd'
        data_class = std_msgs.msg.String
        cmd = planet
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_wcs(self,ra,dec):
        """
        msg
        - type : list
        - unit : ra  [deg]
               : dec [deg]
        - frame : fk5
        """
        topic_name = '/necst_telescope/coordinate/wcs_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [ra,dec]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def antenna_tracking_check(self):
        print(" Moving now....")
        time.sleep(2.)
        while not self.antenna_tracking_flag:# or (int(self.command_time) != self.antenna_tracking_time):
            time.sleep(0.01)
            pass
        return

class camera(object):
    def __init__(self):
        self.make_pub = make_pub()

    def capture(self,savepath):
        """
        msg
        - type : String

        """
        topic_name = '/dev/m100/capture/savepath'
        data_class = std_msgs.msg.String
        cmd.data = savepath
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass
