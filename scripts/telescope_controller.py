#!/usr/bin/env python3

import sys
import time
import rospy
import std_msgs.msg
sys.path.append("../../necst-core/scripts")
import topic_utils

class controller(object):
    def __init__(self):
        self.antenna = antenna()
        self.camera = camera()

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


class antenna(object):
    def __init__(self):
        self.make_pub = make_pub()
        self.track  = topic_utils.receiver('/necst/telescope/tracking_check'            ,std_msgs.msg.Bool)
        self.az     = topic_utils.receiver('/necst/telescope/az'                        ,std_msgs.msg.Float64)
        self.az_cmd = topic_utils.receiver('/necst/telescope/coordinate/apprent_az_cmd' ,std_msgs.msg.Float64)
        self.el     = topic_utils.receiver('/necst/telescope/el'                        ,std_msgs.msg.Float64)
        self.el_cmd = topic_utils.receiver('/necst/telescope/coordinate/apprent_az_cmd' ,std_msgs.msg.Float64)

    def move_azel(self,az,el):
        """
        msg
        - type : list
        - unit : az [deg]
               : el [deg]
        """
        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        time.sleep(0.1)
        topic_name = '/necst/telescope/coordinate/refracted_azel_cmd'
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
        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        time.sleep(0.1)
        topic_name = '/necst/telescope/coordinate/planet_cmd'
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
        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        time.sleep(0.1)
        topic_name = '/necst/telescope/coordinate/wcs_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [ra,dec]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def tracking_check(self):
        tracking_flag = False
        print(" Moving now....")
        time.sleep(2.)
        while not tracking_flag:
            tracking_flag = self.track.recv()
            time.sleep(0.01)
            pass
        return

    def get_az_cmd(self):
        az_cmd = self.az_cmd.recv()
        return az_cmd

    def get_el_cmd(self):
        el_cmd = self.el_cmd.recv()
        return el_cmd

    def get_az(self):
        az = self.az.recv()
        return az

    def get_el(self):
        el = self.el.recv()
        return el
    """
    def initilize(self):
        data_class = std_msgs.msg.Bool
        data = True
        topic_name = '/1p85m/el_lock_cmd'
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        topic_name = '/1p85m/az_lock_cmd'
        self.make_pub.publish(topic_name, data_class, msg = cmd)

    def finalize(self):
        data_class = std_msgs.msg.Bool
        data = False
        topic_name = '/1p85m/el_lock_cmd'
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        data_class = std_msgs.msg.Bool
        data = False
        topic_name = '/1p85m/el_lock_cmd'
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        topic_name = '/1p85m/az_lock_cmd'
        self.make_pub.publish(topic_name, data_class, msg = cmd)
    """

    def get_condition(self):
        pass


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
        cmd = savepath
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass
