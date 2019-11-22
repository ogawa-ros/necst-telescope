#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray

node_name = 'motor_locker'

class motor_locker(object):

    def __init__(self):
        self.az_upper_limit = rospy.get_param("~az_upper_limit")
        self.az_lower_limit = rospy.get_param("~az_lower_limit")
        self.el_upper_limit = rospy.get_param("~el_upper_limit")
        self.el_lower_limit = rospy.get_param("~el_lower_limit")

        self.pub_az_lock = rospy.Publisher("/1p85m/az_lock_cmd",Bool, queue_size=1)
        self.pub_el_lock = rospy.Publisher("/1p85m/el_lock_cmd",Bool, queue_size=1)

        rospy.Subscriber("/1p85m/az", Float64, self.recieve_az)
        rospy.Subscriber("/1p85m/el", Float64, self.recieve_el)



    def recieve_az(self, q):
        az = q.data
        if az > self.az_upper_limit or az < self.az_lower_limit:
            self.pub_az_lock.publish(True)
        else:
            pass
        return

    def recieve_el(self, q):
        el = q.data
        if el > self.el_upper_limit or el < self.el_lower_limit:
            self.pub_el_lock.publish(True)
        else:
            pass
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    lock = motor_locker()
    rospy.spin()
