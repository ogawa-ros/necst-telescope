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

        rospy.Subscriber("/1p85m2019/az", Float64, self.recieve_az)
        rospy.Subscriber("/1p85m2019/el", Float64, self.recieve_el)

        pub_az_lock = rospy.Publisher("/pyinterface/pci7415/rsw0/x/output_do")
        pub_el_lock = rospy.Publisher("/pyinterface/pci7415/rsw0/y/output_do",std_msgs.msg.Int64MultiArray, queue_size=1)

    def recieve_az(self, q):
        az = q.data
        if az > self.az_upper_limit or az < self.az_lower_limit:
            array = Int64MultiArray()
            array.data = [0,0,0,0]
            pub_az_lock.publish(array)
        else:
            pass
        return

    def recieve_el(self, q):
        el = q.data
        if el > self.el_upper_limit or el < self.el_lower_limit:
            array = Int64MultiArray()
            array.data = [0,0,0,0]
            pub_el_lock.publish(array)
        else:
            pass
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    lock = motor_locker()
    rospy.spin()
