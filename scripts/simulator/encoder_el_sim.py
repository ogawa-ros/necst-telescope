#!/usr/bin/env python3

name = "encoder_el_sim"

import time
import threading
import rospy
import std_msgs.msg


class encoder_el_sim(object):

    command_speed = 0.0
    enc_el = 0.0

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/1p85m2019/el",
                data_class = std_msgs.msg.Float64,
                latch = True,
                queue_size = 1,
            )

        self.topic_from = rospy.Subscriber(
                name = "/1p85m2019/el_speed",
                data_class = std_msgs.msg.Float64,
                callback = self.encoder_el_sim,
                queue_size = 1,
            )

    def encoder_el_sim(self, status):
        self.command_speed = status.data
        return

    def publish_status(self):

        enc_el_last = None
        while not rospy.is_shutdown():
            self.enc_el += self.command_speed * 0.01 * 0.617

            if self.enc_el != enc_el_last:
                self.topic_to.publish(int(self.enc_el / 3600))

                enc_el_last = self.enc_el

            time.sleep(0.01)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.publish_status)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    enc_el_sim = encoder_el_sim()
    enc_el_sim.start_thread()
    rospy.spin()
