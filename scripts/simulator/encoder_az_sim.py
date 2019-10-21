#!/usr/bin/env python3

name = "encoder_az_sim"

import time
import threading
import rospy
import std_msgs.msg


class encoder_az_sim(object):

    command_speed = 0.0
    enc_az = 0.0

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/encorder_az",
                data_class = std_msgs.msg.Float64,
                latch = True,
                queue_size = 1,
            )

        self.topic_from = rospy.Subscriber(
                name = "/az_speed",
                data_class = std_msgs.msg.Float64,
                callback = self.encoder_az_sim,
                queue_size = 1,
            )

    def encoder_az_sim(self, status):
        self.command_speed = status.data
        return

    def publish_status(self):

        enc_az_last = None
        while not rospy.is_shutdown():
            self.enc_az += self.command_speed * 0.1 * 0.1

            if self.enc_az != enc_az_last:
                self.topic_to.publish(int(self.enc_az / 3600))

                enc_az_last = self.enc_az

            time.sleep(0.01)
            continue
              
    def start_thread(self):
        th = threading.Thread(target=self.publish_status)
        th.setDaemon(True)
        th.start()

if __name__ == "__main__":
    rospy.init_node(name)
    enc_az_sim = encoder_az_sim()
    enc_az_sim.start_thread()
    rospy.spin()