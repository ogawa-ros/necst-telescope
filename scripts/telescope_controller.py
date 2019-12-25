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

    def move_azel(self,az,el,off_x=0,off_y=0,off_unit="deg"):
        """
        msg
        - type : list
        - unit : az [deg]
               : el [deg]
               : off_x [deg] or [arcsec] or [arcmin]
               : off_y [deg] or [arcsec] or [arcmin]
        """

        if off_unit == "arcsec" :
            az = az*3600
            el = el*3600
        if off_unit == "arcmin" :
            az = az*60
            el = el*60
        if off_unit == "deg" :
            pass

        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/azel_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [az,el,off_x,off_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_planet(self,planet,off_x=0,off_y=0,off_unit="deg"):
        """
        msg
        - type : list
        - cmd : 0 : sun
                1 : moon
                2 : mercury
                3 : venus
                4 : mars
                5 : jupiter
                6 : saturn
                7 : uranus
                8 : neptune
        - unit : off_x [deg]
               : off_y [deg]
        """

        if off_unit == "arcsec" :
            az = az*3600
            el = el*3600
        if off_unit == "arcmin" :
            az = az*60
            el = el*60
        if off_unit == "deg" :
            pass

        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/planet_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [planet,off_x,off_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_wcs(self,x,y,off_x=0,off_y=0,frame="fk5"):
        """
        msg
        - type : list
        - unit : x [deg]
               : y [deg]
               : off_x [deg]
               : off_y [deg]

        - type : string
        - cmd : fk5, galactic, icrs
        """

        topic_name = '/necst/telescope/coordinate/stop_refracted_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/wcs_frame_cmd'
        data_class = std_msgs.msg.String
        cmd = frame
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/wcs_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [x,y,off_x,off_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_raster_wcs(self,x,y,lx,ly,start_t,scan_t,frame="fk5"):
        dx = x/scan_t*0.1
        num = int(lx/dx)

        topic_name = '/necst/telescope/coordinate/wcs_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()

        for i in range(num):
            off_x = dx*i
            off_y = dy*i
            cmd.data = [x,y,off_x,off_y]
            self.make_pub.publish(topic_name, data_class, msg = cmd)
            continue

        pass



    def tracking_check(self):
        tracking_flag = False
        print(" Moving now....")
        time.sleep(1)
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
