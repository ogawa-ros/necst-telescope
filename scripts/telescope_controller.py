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

    def move_azel(self,az,el,offset_x=0,offset_y=0,offset_unit="deg"):
        """
        msg
        - type : list
        - unit : az [deg]
               : el [deg]
               : offset_x [deg] or [arcsec] or [arcmin]
               : offset_y [deg] or [arcsec] or [arcmin]
        """

        if offset_unit == "arcsec" :
            az = az/3600
            el = el/3600
        if offset_unit == "arcmin" :
            az = az/60
            el = el/60
        if offset_unit == "deg" :
            pass

        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/azel_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [az,el,offset_x,offset_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_planet(self,planet,offset_x=0,offset_y=0,offset_unit="deg"):
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
        - unit : offset_x [deg]
               : offset_y [deg]
        """
        if planet = "sun"    : planet = 0
        if planet = "moon"   : planet = 1
        if planet = "mercury": planet = 2
        if planet = "venus"  : planet = 3
        if planet = "mars"   : planet = 4
        if planet = "jupiter": planet = 5
        if planet = "saturn" : planet = 6
        if planet = "uranus" : planet = 7
        if planet = "neptune": planet = 8

        if offset_unit == "arcsec" :
            az = az/3600
            el = el/3600
        if offset_unit == "arcmin" :
            az = az/60
            el = el/60
        if offset_unit == "deg" :
            pass

        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/planet_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [planet,offset_x,offset_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_wcs(self,x,y,offset_x=0,offset_y=0,frame="fk5",offset_unit="deg"):
        """
        msg
        - type : list
        - unit : x [deg]
               : y [deg]
               : offset_x [deg]
               : offset_y [deg]
        msg
        - type : string
        - cmd : fk5, galactic, icrs
        """

        if offset_unit == "arcsec" :
            offset_x = offset_x/3600
            offset_y = offset_y/3600
        if offset_unit == "arcmin" :
            offset_x = offset_x/60
            offset_y = offset_y/60
        if offset_unit == "deg" :
            pass

        topic_name = '/necst/telescope/coordinate/stop_cmd'
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
        cmd.data = [x,y,offset_x,offset_y]
        self.make_pub.publish(topic_name, data_class, msg = cmd)
        pass

    def move_raster_wcs(self,x,y,lx,ly,scan_t,l_unit="deg",frame="fk5"):
        """
        msg
        - type : list
        - unit : x [deg]
               : y [deg]
               : lx [deg]
               : ly [deg]
               : scan_t [s]
        msg
        - type : string
        - cmd : fk5, galactic, icrs
        """

        if l_unit == "arcsec" :
            lx = lx/3600
            ly = ly/3600
        if l_unit == "arcmin" :
            lx = lx/60
            ly = ly/60
        if l_unit == "deg" :
            pass

        self.move_wcs(x,y)
        self.tracking_check()

        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/wcs_frame_cmd'
        data_class = std_msgs.msg.String
        cmd = frame
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/wcs_raster_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [x,y,lx,ly,scan_t]
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        pass

    def move_raster_planet(self,planet,lx,ly,scan_t,l_unit="deg"):
        """
        msg
        - type : list
        - unit : planet
               : lx [deg]
               : ly [deg]
               : scan_t [s]
        """

        if l_unit == "arcsec" :
            lx = lx/3600
            ly = ly/3600
        if l_unit == "arcmin" :
            lx = lx/60
            ly = ly/60
        if l_unit == "deg" :
            pass

        if planet = "sun"    : planet = 0
        if planet = "moon"   : planet = 1
        if planet = "mercury": planet = 2
        if planet = "venus"  : planet = 3
        if planet = "mars"   : planet = 4
        if planet = "jupiter": planet = 5
        if planet = "saturn" : planet = 6
        if planet = "uranus" : planet = 7
        if planet = "neptune": planet = 8

        self.move_planet(planet,-lx/2,-ly/2)
        self.tracking_check()

        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/planet_raster_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [planet,lx,ly,scan_t]
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        pass


    def move_raster_azel(self,x,y,lx,ly,scan_t,l_unit="deg"):
        """
        msg
        - type : list
        - unit : x [deg]
               : y [deg]
               : lx [deg]
               : ly [deg]
               : scan_t [s]
        """

        if l_unit == "arcsec" :
            lx = lx/3600
            ly = ly/3600
        if l_unit == "arcmin" :
            lx = lx/60
            ly = ly/60
        if l_unit == "deg" :
            pass

        self.move_azel(x,y)
        self.tracking_check()

        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)

        topic_name = '/necst/telescope/coordinate/azel_raster_cmd'
        data_class = std_msgs.msg.Float64MultiArray
        cmd = std_msgs.msg.Float64MultiArray()
        cmd.data = [x,y,lx,ly,scan_t]
        self.make_pub.publish(topic_name, data_class, msg = cmd)

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
    """

    def finalize(self):
        topic_name = '/necst/telescope/coordinate/stop_cmd'
        data_class = std_msgs.msg.Bool
        cmd = True
        self.make_pub.publish(topic_name, data_class, msg = cmd)


    def get_condition(self):
        pass
