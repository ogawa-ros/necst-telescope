#!/usr/bin/env python3

name = "antenna_az_commander_pid"

import math
import time
import rospy
import std_msgs.msg


class antenna_az_feedback(object):

    speed_d = 0.0
    pre_deg = 0.0
    pre_hensa = 0.0
    enc_before = 0.0
    ihensa = 0.0
    i_ave_num = 10
    t_now = t_past = 0.0
    current_speed = 0

    deg_enc = 0.0

    lock = False

    def __init__(self):

        self.p_coeff = rospy.get_param("~p_coeff")
        self.i_coeff = rospy.get_param("~i_coeff")
        self.d_coeff = rospy.get_param("~d_coeff")

        self.hensa_stock = [0]*self.i_ave_num

        self.gear_ratio = rospy.get_param("~gear_ratio")
        self.pulseper360deg = rospy.get_param("~pulseper360deg")
        self.pulse_a = rospy.get_param("~pulse_a")
        self.pulse_b = rospy.get_param("~pulse_b")

        self.MOTOR_MAXSTEP = rospy.get_param("~MOTOR_MAXSTEP")
        self.MOTOR_AZ_MAXSPEED = rospy.get_param("~MOTOR_AZ_MAXSPEED")

        self.topic_to = rospy.Publisher(
                name = "/1p85m/az_speed",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        self.topic_cur = rospy.Publisher(
                name = "/1p85m/az_current_speed",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )


        self.topic_tar = rospy.Publisher(
                name = "/1p85m/az_target_speed",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        self.topic_hensa = rospy.Publisher(
                name = "/1p85m/az_pid_hensa",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        topic_from1 = rospy.Subscriber(
                name = "/1p85m/az_cmd2",
                data_class = std_msgs.msg.Float64,
                callback = self.antenna_az_feedback,
                queue_size = 1,
            )

        topic_from2 = rospy.Subscriber(
                name = "/1p85m/az",
                data_class = std_msgs.msg.Float64,
                callback = self.antenna_az_encoder,
                queue_size = 1,
            )

        pass

    def antenna_az_feedback(self, command):
        MOTOR_MAXSTEP = self.MOTOR_MAXSTEP
        MOTOR_AZ_MAXSPEED = self.MOTOR_AZ_MAXSPEED
        # deg/sec

        deg_cmd = command.data

        if self.t_past == 0.0:
            self.t_past = time.time()
        else:
            pass
        self.t_now = time.time()

        ret = self.calc_pid(deg_cmd, self.deg_enc,
                self.pre_deg, self.pre_hensa, self.ihensa, self.enc_before,
                self.t_now, self.t_past,
                self.p_coeff, self.i_coeff, self.d_coeff)
        speed = ret[0]

        #update
        self.pre_hensa = deg_cmd - self.deg_enc
        self.pre_deg = deg_cmd
        self.enc_before = self.deg_enc
        self.ihensa = ret[1]
        self.t_past = self.t_now

        #deg->palth
        speed = speed*self.gear_ratio/360*(self.pulseper360deg*(self.pulse_b/self.pulse_a))

        #limit of acceleraion
        if abs(speed - self.speed_d) < MOTOR_MAXSTEP:
            self.speed_d = speed
        else:
            if (speed - self.speed_d) < 0:
                a = -1
            else:
                a = 1
            self.speed_d += a*MOTOR_MAXSTEP

        #limit of max speed
        if self.speed_d > MOTOR_AZ_MAXSPEED:
            self.speed_d = MOTOR_AZ_MAXSPEED
        if self.speed_d < -MOTOR_AZ_MAXSPEED:
            self.speed_d = -MOTOR_AZ_MAXSPEED

        command_speed = self.speed_d

        if self.lock == True:
            self.speed_d = 0.0
            self.topic_to.publish(0.0)
            return
        else:
            self.topic_to.publish(command_speed)
        return

    def antenna_az_encoder(self, status):
        self.deg_enc = status.data
        return

    def antenna_az_pid(self, status):
        self.p_coeff = status.data[0]
        self.i_coeff = status.data[1]
        self.d_coeff = status.data[2]
        return

    def calc_pid(self,target_deg, encoder_deg, pre_deg, pre_hensa, ihensa, enc_before, t_now, t_past, p_coeff, i_coeff, d_coeff):
        """
        DESCRIPTION
        ===========
        This function determine az&el speed for antenna
        """

        #calculate ichi_hensa
        hensa = target_deg - encoder_deg

        self.hensa_stock.append(hensa)
        self.hensa_stock = self.hensa_stock[1:]

        dhensa = hensa - pre_hensa
        if math.fabs(dhensa) > 1:
            dhensa = 0

        if (encoder_deg - enc_before) != 0.0:
            self.current_speed = (encoder_deg - enc_before) / (t_now-t_past)

        if pre_deg == 0: # for first move
            target_speed = 0
        else:
            target_speed = (target_deg - pre_deg)/(t_now - t_past)

        #ihensa += (hensa + pre_hensa)/2
        #if math.fabs(hensa) > 50: #50??
        #    ihensa = 0.0
        try:
            ihensa = sum(self.hensa_stock)/len(self.hensa_stock)
        except:
            ihensa = 0

        #PID
        rate = target_speed + p_coeff*hensa + i_coeff*ihensa*(t_now-t_past) + d_coeff*dhensa/(t_now-t_past)

        #print(current_speed)
        #print(target_deg,pre_deg,t_now,t_past)
        #print(rate,target_speed,hensa)
        self.topic_tar.publish(target_speed)
        self.topic_cur.publish(self.current_speed)
        self.topic_hensa.publish(hensa)
        return [rate, ihensa]

if __name__ == "__main__":
    rospy.init_node(name)
    feedback = antenna_az_feedback()
    rospy.spin()
