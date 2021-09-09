#!/usr/bin/env python
#-*- coding:utf-8 -*-

#python2??
import rospy
from rospy.names import scoped_name
from std_msgs.msg import  Int16
from std_msgs.msg import Float32
import time

sub_speed=0
cruise_speed=30
sub_Auto=0
sub_ASM=0

rospy.init_node('pid_vel') #노드 생성

def callback(msg):
    global sub_speed 
    sub_speed = msg.data[19]

    global sub_Auto 
    sub_Auto = msg.data[5]

    global sub_ASM 
    sub_ASM = msg.data[7]

    print( sub_speed, sub_Auto, sub_ASM)

#subscriber 선언

sub = rospy.Subscriber('/ioniq_info', Float32, callback, queue_size=1)


#Publisher 선언
pub_accel = rospy.Publisher('/dbw_cmd/Accel', Int16, queue_size=1)
pub_brake = rospy.Publisher('/dbw_cmd/Brake', Int16, queue_size=1)

def main(args=None):
    global sub_ASM
    global sub_Auto
    global sub_speed
    global cruise_speed

    kp = 1.25
    ki = 0.85
    kd = 0
    prev_error = 0
    error_i = 0

    accel=0
    brake=0

    while(sub_Auto!=0):
        prev_time=time.time()
        cur_time=time.time()
        del_time=cur_time - prev_time

        error_p = cruise_speed - sub_speed
        error_i = error_i + error_p * del_time
        error_d = (error_p - prev_error)/del_time

        pid_out = kp*error_p + ki*error_i + kd*error_d

        prev_error = error_p
        prev_time = cur_time

        # accel_max - accel_dead_zone = 3000 - 800 = 2200
		# 2200/10 = 220, 220 + 1 = 221
        if pid_out > 0:
                for i in range(221):
                    if i <= pid_out < i+1:
                        accel.data = 800 + 10*i
                        brake.data = 0
			
		# brake_max - brake_dead_zone = 27000 - 3500 = 23500
		# 23500/10 = 2350, 2350 + 1 = 2351
        elif pid_out < 0:
                for i in range(2351):
                    if i <= abs(pid_out) < i+1:
                        accel.data = 0
                        brake.data = 3500+10*i



        pub_accel.publish(accel)
        pub_brake.publish(brake)

        print("cruise_mode : ", sub_Auto , "  , cruise_speed : ", cruise_speed, "brake : ", brake.data, "accel : ", accel.data)







if __name__ == "__main__":
    main()
