#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import serial
import time

# g_data = 0
# g_data_list =0
g_linear_x = 0
g_angular_z = 0

def set_vel(vel_msg, lv, av):
    vel_msg.linear.x  = lv
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = av

def read_from_serial_port():
    global g_data
    global g_linear_x
    global g_angular_z
    global g_data_list

    # Specify the correct port and baudrate
    ser = serial.Serial('/dev/pts/7', 9600)

    g_data = ser.readline().decode('utf-8').strip()
    g_data_list = g_data.split(',')
    
    g_linear_x = float(g_data_list[1])
    g_angular_z = float(g_data_list[0])

    time.sleep(0.01)

def main():
    rospy.init_node('momo_serial')
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    set_vel(vel_msg, 0, 0)
    
    while not rospy.is_shutdown():
        read_from_serial_port()

        vel_msg.linear.x  = g_linear_x
        vel_msg.angular.z = g_angular_z

        vel_publisher.publish(vel_msg)
        rospy.loginfo( g_data_list )


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException: pass
