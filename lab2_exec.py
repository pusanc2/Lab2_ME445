#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1 (3 is top and 1 is bottom for the Q coordinates)
Q11 = [149.78*pi/180.0, -55.13*pi/180.0, 120.66*pi/180.0, -156.03*pi/180.0, -90.5*pi/180.0, 59.95*pi/180.0]
Q12 = [149.80*pi/180.0, -63.12*pi/180.0, 119.73*pi/180.0, -147.11*pi/180.0, -90.44*pi/180.0, 59.87*pi/180.0]
Q13 = [149.83*pi/180.0, -69.92*pi/180.0, 117.53*pi/180.0, -138.11*pi/180.0, -90.39*pi/180.0, 59.80*pi/180.0]
Q21 = [172.84*pi/180.0, -54.70*pi/180.0, 118.16*pi/180.0, -153.63*pi/180.0, -91.85*pi/180.0, 82.54*pi/180.0]
Q22 = [172.86*pi/180.0, -61.82*pi/180.0, 117.27*pi/180.0, -145.62*pi/180.0, -91.80*pi/180.0, 82.48*pi/180.0]
Q23 = [172.42*pi/180.0, -68.83*pi/180.0, 115.13*pi/180.0, -136.33*pi/180.0, -90.51*pi/180.0, 82.37*pi/180.0]
Q31 = [194.53*pi/180.0, -48.50*pi/180.0, 101.58*pi/180.0, -141.51*pi/180.0, -91.63*pi/180.0, 104.25*pi/180.0]
Q32 = [194.55*pi/180.0, -54.67*pi/180.0, 100.51*pi/180.0, -134.29*pi/180.0, -91.60*pi/180.0, 104.18*pi/180.0]
Q33 = [194.57*pi/180.0, -59.68*pi/180.0, 98.40*pi/180.0, -127.13*pi/180.0, -91.57*pi/180.0, 104.10*pi/180.0]

safe_point_1 = [149.86*pi/180.0, -75.73*pi/180.0, 114.08*pi/180.0, -128.86*pi/180.0, -90.35*pi/180.0, 59.71*pi/180.0]
safe_point_2 = [172.44*pi/180.0, -73.72*pi/180.0, 112.21*pi/180.0, -128.51*pi/180.0, -90.5*pi/180.0, 82.3*pi/180.0]
safe_point_3 = [194.59*pi/180.0, -63.42*pi/180.0, 95.74*pi/180.0, -120.78*pi/180.0, -91.55*pi/180.0, 104.03*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
# Define the angles for Q21,22,23 and Q31,32,33
Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
gripper_status = 0
def gripper_callback(msg):
    global gripper_status
    gripper_status = msg.analog

############### Your Code End Here ###############

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0


    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    #Setting the starting location
    input_done = 0
    starting_location = 0

    while(not input_done):
        input_string = input("Enter the starting location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            starting_location = 0
        elif (int(input_string) == 2):
            input_done = 1
            starting_location = 1
        elif (int(input_string) == 3):
            input_done = 1
            starting_location = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
    
    #Setting the ending location 
    input_done_2 = 0
    ending_location = 0
    
    while(not input_done_2):
        input_string = input("Enter the ending location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            ending_location = 0
        elif (int(input_string) == 2):
            input_done = 1
            ending_location = 1
        elif (int(input_string) == 3):
            input_done = 1
            ending_location = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
    intermediate_location = 3 - starting_location - ending_location    
    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    while(loop_count > 0):

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        rospy.loginfo("Sending goal 1 ...")
        move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

        gripper(pub_command, loop_rate, suction_on)
        # Delay to make sure suction cup has grasped the block
        time.sleep(1.0)

        rospy.loginfo("Sending goal 2 ...")
        move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

        rospy.loginfo("Sending goal 3 ...")
        move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

        loop_count = loop_count - 1

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
