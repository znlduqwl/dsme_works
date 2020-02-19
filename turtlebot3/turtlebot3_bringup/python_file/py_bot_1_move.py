#! /usr/bin/env python


import rospy
import tf
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from nav_msgs.msg import *

from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray

from std_msgs.msg import String


center_scan_val ,right_scan_val , left_scan_val = 1 , 1 , 1

forward_dist , side_dist = 0.4 , 0.3

forward_vel,turn_vel=0.1, (np.deg2rad(10))

position_x,position_y,position_z =0,0,0

orientation_x,orientation_y,orientation_z,orientation_w=0,0,0,0

current_pose=0
escape_rad_right=np.deg2rad(30)
escape_rad_left=np.deg2rad(30)

world_x, world_y, world_rot_x, world_rot_y, world_rot_z, world_rot_w=0,0,0,0,0,0

bot_1_x , bot_1_y , bot_rot_x, bot_rot_y, bot_rot_z, bot_rot_w=0,0,0,0,0,0


bot_1_euler=world_euler=[]

space=[]

world_ids__=""
ids=0
a=0
maximum_value=0
count=0

ids_2=False
ids_3=False

case=""
dist=0.25

def update_velocity(linear,angular):
    moving=Twist()
    moving.linear.x  = linear;
    moving.angular.z = angular;

    cmd_vel_pub.publish(moving);


def odom_callback(msg):
    global position_x
    global position_y
    global position_z

    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    position_z = msg.pose.pose.position.z


    orientation_w=msg.pose.pose.orientation.w
    orientation_x=msg.pose.pose.orientation.x
    orientation_y=msg.pose.pose.orientation.y
    orientation_z=msg.pose.pose.orientation.z

    siny=2*(orientation_w*orientation_z+orientation_x*orientation_y)
    cosy=1-2*(((orientation_z)**2)+((orientation_y)**2))

    global current_pose
    current_pose = math.atan2(siny, cosy)

def lader_callback(msg):

    global right_scan_val
    right_scan_val= msg.ranges[330]

    global left_scan_val
    left_scan_val=msg.ranges[30]

    global  center_scan_val
    center_scan_val=msg.ranges[0]


def controller():
    br=tf.TransformBroadcaster()
    checking_pose=0

    move = Twist()
    rate= rospy.Rate(1)
    driving_state = "find_direction"
    for_stop_num = 0
    turn_state="none"
    str_state="none"
    euler_state="none"
    global a

    while not rospy.is_shutdown():
        rospy.Subscriber("status", String, for_ids)

        #print("x : {}".format(position_x)+" y : {}".format(position_y)+" z : {}".format(position_z))
        print("-"*30)
        print("center : " + str(round(center_scan_val*100,4)) + "cm,  right : " + str(str(round(right_scan_val*100,4))) + "cm,  left : " + str(str(round(left_scan_val*100,4)))+"cm")
        #print(driving_state)

        cal_x=abs(world_x-bot_1_x)
        cal_y=abs(world_y-bot_1_y)

        print("x is {}".format(cal_x))
        print(bot_1_euler)
        print(turn_state)
        print(driving_state)
        print(case)
        if case == "one" :

            if driving_state == "find_direction":
                if center_scan_val >= forward_dist:

                    if left_scan_val > 0 and left_scan_val < side_dist:
                        checking_pose = current_pose
                        driving_state = "turn_right"
                        print("step : 1 - turn_right")
                        if (abs(current_pose - checking_pose) >= escape_rad_right):
                            driving_state = "find_direction"

                        else:
                            update_velocity(-0.01, -1 * turn_vel)
                            driving_state = "find_direction"

                    elif right_scan_val > 0 and right_scan_val < side_dist:
                        checking_pose = current_pose
                        driving_state = "turn_left"
                        print("step : 1 - turn_left")
                        if (abs(current_pose - checking_pose) >= escape_rad_left):
                            driving_state = "find_direction"
                        else:

                            update_velocity(-0.01, turn_vel)
                            driving_state = "find_direction"

                    else:
                        # driving_state = "drive_forward"
                        print("step : 1 - straight")
                        update_velocity(forward_vel, 0.0)
                        driving_state = "find_direction"


                elif center_scan_val > 0 and center_scan_val < forward_dist:
                    checking_pose = current_pose
                    if left_scan_val > right_scan_val and left_scan_val != 0 and right_scan_val != 0:
                        driving_state = "turn_left"
                        print("step : 2 - turn_left")
                        update_velocity(-0.01, turn_vel)
                        driving_state = "find_direction"

                    else:
                        driving_state = "turn_right"
                        update_velocity(-0.01, -1 * turn_vel)
                        print("step : 2 - turn_right")
                        driving_state = "find_direction"

                elif center_scan_val == 0.0:
                    for_stop_num = for_stop_num + 1

                    if for_stop_num > 6:
                        update_velocity(forward_vel * (-1), turn_vel)
                        for_stop_num = 0
        elif case == "two" :
            if driving_state == "find_direction" :

                if cal_x>=dist :

                    if bot_1_euler[2] >= 0:
                        euler_state = "plus"

                    elif bot_1_euler[2] < 0:
                        euler_state = "minus"


                    if center_scan_val >= forward_dist :



                        if left_scan_val > 0 and left_scan_val < side_dist :
                            checking_pose = current_pose
                            driving_state= "turn_right"
                            turn_state="turn_right"

                            print("step : 1 - turn_right")
                            if (abs(current_pose - checking_pose) >= escape_rad_right):
                                driving_state = "find_direction"

                            else:
                                update_velocity(0, -1 * turn_vel)
                                driving_state = "find_direction"

                        elif right_scan_val > 0 and right_scan_val < side_dist :
                            checking_pose = current_pose
                            driving_state = "turn_left"
                            turn_state="turn_left"
                            print("step : 2 - turn_left")
                            if (abs(current_pose - checking_pose) >= escape_rad_left):
                                driving_state = "find_direction"
                            else:

                                update_velocity(0, turn_vel)
                                driving_state = "find_direction"

                        else :
                            #driving_state = "drive_forward"
                            print("step : 3 - straight")
                            #str_state="straight"
                            update_velocity(forward_vel, 0.0)
                            driving_state = "find_direction"



                    elif (center_scan_val > 0 and center_scan_val < forward_dist) :
                        checking_pose=current_pose
                        if left_scan_val > right_scan_val and left_scan_val != 0 and right_scan_val != 0 :
                            driving_state = "turn_left"
                            turn_state="turn_left"
                            print("step : 4 - turn_left")
                            update_velocity(0, turn_vel)
                            driving_state = "find_direction"

                        else :
                            driving_state = "turn_right"
                            turn_state = "turn_right"
                            update_velocity(0, -1 * turn_vel)
                            print("step : 5 - turn_right")
                            driving_state = "find_direction"

                    elif center_scan_val == 0.0 :
                        for_stop_num=for_stop_num+1

                        if for_stop_num > 6 :

                            update_velocity(forward_vel*(-1), turn_vel)
                            for_stop_num=0


                elif  cal_x < dist :


                    driving_state="stop"

                    update_velocity(0.0, 0.0)

                    if str_state=="straight" or turn_state=="none":
                        update_velocity(-0.01, math.pi/2)
                        print("straght!!!")
                        update_velocity(forward_vel,0)
                        #driving_state = "find_direction"

                    if turn_state=="turn_left":

                        print("run1")

                        if euler_state=="minus":

                            if bot_1_euler[2] < 0 :
                                print("run2")
                                update_velocity(0, turn_vel*2)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                            elif bot_1_euler[2] >=0:
                                print("run 2-1")
                                update_velocity(forward_vel, 0)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                                #update_velocity(0.3, 0.0)
                                if 0 < center_scan_val and center_scan_val < forward_dist :
                                    update_velocity(forward_vel, math.pi/2)  # -pi - ( my euler ) - pi/4
                                    rospy.sleep(0.3)  # delay sec
                                    driving_state = "find_direction"

                        elif euler_state=="plus":

                            if bot_1_euler[2] > 0 :
                                print("run3")
                                update_velocity(0, turn_vel*2)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec

                            elif bot_1_euler[2] <=0:
                                print("run 3-1")
                                update_velocity(forward_vel, 0)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                                #update_velocity(0.3, 0.0)
                                if 0 < center_scan_val and center_scan_val < forward_dist :
                                    update_velocity(forward_vel, math.pi/2)  # -pi - ( my euler ) - pi/4
                                    rospy.sleep(0.3)  # delay sec
                                    driving_state = "find_direction"


                    elif turn_state=="turn_right":


                        print("run4")


                        if euler_state=="minus":

                            if bot_1_euler[2] < 0 :
                                print("run5")
                                update_velocity(0, -1*turn_vel*2)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                            elif bot_1_euler[2] >=0:
                                print("run 5-1")
                                update_velocity(forward_vel, 0)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                                #update_velocity(0.3, 0.0)
                                if 0 < center_scan_val and center_scan_val < forward_dist :
                                    update_velocity(forward_vel, -1*math.pi/2)  # -pi - ( my euler ) - pi/4
                                    rospy.sleep(0.3)  # delay sec
                                    driving_state = "find_direction"

                        elif euler_state=="plus":

                            if bot_1_euler[2] > 0 :
                                print("run6")
                                update_velocity(0, -1*turn_vel*2)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec

                            elif bot_1_euler[2] <=0:
                                print("run 6-1")
                                update_velocity(forward_vel, 0)  # -pi - ( my euler ) - pi/4
                                print(" <0 left is running")
                                # update_velocity(forward_vel, 0.0)
                                rospy.sleep(0.3)  # delay sec
                                #update_velocity(0.3, 0.0)
                                if 0 < center_scan_val and center_scan_val < forward_dist :
                                    update_velocity(forward_vel, -1*math.pi/2)  # -pi - ( my euler ) - pi/4
                                    rospy.sleep(0.3)  # delay sec
                                    driving_state = "find_direction"

                    driving_state = "find_direction"
        rospy.sleep(0.3)
    if rospy.is_shutdown():
        print("im die")
        update_velocity(-0.4, 0.0)

    cmd_vel_pub.publish(move)
    rate.sleep()

def for_ids(data):
    global case
    case=data.data
    #print("case is {}".format(data))

def bot_1_pose_(data):

    global bot_1_x, bot_1_y, bot_rot_x, bot_rot_y, bot_rot_z, bot_rot_w , bot_1_euler

    bot_1_x=data.position.x
    bot_1_y=data.position.y
    bot_rot_x = data.orientation.x
    bot_rot_y = data.orientation.y
    bot_rot_z = data.orientation.z
    bot_rot_w = data.orientation.w

    bot_1_euler = tf.transformations.euler_from_quaternion((bot_rot_x, bot_rot_y, bot_rot_z, bot_rot_w))


def world_pose_(data):

    global world_x, world_y, world_rot_x, world_rot_y, world_rot_z, world_rot_w , world_euler


    world_x=data.position.x
    world_y=data.position.y
    world_rot_x = data.orientation.x
    world_rot_y = data.orientation.y
    world_rot_z = data.orientation.z
    world_rot_w = data.orientation.w

    world_euler = tf.transformations.euler_from_quaternion((world_rot_x, world_rot_y, world_rot_z, world_rot_w))



if __name__ == '__main__':
    try:
        rospy.init_node('my_bot_1_control')
        cmd_vel_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
        odom_sub = rospy.Subscriber('tb3_0/odom', Odometry, odom_callback)
        scan_sub = rospy.Subscriber('tb3_0/scan', LaserScan, lader_callback)


        world_pose_sub = rospy.Subscriber('world_pose' , Pose , world_pose_)
        bot_1_pose_sub = rospy.Subscriber('bot_1_pose', Pose, bot_1_pose_)

        controller()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:

        stop_=Twist()
        stop_.linear.x = 0.0
        stop_.linear.y = 0.0
        stop_.linear.z = 0.0
        stop_.angular.x = 0.0
        stop_.angular.y = 0.0
        stop_.angular.z = 0.0
        cmd_vel_pub.publish(stop_)
