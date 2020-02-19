#!/usr/bin/env python

import rospy
import roslib
import tf
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from geometry_msgs.msg import *
from nav_msgs.msg import *

import math
import numpy as np
from sensor_msgs.msg import LaserScan

id=''
pose_x,pose_y,pose_z=0,0,0
rot_x,rot_y,rot_z,rot_w=0,0,0,0

center_scan_val ,right_scan_val , left_scan_val = 1 , 1 , 1

forward_dist , side_dist = 0.4 , 0.3

forward_vel,turn_vel=0.3,0.3

position_x,position_y,position_z =0,0,0

orientation_x,orientation_y,orientation_z,orientation_w=0,0,0,0

current_pose=0
escape_rad_right=np.deg2rad(30)
escape_rad_left=np.deg2rad(30)



def fiducial_tf(data,turtlebot_id):
    br=tf.TransformBroadcaster()
    pose_ = Pose()
    #odom_pub=rospy.Publisher("world_odom",Odometry,queue_size=10)
    #map_pub = rospy.Publisher('1/map', OccupancyGrid)
    #odom=Odometry()
    #current_time=rospy.Time.now()
    global id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w


    for data_ in data.transforms :
        id=str(data_.fiducial_id)
        pose_x=data_.transform.translation.x
        pose_y=data_.transform.translation.y
        pose_z=data_.transform.translation.z

        rot_x=data_.transform.rotation.x
        rot_y = data_.transform.rotation.y
        rot_z = data_.transform.rotation.z
        rot_w = data_.transform.rotation.w

        """
        odom.header.stamp=current_time
        odom.header.frame_id="odom"
        odom.pose.pose.position=Point(pose_x,pose_y,0.)
        odom.pose.pose.orientation=Quaternion(rot_x,rot_y,rot_z,rot_w)
        """
    #odom_pub.publish(odom)

    if turtlebot_id==id:
        pose_.position.x=pose_x
        pose_.position.y = pose_y
        pose_.position.z = pose_z
        pose_.orientation.x = rot_x
        pose_.orientation.y = rot_y
        pose_.orientation.z = rot_z
        pose_.orientation.w = rot_w
        pub.publish(pose_)
        #print("run")
        br.sendTransform((pose_x,pose_y,0),(rot_x,rot_y,rot_z,rot_w),rospy.Time.now(),turtlebot_id,"camera_link")
        #euler add quaternion !!!!!

if __name__ == '__main__':
    try:
        rospy.init_node("bot_1")

        turtlebot_id = '3'

        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, fiducial_tf, turtlebot_id)
        pub = rospy.Publisher('bot_2_pose', Pose, queue_size=10)

        # node()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass