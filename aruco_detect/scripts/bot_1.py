#!/usr/bin/env python

import rospy
import roslib
import tf
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import String



ids=0
pose_x,pose_y,pose_z=0,0,0
rot_x,rot_y,rot_z,rot_w=0,0,0,0

def callback(data,turtlebot_id):
    global pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w
    global ids
    for data_ in data.transforms :
        ids=data_.fiducial_id
        pose_x=data_.transform.translation.x
        pose_y=data_.transform.translation.y
        pose_z=data_.transform.translation.z

        rot_x=data_.transform.rotation.x
        rot_y = data_.transform.rotation.y
        rot_z = data_.transform.rotation.z
        rot_w = data_.transform.rotation.w

def main () :


    pose_ = Pose()

    global ids

    while not rospy.is_shutdown():



        if int(turtlebot_id) == ids:

            pose_.position.x = pose_x
            pose_.position.y = pose_y
            pose_.position.z = pose_z
            pose_.orientation.x = rot_x
            pose_.orientation.y = rot_y
            pose_.orientation.z = rot_z
            pose_.orientation.w = rot_w


            pub.publish(pose_)

            # print("run")
            br.sendTransform((pose_x, pose_y, 0), (rot_x, rot_y, rot_z, rot_w), rospy.Time.now(), turtlebot_id,
                             "camera_link")



if __name__ == '__main__':

    try:
        rospy.init_node("bot_1")

        listener = tf.TransformListener()
        br = tf.TransformBroadcaster()

        turtlebot_id = '2'

        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, callback, turtlebot_id)
        pub = rospy.Publisher('bot_1_pose', Pose, queue_size=10)

        main()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass