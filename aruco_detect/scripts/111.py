#!/usr/bin/env python

import rospy
import roslib
import tf
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import String

id=''
pose_x,pose_y,pose_z=0,0,0
rot_x,rot_y,rot_z,rot_w=0,0,0,0

def callback(data,turtlebot_id):
    br=tf.TransformBroadcaster()
    pose_ = Pose()
    std = String()
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

        euler=tf.transformations.euler_from_quaternion((rot_x,rot_y,rot_z,rot_w))
        std_pub.publish(str(id))
        id = "0"

        print(euler)


        pub.publish(pose_)
        std_pub.publish(str(id))
        id = "0"
        #print("run")
        br.sendTransform((pose_x,pose_y,0),(rot_x,rot_y,rot_z,rot_w),rospy.Time.now(),turtlebot_id,"camera_link")

    elif id=="0":
        std_pub.publish(str(id))
        #euler add quaternion !!!!!
        #print(pose_x)
        #print(pose_y)
        #print(pose_z)
        #pub.publish(pose_)
    #br.sendTransform(())
#def node():
   # while not rospy.is_shutdown():



if __name__ == '__main__':
    rospy.init_node("ex_sub")
    turtlebot_id = '1'


    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, callback, turtlebot_id)
    pub = rospy.Publisher('world_pose', Pose, queue_size=10)
    std_pub=rospy.Publisher('world_ids',String,queue_size=10)
    #rospy.Subscriber("tb3_0/map", OccupancyGrid, cacaca)
    # node()

    rospy.spin()

