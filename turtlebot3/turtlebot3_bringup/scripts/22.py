#!/usr/bin/python

import math
import numpy as np
import roslib; roslib.load_manifest('tf2_geometry_msgs')
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped


def test_similar(test_name, v, x, y, z):
    tolerance = 1e-6
    if np.abs(np.array([v.x, v.y, v.z]) - np.array([x, y, z])).max() > tolerance:
        print('FAIL:', test_name)
        print ('expected:', x, y, z)
        print ('actual  :', v.x, v.y, v.z)


# a translation should not modify a Vector3 (only rotation should)
# testing with a transform that is a pure translation
t = TransformStamped()
t.transform.translation.z = 1
t.transform.rotation.w = 1

v = Vector3Stamped()
v.vector.x = 1
v.vector.y = 0
v.vector.z = 0

vt = tf2_geometry_msgs.do_transform_vector3(v, t)
test_similar('translation should not be applied', vt.vector, v.vector.x, v.vector.y, v.vector.z)


# making sure the rotation is applied properly
t = TransformStamped()
t.transform.translation.x = 0
q = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]

v = Vector3Stamped()
v.vector.x = 1
v.vector.y = 0
v.vector.z = 0

vt = tf2_geometry_msgs.do_transform_vector3(v, t)
test_similar('rotation', vt.vector, 0, 1, 0)