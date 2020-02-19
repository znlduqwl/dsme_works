#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
import os
import copy
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int8MultiArray


def image_save(image_array):

    if image_array.mode != 'RGB':
        im=image_array.convert('RGB')

        os.chdir(dir_path + folder_name[file_number] + "/homework#" + str(i))
        im.save(file_name[file_number] + ".jpg")


def callback(OccupancyGrid):
    mapdata = OccupancyGrid.data
    mapdata=np.asarray(mapdata)
    mapdata=mapdata.reshape(384,384)
    space=np.zeros((128,128))
    np.savetxt('/home/tak/ex_map.txt',mapdata)
    a = 0 
    b  =0
    c  = 0
    for i in range(127,255):
       for j in range(127,255):
          space[i-127,j-127]=copy.deepcopy(mapdata[i,j])


    for i in range(128):
       for j in range(128):
          if(space[i,j]==-1):
             a=a+1
             space[i,j]=0
          elif(space[i,j]==0):
             b=b+1
             space[i,j]=255

          elif(space[i,j]==100):
             c=c+1
             space[i,j]=255
          #else if(mapdata[i,j]!=-1):
    #img = Image.fromarray(mapdata)
    img = Image.fromarray(space,mode='L')
    img.save('/home/tak/my.png')
    #img.show()

    print(a)
    print(b)
    print(c)
    print("done")


def somethingCool():
    global mapdata
    mapdata = Int8MultiArray()
    rospy.init_node('test_name', anonymous=False)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.loginfo(mapdata)
    rospy.spin()


if __name__ == '__main__':
    try:
        somethingCool()
    except rospy.ROSInterruptException:
        pass
