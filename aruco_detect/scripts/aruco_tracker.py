#!/usr/bin/env python

import roslib
import rospy

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
#import cv2.aruco as aruco
import glob
#import pyrealsense2 as rs

from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError



def camera_calibration():
    WAIT_TIME = 1
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/root/capture_board/*.jpg')

    i = 0

    for fname in images:

        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(WAIT_TIME)

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    cv_file = cv2.FileStorage("./camera_info.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

    return( mtx,dist)

def main():

    marker_size_in_mm = 145
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    mtx_,dist_=camera_calibration()

    ###------------------ ARUCO TRACKER ---------------------------
    while (True):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())


        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash

        if np.all(ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rotation_vector, translation_vector, _ = aruco.estimatePoseSingleMarkers(corners, marker_size_in_mm / 1000,
                                                                                     mtx_, dist_)
            # (rvec-tvec).any() # get rid of that nasty numpy value array error


            # R is rotation vector of marker relative to camera
            R, _ = cv2.Rodrigues(rotation_vector[0])

            print(translation_vector[0])
            print(R)


            for i in range(0, ids.size):
                # draw axis for the aruco markers
                aruco.drawAxis(color_image, mtx_, dist_, rotation_vector[i], translation_vector[i], 0.1)

            # draw a square around the markers
            aruco.drawDetectedMarkers(color_image, corners)

            # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0]) + ', '
            # ids is column vector.
            cv2.putText(color_image, "Id: " + strg, (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(color_image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        bridge=CvBridge()

        try :
            final_img_data=cv2.imread(color_image,mode="RGB")
            cv_image=bridge.imgmsg_to_cv2(final_img_data,"bgr8")
            img_pub.publish(bridge.cv2_to_imgmsg(cv_image,"bgr8"))

        except CvBridgeError as e :
            print(e)

        # display the resulting frame
        cv2.imshow('frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture

    cv2.destroyAllWindows()



if __name__ == '__main__':

    try:


        rospy.init_node("publish_informations")
        img_pub=rospy.Publisher("img_pub",Image)
        main()


        rospy.spin()

    except rospy.ROSInterruptException:
        pass




# References
# 1. https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
# 2. https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
# 3. https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
