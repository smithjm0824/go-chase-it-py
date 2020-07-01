#! /usr/bin/env python

import rospy
import math
import numpy
import cv2
from cv_bridge import CvBridge
from collections import Counter
from sensor_msgs.msg import Image
from ball_chaser.srv import DriveToTarget, DriveToTargetRequest

client = None

def drive_robot(lin_x, ang_z):
    rospy.loginfo("Chasing the ball.")

    srv = DriveToTargetRequest()
    srv.linear_x = lin_x
    srv.angular_z = ang_z

    try:
        client(srv)
    except Exception as e:
        rospy.logerr(e)

def process_image_callback(img):
    bridge = CvBridge()
    cv_image = numpy.asarray(bridge.imgmsg_to_cv2(img, desired_encoding="bgr8"))
    rows, cols, pixels = cv_image.shape
    step = cols * pixels
    cv_image = cv_image.reshape(rows * cols, pixels)
    white_pixel = 255
    left, middle, right = 0, 0, 0
    unique_white_pixels = numpy.unique(numpy.where(cv_image == [white_pixel, white_pixel, white_pixel]))
      
    def determine_third(index):
        return math.floor(((float(index) % float(rows)) / float(cols)) * 3.0)

    if len(unique_white_pixels) > 0:
        vectorized_third = numpy.vectorize(determine_third)
        counts = Counter(vectorized_third(unique_white_pixels))
        max_occur = counts.most_common(1)[0][0]

        if int(max_occur) == 0:
            left += 1
        elif int(max_occur) == 1:
            middle += 1
        elif int(max_occur) == 2:
            right += 1
    
    if left > middle and left > right:
        drive_robot(0.25, 0.5)
    elif middle > left and middle > right:
        drive_robot(0.25, 0)
    elif right > left and right > middle:
        drive_robot(0.25, -0.5)
    else:
        drive_robot(0, 0.5)

if __name__ =='__main__':
    rospy.init_node("process_image")
    client = rospy.ServiceProxy("/ball_chaser/command_robot", DriveToTarget)
    subscriber = rospy.Subscriber(name="/camera/rgb/image_raw", data_class=Image, queue_size=10, callback=process_image_callback)

    rospy.spin()
    