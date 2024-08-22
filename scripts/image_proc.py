#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

#from ultralytics import YOLO
#import torch

#model = YOLO('yolov8m.pt')

def callback(data):
    br = CvBridge()
    
    rospy.loginfo("Receiving Video Frame!")

    current_frame = br.imgmsg_to_cv2(data)
    print("Frsame converted")

    # results = model.predict(current_frame, conf=0.5)

    # print(results)

    # cv2.imshow("camera", current_frame)
    # cv2.waitKey(1)

def receive_message():
    rospy.init_node('image_processing', anonymous=True)
    rospy.Subscriber('/hires_small_color', Image, callback)
    rospy.spin()
    # cv2.destroyAllWindows()

if __name__=='__main__':
    receive_message()
