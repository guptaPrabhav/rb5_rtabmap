#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml

class CamImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
                
        # Subscribers
        rospy.Subscriber("/stereo_front/left", Image, self.stereo_front_left_callback)
        rospy.Subscriber("/stereo_front/right", Image, self.stereo_front_right_callback)
                                                       
        # Publishers
        # Change publishers according to Stereo setup and what is expected by RTAB-MAP
                                                                               
        self.stereo_front_left_image_pub = rospy.Publisher("/stereo/left/image_raw", Image, queue_size=1000)
        self.stereo_front_left_info_pub = rospy.Publisher("/stereo/left/camera_info", CameraInfo, queue_size=1000)
        self.stereo_front_right_image_pub = rospy.Publisher("/stereo/right/image_raw", Image, queue_size=1000)
        self.stereo_front_right_info_pub = rospy.Publisher("/stereo/right/camera_info", CameraInfo, queue_size=1000)
        
        self.camera_info_left = self.load_camera_info_left()
        
        self.camera_info_right = self.load_camera_info_right()

        rospy.Timer(rospy.Duration(1.0 / 30), self.publish_stereo_front_left_info)
        rospy.Timer(rospy.Duration(1.0 / 30), self.publish_stereo_front_right_info)

    # Add another for the two camera's in the stereo setup

    # Edit the below two methods for extracting the correct information later...

    def load_camera_info_left(self):
        camera_info_msg = CameraInfo()
        camera_info_msg.height = 480
        camera_info_msg.width = 640
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = [ -1.6571748643440132e-01, 6.3134583515870882e-02, 2.4908601395800438e-03, 6.9258577723375913e-04, 0. ]
        camera_info_msg.K = [ 5.0159907997903338e+02, 0., 2.9377341710319376e+02, 0., 5.0083699409439379e+02, 2.3544444409742863e+02, 0., 0., 1. ]
        camera_info_msg.P = [ 5.0159907997903338e+02, 0., 2.9377341710319376e+02, 0., 5.0083699409439379e+02, 2.3544444409742863e+02, 0., 0., 1., 0., 0., 0. ]
        camera_info_msg.R = [ 9.9995382093423246e-01, -9.1903872613942166e-03, -2.8094093711296423e-03, 9.2509466314868449e-03, 9.9970714036476482e-01, 2.2361875818584419e-02, 2.6030723098620125e-03, -2.2386832864208624e-02, 9.9974599460505953e-01 ]
        return camera_info_msg
   
    def load_camera_info_right(self):
        camera_info_msg = CameraInfo()
        camera_info_msg.height = 480
        camera_info_msg.width = 640
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = [ -1.6640627389329365e-01, 6.4800083011513396e-02, 1.1988146735987267e-04, -6.3680006718804514e-04, 0. ]
        camera_info_msg.K = [ 5.0289492433892644e+02, 0., 3.1156572782508289e+02, 0., 5.0234014337071841e+02, 2.4962793784523797e+02, 0., 0., 1. ]
        camera_info_msg.P = [ 5.0289492433892644e+02, 0., 3.1156572782508289e+02, 0., 5.0234014337071841e+02, 2.4962793784523797e+02, 0., 0., 1., -39.79910431218264, 0., 0. ]
        camera_info_msg.R = [ 9.9995382093423246e-01, -9.1903872613942166e-03, -2.8094093711296423e-03, 9.2509466314868449e-03, 9.9970714036476482e-01, 2.2361875818584419e-02, 2.6030723098620125e-03, -2.2386832864208624e-02, 9.9974599460505953e-01 ]
        return camera_info_msg

    # Add another rgb_callback for the second stereo camera
    def stereo_front_left_callback(self, msg):
        try:
            # Convertir le message ROS en une image OpenCV
            raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Conversion YUV422 ou NV12 en RGB8
            if msg.encoding == "yuv422":
                rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_YUV2RGB_Y422)
            elif msg.encoding == "nv12":
                yuv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height * 3 // 2, msg.width))
                rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_NV12)
            elif msg.encoding == "mono8":
                rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_GRAY2RGB)
                converted_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
                converted_msg.header.frame_id = 'stereo_front_left_link'
                converted_msg.header.stamp = rospy.Time.now()
                self.stereo_front_left_image_pub.publish(converted_msg)
                return 
            else:
                rospy.logerr("1. Unsupported encoding: {}".format(msg.encoding))
                return

            # Publish the converted image
            converted_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            converted_msg.header.frame_id = 'stereo_front_left_link'
            converted_msg.header.stamp = rospy.Time.now()
            self.stereo_front_left_image_pub.publish(converted_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def publish_stereo_front_left_info(self, event):
        self.camera_info_left.header.stamp = rospy.Time.now()
        self.camera_info_left.header.frame_id = 'stereo_front_left_link'
        self.stereo_front_left_info_pub.publish(self.camera_info_left)


    def stereo_front_right_callback(self, msg):
        try:
            # Convertir le message ROS en une image OpenCV
            right_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Conversion YUV422 ou NV12 en RGB8
            if msg.encoding == "yuv422":
                right_rgb_image = cv2.cvtColor(right_raw_image, cv2.COLOR_YUV2RGB_Y422)
            elif msg.encoding == "nv12":
                right_yuv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height * 3 // 2, msg.width))
                right_rgb_image = cv2.cvtColor(right_yuv_image, cv2.COLOR_YUV2RGB_NV12)
            elif msg.encoding == "mono8":
                right_rgb_image = cv2.cvtColor(raw_image,cv2.COLOR_GRAY2RGB)
                right_converted_msg = self.bridge.cv2_to_imgmsg(right_rgb_image, encoding="rgb8")
                right_converted_msg.header.frame_id = 'stereo_front_left_link'
                right_converted_msg.header.stamp = rospy.Time.now()
                self.stereo_front_right_image_pub.publish(right_converted_msg)
                return
            else:
                rospy.logerr("2. Unsupported encoding: {}".format(msg.encoding))
                return
            
            # Publish the converted image
            right_converted_msg = self.bridge.cv2_to_imgmsg(right_rgb_image, encoding="rgb8")
            right_converted_msg.header.frame_id = 'stereo_front_right_link'
            right_converted_msg.header.stamp = rospy.Time.now()
            self.stereo_front_right_image_pub.publish(right_converted_msg)
        
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    
    def publish_stereo_front_right_info(self, event):
        self.camera_info_right.header.stamp = rospy.Time.now()
        self.camera_info_right.header.frame_id = 'stereo_front_right_link'
        self.stereo_front_right_info_pub.publish(self.camera_info_right)


def main():
    rospy.init_node('cam_image_converter', anonymous=True)
    CamImageConverter()
    rospy.spin()

if __name__=='__main__':
    main()
