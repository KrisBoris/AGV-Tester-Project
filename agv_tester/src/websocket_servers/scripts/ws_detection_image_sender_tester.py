#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DetectionImageSenderNode:

    def __init__(self):

        rospy.init_node("detection_image_sender")

        self.__publisher = rospy.Publisher("/darknet_ros/detection_image", 
            Image, queue_size=1)
        self.__timer = rospy.Timer(rospy.Duration(1.0), self.__publish_image)

        self.__bridge = CvBridge()

    
    def __publish_image(self, event):
        # Create a sample OpenCV image (black image here)
        cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw something for testing
        cv2.putText(cv_image, "Test Image", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # Convert OpenCV image to ROS Image message
        image_msg = self.__bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        # Optionally set frame ID
        image_msg.header.frame_id = "camera_frame"
        # Publish
        self.__publisher.publish(image_msg)
        rospy.loginfo("Published test image")
        
        # image = Image()
        # image.header.seq = 1
        # image.header.stamp = 1.0
        # image.header.frame_id = "001"
        # image.height = 480
        # image.width = 640
        # image.encoding = 


if __name__ == "__main__":
    node = DetectionImageSenderNode()
    rospy.loginfo("detection_image_sender node has been created")
    rospy.spin()