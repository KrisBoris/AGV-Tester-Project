#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

class CameraImageTesterNode:

    def __init__(self):

        rospy.init_node("camera_image_tester")

        self.__publisher_camera_image = rospy.Publisher("/csi_cam_0/image_raw/compressed", CompressedImage, queue_size=20)
        self.__rate = rospy.Rate(2.0)

        rospy.loginfo("camera_image_tetser node has been created")


    def run(self):
        while not rospy.is_shutdown():
            msg = CompressedImage()

             # Fill header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_frame"
            
            # Fill format and data
            msg.format = "jpeg"  # or "png"
            msg.data = b'\xff\xd8\xff\xe0'  # Example JPEG bytes (replace with real image data)
            
            self.__publisher_camera_image.publish(msg)
            rospy.loginfo("Published CompressedImage")
            self.__rate.sleep()
            

if __name__ == "__main__":
    node = CameraImageTesterNode()
    node.run()