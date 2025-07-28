#!/usr/bin/env python

import rospy
import os
import errno
from sensor_msgs.msg import CompressedImage

class ObjectRecognitionNode:

    def __init__(self):
        
        rospy.init_node("object_recognition_node")

        self.__pipe_path = "/tmp/object_recognition_pipe"        
        self.__subscriber_camera_image = rospy.Subscriber("/csi_cam_0/image_raw/compressed", CompressedImage, self.__callback_camera_image, queue_size=20)

        self.__check_pipe


    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            try:
                os.mkfifo(self.__pipe_path)                                
            except OSError as e:
                # EEXIST - file already exists
                if e.errno != errno.EEXIST:                    
                    raise    


    def __callback_camera_image(self, camera_image):
        try:

            # Call object recognition model here, then send the result to pipe

            with open(self.__pipe_path, "w") as pipe:
                pipe.write(camera_image + "\n")
        except Exception as e:
            print("Error writing to pipe:", e)


if __name__ == "__main__":
    ObjectRecognitionNode()
    rospy.spin()