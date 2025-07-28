#!/usr/bin/env python

import rospy
import os
import errno
from sensor_msgs.msg import CompressedImage
import json
import base64

class CameraImageSenderNode:

    def __init__(self):
        
        rospy.init_node("camera_image_sender")

        self.__pipe_path = "/tmp/camera_image_pipe"        
        self.__subscriber_camera_image = rospy.Subscriber("/csi_cam_0/image_raw/compressed", CompressedImage, self.__callback_camera_image, queue_size=20)

        self.__check_pipe

        rospy.loginfo("camera_image_sender node has been created")


    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            try:
                os.mkfifo(self.__pipe_path)                                
            except OSError as e:
                # EEXIST - file already exists
                if e.errno != errno.EEXIST:                    
                    raise    


    def __callback_camera_image(self, compressed_image):
        try:
            rospy.loginfo("Captured image: " + str(compressed_image.header.stamp) + "\n" + str(compressed_image.header.frame_id) + "\n" + str(compressed_image.format))
            with open(self.__pipe_path, "w") as pipe:
                msg = self.__compressed_image_to_json(compressed_image)
                pipe.write(msg + "\n")
        except Exception as e:
            print("Error writing to pipe:", e)

    
    def __compressed_image_to_json(self, msg):
        return json.dumps({
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "format": msg.format,
            "data": base64.b64encode(msg.data).decode('utf-8')
        })


if __name__ == "__main__":
    CameraImageSenderNode()
    rospy.spin()