#!/usr/bin/env python

import rospy
import os
import errno
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import base64
import json

class PipesWriterTesterNode:

    def __init__(self):
        
        rospy.init_node("pipes_writer_tester")

        self.__pipe_path = "/tmp/camera_image_pipe"
        self.__rate = rospy.Rate(1.0)

        self.__check_pipe()

        rospy.loginfo("pipes_writer_tester node has been created")


    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            rospy.loginfo("Creating a pipe")
            try:
                os.mkfifo(self.__pipe_path)
                rospy.loginfo("Pipe created")                                
            except OSError as e:
                # EEXIST - file already exists
                rospy.loginfo("Couldn't create a pipe")
                if e.errno != errno.EEXIST:                    
                    raise


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


    def run(self):                  
        rospy.loginfo("Before entering while loop")         
        while not rospy.is_shutdown():            
            try:    
                msg = CompressedImage()                                            

                # Fill header
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_frame"
                
                # Fill format and data
                msg.format = "jpeg"  # or "png"
                msg.data = b'\xff\xd8\xff\xe0'  # Example JPEG bytes (replace with real image data)
                
                rospy.loginfo("Before converting image")
                new_msg = self.__compressed_image_to_json(msg)

                rospy.loginfo("Before opening pipe")
                with open(self.__pipe_path, "w") as pipe:                                                   
                    
                    rospy.loginfo("Before writing to pipe")
                    pipe.write(new_msg + "\n")  
                    rospy.loginfo(str(new_msg))

                self.__rate.sleep()                    

            except Exception as e:
                rospy.loginfo("Exception: " + str(e))       


if __name__ == "__main__":
    node = PipesWriterTesterNode()    
    rospy.loginfo("Before starting loop")
    node.run()    