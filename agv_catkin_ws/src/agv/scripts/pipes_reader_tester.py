#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import os
import errno

class PipeReaderTesterNode:

    def __init__(self):
        
        rospy.init_node("pipe_reader_tester")

        self.__pipe_path = "/tmp/camera_image_pipe"

        self.__check_pipe()
        rospy.loginfo("pipe_reader_tester node has been created")

    
    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            try:
                os.mkfifo(self.__pipe_path)                                
            except OSError as e:
                # EEXIST - file already exists
                if e.errno != errno.EEXIST:                    
                    raise   


    def run(self):        
        while not rospy.is_shutdown():
            try:
                # Open the pipe for reading  
                with open(self.__pipe_path, "r") as pipe:                  
                    for line in pipe:
                        msg = line.strip()
                        if msg:
                            rospy.loginfo("Received message: " + str(msg))
            except Exception as e:
                print("Pipe read error:", e)                


if __name__ == "__main__":
    node = PipeReaderTesterNode()
    node.run()    