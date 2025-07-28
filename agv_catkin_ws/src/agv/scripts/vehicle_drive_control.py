#!/usr/bin/env python

import rospy
import os
import errno
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class VehicleDriveControlNode:

    def __init__(self):
        
        rospy.init_node("vehicle_drive_control")
        
        self.__pipe_path = "/tmp/vehicle_drive_control_pipe"
        self.__publisher_vehicle_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=20) 

        self.__check_pipe()   

        rospy.loginfo("vehicle_drive_control node has been created")


    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            try:
                os.mkfifo(self.__pipe_path)                                
            except OSError as e:
                # EEXIST - file already exists
                if e.errno != errno.EEXIST:                    
                    raise


    def run(self):
        # Continuously read from the pipe and publish to ROS topic
        while not rospy.is_shutdown():
            try:
                with open(self.__pipe_path, "r") as pipe:
                    for line in pipe:                        
                        data = json.loads(line.strip())
                        if data:
                            msg = Twist()
                            if "linear" in data:
                                msg.linear.x = data["linear"]
                            if "angular" in data:
                                msg.angular.z = data["angular"]
                            rospy.loginfo("Publishing: %s", msg)
                            self.__publisher_vehicle_vel.publish(msg)
            except IOError as e:
                rospy.logwarn("IOError reading pipe: %s", e)                


if __name__ == "__main__":
    try:
        node = VehicleDriveControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node has been stopped")      