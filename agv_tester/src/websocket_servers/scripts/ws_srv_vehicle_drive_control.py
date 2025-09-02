#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import websockets
from websockets import server
import asyncio
import json


class WsSrvVehicleDriveControlNode:
    """ Creates ROS node responsible for creating WebSocket server that establishes
        connection with only the first client, receiving drive commands from it
        and publishing them to the given ROS topic. """


    def __init__(self):

        rospy.init_node("ws_srv_vehicle_drive_control")

        self.__drive_control_topic = rospy.get_param("~drive_control_topic", "/cmd_vel")
        self.__drive_control_queue_size = rospy.get_param("~drive_control_queue_size", 1)        
        self.__publisher_vehicle_drive_control = rospy.Publisher(
            self.__drive_control_topic, Twist, queue_size=self.__drive_control_queue_size)

        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "0.0.0.0")
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7890)                
        self.__websocket_server = server.serve(self.process_vehicle_control_msg,
            self.__ws_srv_address, self.__ws_srv_port)
        
        # Reference to the currently connected WebSocket client
        self.__connected_client = None

        rospy.loginfo("Vehicle drive control WebSocket server node has been created")                    
            

    async def process_vehicle_control_msg(self, websocket, path):  
        """ Establishes connection with only the first client 
            and publishes received messages to the topic. """
                
        if self.__connected_client:
            rospy.loginfo("Rejecting new client: one already connected.")
            await websocket.close(code=1000, reason="Only one client allowed.")
            return
        
        self.__connected_client = websocket
        rospy.loginfo("Client connected.")

        try:
            async for message in websocket:                                
                data = json.loads(message)
                
                # AGV requires only 'linear.x' field (drive forward or backward)
                # and 'angular.z' field (turn left or right)
                drive_command = Twist()
                drive_command.linear.x = data.get("linear", 0.0)
                drive_command.angular.z = data.get("angular", 0.0)

                self.__publisher_vehicle_drive_control.publish(drive_command)

        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo("Client disconnected.")
        finally:
            if self.__connected_client == websocket:
                self.__connected_client = None                    


    def get_websocket_server(self):
        """ Returns WebSocket server. """

        return self.__websocket_server
                
        
if __name__ == "__main__":    
    server = WsSrvVehicleDriveControlNode()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(server.get_websocket_server())
    rospy.loginfo("WebSocket server started")
    loop.run_forever()     