#!/usr/bin/env python3

import rospy
import websockets
import asyncio
import json


class WebsocketSrvPublisherTesterNode:
    """ Creates ROS node responsible for connecting to given WebSocket server
        and periodically sending drive commands to it. """

    def __init__(self):
        
        rospy.init_node("ws_vehicle_drive_control_sender_tester")

        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "192.168.45.15")
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7893)
        self.__ws_srv_full_address = "ws://" + self.__ws_srv_address + ":" + str(self.__ws_srv_port)        

        self.__publishing_rate_value = rospy.get_param("publishing_rate", 0.5)
        self.__publishing_rate = rospy.Rate(self.__publishing_rate_value)

        rospy.loginfo("ws_vehicle_drive_control_sender_tester node has been created")


    async def send_messages(self):
        """ Connects to the server and sends messages periodically. """

        async with websockets.connect(self.__ws_srv_full_address) as websocket:
            while not rospy.is_shutdown():
                # Message fields correspond to geometry_msgs/msg/Twist message type
                drive_command = {
                    "linear": 0.5,
                    "angular": 0.1
                }
                await websocket.send(json.dumps(drive_command))
                rospy.loginfo(f"Sent: {drive_command}")
                
                self.__publishing_rate.sleep()


if __name__ == "__main__":
    rospy.loginfo("Starting WebSocket client")
    client = WebsocketSrvPublisherTesterNode()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(client.send_messages())