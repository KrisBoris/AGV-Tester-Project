#!/usr/bin/env python3

import rospy
import websockets
import asyncio
import json


class WebsocketSrvClientTesterNode:
    """ Creates ROS node responsible for receiving images
        from WebSocket server and displaying their data. """

    def __init__(self):        

        rospy.init_node("ws_camera_image_receiver_tester")

        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "192.168.45.107")
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7891)
        self.__ws_srv_full_address = "ws://" + self.__ws_srv_address + ":" + str(self.__ws_srv_port)        

        rospy.loginfo("ws_camera_image_receiver_tester node has been created")


    async def messages_receiver(self):
        """ Connects to the server and receives messages sent by it. """
        
        rospy.loginfo("Connectiing to the server")
        async with websockets.connect(self.__ws_srv_full_address) as websocket:
            rospy.loginfo("Connection successful")
            while not rospy.is_shutdown():
                try:
                    message = await websocket.recv()
                    image = json.loads(message)
                   
                    # Display image's data
                    rospy.loginfo("------------------------------------")
                    if "header" in image:
                        if "seq" in image["header"]:
                            rospy.loginfo("seq: " + str(image["header"]["seq"]))
                        if "stamp" in image["header"]:
                            rospy.loginfo("stamp: " + str(image["header"]["stamp"]))
                        if "frame_id" in image["header"]:
                            rospy.loginfo("frame_id: " + str(image["header"]["frame_id"]))
                    if "format" in image:
                        rospy.loginfo("format: " + str(image["format"]))
                    rospy.loginfo("------------------------------------")                  

                except websockets.ConnectionClosed:
                    rospy.loginfo("Connection closed by server.")
                    break


if __name__ == "__main__":
    rospy.loginfo("Starting WebSocket client")
    
    client = WebsocketSrvClientTesterNode()
    loop = asyncio.get_event_loop()

    try:
        loop.run_until_complete(client.messages_receiver())
    except KeyboardInterrupt:
        print("\nClient stopped manually.")
    finally:
        loop.close()