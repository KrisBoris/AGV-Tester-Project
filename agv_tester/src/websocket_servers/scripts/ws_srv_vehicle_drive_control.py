#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import websockets
from websockets import server
import asyncio
import signal
import json


class WsSrvVehicleDriveControlNode:
    """ Creates ROS node responsible for creating WebSocket server that establishes
        connection with only the first client, receiving drive commands from it
        and publishing them to the given ROS topic """


    def __init__(self):

        # Disable signals allows shutdown (e.g. Ctrl+C) to be handled by asyncio
        rospy.init_node("ws_srv_vehicle_drive_control", disable_signals=True)

        # Topic to publish messages to control vehicle's movement
        self.__drive_control_topic = rospy.get_param("~drive_control_topic", "/cmd_vel")
        # Queue (buffer) size for sent vehicle's movement control messages
        self.__drive_control_queue_size = rospy.get_param("~drive_control_queue_size", 1)        
        # Publisher to the topic to control vehicle's movement
        self.__publisher_vehicle_drive_control = rospy.Publisher(
            self.__drive_control_topic, Twist, queue_size=self.__drive_control_queue_size)

        # WebSocket server IP address
        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "192.168.45.28")
        # WebSocket server port
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7893)                
        # WebSocket server object
        self.__websocket_server = None
        
        # Reference to the currently connected WebSocket client
        self.__connected_client = None

        rospy.loginfo("Vehicle drive control WebSocket server node has been created")                    
            
    async def process_vehicle_control_msg(self, websocket, path):  
        """ Establishes connection with only the first client 
            and publishes received messages to the topic """
                
        if self.__connected_client:
            rospy.loginfo("Rejecting new client: one already connected.")
            await websocket.close(code=1000, reason="Only one client allowed.")
            return
        
        self.__connected_client = websocket
        rospy.loginfo("Client connected.")

        try:
            async for message in websocket:                                
                data = json.loads(message)
                linear = data.get("linear", {0.0, 0.0, 0.0})
                angular = data.get("angular", {0.0, 0.0, 0.0})
                
                # AGV requires only 'linear.x' field (drive forward or backward)
                # and 'angular.z' field (turn left or right)
                drive_command = Twist()
                drive_command.linear.x = linear.get("x", 0.0)
                drive_command.angular.z = angular.get("z", 0.0)

                self.__publisher_vehicle_drive_control.publish(drive_command)

        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo("Client disconnected.")
        finally:
            if self.__connected_client == websocket:
                self.__connected_client = None                    

    async def start_server(self):
        """ Starts WebSocket server asynchronously """        
        self.__websocket_server = await server.serve(self.process_vehicle_control_msg,
            self.__ws_srv_address, self.__ws_srv_port)
        rospy.loginfo("WebSocket server started")
        await self.__websocket_server.wait_closed()

    async def shutdown(self):
        """ Shutdowns both WebSocket server and ROS node cleanly """
        if self.__websocket_server:
            self.__websocket_server.close()
            await self.__websocket_server.wait_closed()
        rospy.signal_shutdown("Shutdown requested")
        rospy.loginfo("Shutdown complete")    
                
        
if __name__ == "__main__":    
    server_node = WsSrvVehicleDriveControlNode()
    loop = asyncio.get_event_loop()

    # Sets up event loop to handle Ctrl+C interrupts
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.ensure_future(server_node.shutdown()))

    try:        
        loop.run_until_complete(server_node.start_server())
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received - shutting down")
    finally:
        loop.run_until_complete(server_node.shutdown())
        loop.close()        