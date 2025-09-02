#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import websockets
import asyncio
import threading
import json


class WsSrvDetectedObjectsReceiverNode:
    """ Creates ROS node responsible for receiving detected objects
        from given ROS topic and creating WebSocket server that establishes
        connection with only the first client and sends the received objects to it. """

    def __init__(self):

        rospy.init_node("ws_srv_detected_objects_receiver")

        self.__detected_objects_topic = rospy.get_param("~detected_objects_topic", "/darknet_ros/bounding_boxes")
        self.__detected_objects_queue_size = rospy.get_param("~detected_objects_queue_size", 1)
        self.__subscriber = rospy.Subscriber(self.__detected_objects_topic, BoundingBoxes,
            self.callback_detected_objects, queue_size=self.__detected_objects_queue_size)
        self.__detected_object_probality_threshold = rospy.get_param("~detected_object_probality_threshold", 0.7)

        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "0.0.0.0")
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7890)
        # Reference to the currently connected WebSocket client
        self.__connected_client = None        
        
        # Asynchronous event loop dedicated to sending detected objects to the WebSocket client
        self.__detected_objects_sending_loop = asyncio.new_event_loop()
        # Asynchronous event loop dedicated to running WebSocket server
        self.__ws_srv_loop = asyncio.new_event_loop()

        rospy.loginfo("ws_srv_detected_objects_receiver node has been created")          

    
    def callback_detected_objects(self, received_objects):  
        """ Runs coroutine in the seperate thread 
            which sends received detected objects to the client. """      

        asyncio.run_coroutine_threadsafe(self.send_to_client(received_objects), self.__detected_objects_sending_loop)
       

    async def process_client_connection(self, websocket, path):
        """ Establishes connection with only the first client. """

        if self.__connected_client:
            rospy.loginfo("Rejecting new client: one already connected.")
            await websocket.close(code=1000, reason="Only one client allowed.")
            return

        rospy.loginfo("Client connected.")
        self.__connected_client = websocket

        try:            
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo("Client disconnected.")
        finally:            
            if self.__connected_client == websocket:
                self.__connected_client = None
    

    async def send_to_client(self, received_objects: BoundingBoxes):
        """ Checks if detected objects' probability is higher then set threshold
            and if so sends the objects' names and count to the client. """

        if self.__connected_client:
            try:  
                detected_objects_names_count = {}
                
                for det_obj in received_objects.bounding_boxes:
                    
                    if det_obj.probability > self.__detected_object_probality_threshold:                    
                       
                        if det_obj.Class in detected_objects_names_count:
                            # Increment object's counter
                            detected_objects_names_count[det_obj.Class] = detected_objects_names_count[det_obj.Class] + 1
                        else:
                            # Create a new object
                            detected_objects_names_count[det_obj.Class] = 1                        
                
                detected_objects_json = json.dumps(detected_objects_names_count)
                await self.__connected_client.send(detected_objects_json)                                                        

            except websockets.exceptions.ConnectionClosed:
                rospy.loginfo("Client disconnected during send.")
                self.__connected_client = None
        else:
            rospy.loginfo("No client connected. Dropping message.")    


    def start_objects_receiving_loop(self):
        """ Starts event loop for sending detected objects to the client. """

        asyncio.set_event_loop(self.__detected_objects_sending_loop)
        self.__detected_objects_sending_loop.run_forever()


    def start_ws_srv_loop(self):
        """ Starts event loop for handling WebSocket connection. """

        asyncio.set_event_loop(self.__ws_srv_loop)
        
        # Server object have to be created inside the thread where it is called
        websocket_server = websockets.serve(self.process_client_connection, 
            self.__ws_srv_address, self.__ws_srv_port)
        
        self.__ws_srv_loop.run_until_complete(websocket_server)
        self.__ws_srv_loop.run_forever()
        rospy.loginfo("Server has been started")
            
        
if __name__ == "__main__":       
    server = WsSrvDetectedObjectsReceiverNode()
    threading.Thread(target=server.start_objects_receiving_loop, daemon=True).start()
    threading.Thread(target=server.start_ws_srv_loop, daemon=True).start()
    rospy.spin()    