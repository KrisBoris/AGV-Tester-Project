#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import websockets
import asyncio
import threading
import json
import base64


class WsSrvCameraImageSenderNode:
    """ Creates ROS node responsible for receiving camera images
        from given ROS topic and creating WebSocket server that establishes
        connection with only the first client and sends the received images to it. """

    def __init__(self):

        rospy.init_node("ws_srv_camera_image_sender")

        self.__camera_image_topic = rospy.get_param("~camera_image_topic", "/csi_cam_0/image_raw/compressed")
        self.__camera_image_queue_size = rospy.get_param("~camera_image_queue_size", 1)
        self.__subscriber = rospy.Subscriber(self.__camera_image_topic, CompressedImage,
            self.callback_camera_image, queue_size=self.__camera_image_queue_size)

        self.__ws_srv_address = rospy.get_param("~ws_srv_address", "0.0.0.0")
        self.__ws_srv_port = rospy.get_param("~ws_srv_port", 7890)
        # Reference to the currently connected WebSocket client
        self.__connected_client = None        
        
        # Asynchronous event loop dedicated to sending images to the WebSocket client
        self.__images_sending_loop = asyncio.new_event_loop()
        # Asynchronous event loop dedicated to running WebSocket server
        self.__ws_srv_loop = asyncio.new_event_loop()

        rospy.loginfo("ws_srv_camera_image_sender node has been created")          

    
    def callback_camera_image(self, received_image: CompressedImage):  
        """ Runs coroutine in the seperate thread 
            which sends received image to the client. """      

        asyncio.run_coroutine_threadsafe(self.send_to_client(received_image), self.__images_sending_loop)
       

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
    

    async def send_to_client(self, image: CompressedImage):
        """ Converts received image to JSON format 
            and sends it to the connected client. """

        if self.__connected_client:
            try:   
                image_json = self.convert_image_to_json(image)            
                await self.__connected_client.send(image_json)                
            except websockets.exceptions.ConnectionClosed:
                rospy.loginfo("Client disconnected during send.")
                self.__connected_client = None
        else:
            rospy.loginfo("No client connected. Dropping message.")    


    def convert_image_to_json(self, image: CompressedImage) -> str:
        """ Converts received image to JSON format.
            Received image must be instance of CompressedImage class. """

        return json.dumps({
            "header": {
                "seq": image.header.seq,
                "stamp": image.header.stamp.to_sec(),
                "frame_id": image.header.frame_id
            },
            "format": image.format,
            "data": base64.b64encode(image.data).decode('utf-8')
        })


    def start_images_receiving_loop(self):
        """ Starts event loop for sending images to the client. """

        asyncio.set_event_loop(self.__images_sending_loop)
        self.__images_sending_loop.run_forever()


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
    server = WsSrvCameraImageSenderNode()
    threading.Thread(target=server.start_images_receiving_loop, daemon=True).start()
    threading.Thread(target=server.start_ws_srv_loop, daemon=True).start()
    rospy.spin()    