#!/usr/bin/env python3

import websockets
from websockets import server
import os
import asyncio
import errno
import json


class WebsocketServerNode:

    def __init__(self):

        self.__host_addr = "localhost"
        self.__host_port = 7890
        self.__connected_client = None
        self.__pipe_path = "/tmp/vehicle_drive_control_pipe"       
        self.__websocket_server = server.serve(self.process_vehicle_control_msg, self.__host_addr, self.__host_port)

        print("Vehicle drive control websocket server has been created")  

        # Check if all pipes already exist
        self.__check_pipe()      


    def __check_pipe(self):
        if not os.path.exists(self.__pipe_path):
            try:
                os.mkfifo(self.__pipe_path)                                
            except OSError as e:
                # EEXIST - file already exists
                if e.errno != errno.EEXIST:                    
                    raise
            

    async def process_vehicle_control_msg(self, websocket, path):  

        # Handle WebSocket connections
        if self.__connected_client:
            print("Rejecting new client: one already connected.")
            await websocket.close(code=1000, reason="Only one client allowed.")
            return

        print("Client connected.")
        self.__connected_client = websocket

        try:
            async for message in websocket:
                print(f"Sent: {message}")
                await self.write_to_pipe(message)                                
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected.")
        finally:
            if self.__connected_client == websocket:
                self.__connected_client = None                


    async def write_to_pipe(self, msg):
        try:
            with open(self.__pipe_path, "w") as pipe:
                pipe.write(msg + "\n")
        except Exception as e:
            print("Error writing to pipe:", e)


    def get_websocket_server(self):
        return self.__websocket_server
                
        
if __name__ == "__main__":
    server = WebsocketServerNode()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(server.get_websocket_server())
    print("Server started")
    loop.run_forever()     