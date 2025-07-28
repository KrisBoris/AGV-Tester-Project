#!/usr/bin/env python3

import websockets
import os
import asyncio
import errno
import argparse


class WebsocketServerNode:

    def __init__(self, address="0.0.0.0", port=7890, pipe_path="/tmp/websocket_server_pipe"):

        self.__host_addr = address
        self.__host_port = port
        self.__connected_client = None
        self.__websocket_server = websockets.serve(self.process_client_connection, self.__host_addr, self.__host_port)
        self.__pipe_path = pipe_path            

        print("Websocket sender server has been created")  

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
            

    async def process_client_connection(self, websocket, path):
        # Handle WebSocket connections
        if self.__connected_client:
            print("Rejecting new client: one already connected.")
            await websocket.close(code=1000, reason="Only one client allowed.")
            return

        print("Client connected.")
        self.__connected_client = websocket

        try:
            await self.pipe_reader()
            await websocket.wait_closed()
        finally:
            print("Client disconnected.")
            if self.__connected_client == websocket:
                self.__connected_client = None


    async def pipe_reader(self):
        # Read data from pipe and send it to the connected client        
        while True:
            try:
                # Open the pipe for reading
                with open(self.__pipe_path, "r") as pipe:                
                    for line in pipe:
                        msg = line.strip()
                        if msg:
                            await self.send_to_client(msg)
                            print("Sent: " + str(msg))                
            except Exception as e:
                print("Pipe read error:", e)
                await asyncio.sleep(1)


    async def send_to_client(self, msg):
        # Send message to connected client if available
        if self.__connected_client:
            try:
                print("Sending message to the client")
                await self.__connected_client.send(msg)
                print("Message has been send")                
            except websockets.exceptions.ConnectionClosed:
                print("Client disconnected during send.")
                self.__connected_client = None
        else:
            print("No client connected. Dropping message:", msg)    


    def get_server(self):
        return self.__websocket_server
            
        
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7890)
    parser.add_argument("--pipe", default="/tmp/websocket_server_pipe")    
    args = parser.parse_args()

    server = WebsocketServerNode(address=args.address, port=args.port, pipe_path=args.pipe)                    
    loop = asyncio.get_event_loop()

    loop.run_until_complete(server.get_server())

    # ws_server = loop.run_until_complete(server.get_server())
    # loop.create_task(server.pipe_reader())
    print("Server has been started")
    loop.run_forever()