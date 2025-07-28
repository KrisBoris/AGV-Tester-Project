#!/usr/bin/env python3

import websockets
import asyncio


class WebsocketSrvClientTester:

    def __init__(self):
        self.__websocket_srv_address = "ws://0.0.0.0:7890"


    async def receiver_messages(self):
        print("Connectiing to the server")
        async with websockets.connect(self.__websocket_srv_address) as websocket:
            print("Connection successful")
            while True:
                try:
                    message = await websocket.recv()
                    print(f"Received: {message}")
                except websockets.ConnectionClosed:
                    print("Connection closed by server.")
                    break


if __name__ == "__main__":
    print("Starting WebSocket client")
    client = WebsocketSrvClientTester()
    loop = asyncio.get_event_loop()

    try:
        loop.run_until_complete(client.receiver_messages())
    except KeyboardInterrupt:
        print("\nClient stopped manually.")
    finally:
        loop.close()