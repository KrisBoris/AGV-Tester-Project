#!/usr/bin/env python3

import websockets
import asyncio
import json


class WebsocketSrvPublisherTester:

    def __init__(self):
        self.__websocket_srv_address = "ws://localhost:7890"


    async def send_messages(self):
        async with websockets.connect(self.__websocket_srv_address) as websocket:
            while True:
                message = {
                    "linear": 1.0,
                    "angular": 0.5
                }
                await websocket.send(json.dumps(message))
                print(f"Sent: {message}")
                await asyncio.sleep(1)  # Wait 1 second


if __name__ == "__main__":
    print("Starting WebSocket client")
    client = WebsocketSrvPublisherTester()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(client.send_messages())