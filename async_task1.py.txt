#!/usr/bin/env python3
import json
import asyncio
import websockets
import struct
import cv2
from picamera2 import Picamera2
from libcamera import Transform
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT

WS_SERVER_URI = f"ws://{API_IP}:{API_PORT}/image"  # WebSocket server URL
CHUNK_SIZE = 60000  # Adjust as necessary

class PiAction:
    """
    Class that represents an action that the RPi needs to take.    
    """

    def init(self, cat, value):
        """
        :param cat: The category of the action.
        :param value: The value of the action.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value


class RaspberryPi:
    """
    Class that represents the Raspberry Pi with asynchronous functionalities.
    """

    def init(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()
        self.android_link = AndroidLink()
        self.stm_link = STMLink()

        self.android_queue = asyncio.Queue()  # Messages to send to Android
        self.rpi_action_queue = asyncio.Queue()  # Messages that need to be processed by RPi
        self.command_queue = asyncio.Queue()  # Messages for STM32 and snap commands

        self.success_obstacles = []
        self.failed_obstacles = []
        self.obstacles = {}
        self.current_location = {}
        self.failed_attempt = False

    async def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###
            await self.android_link.connect()
            await self.stm_link.connect()

            # Define child tasks to run concurrently
            await asyncio.gather(
                self.recv_android(),
                self.recv_stm(),
                self.android_sender(),
                self.command_follower(),
                self.rpi_action()
            )
        except KeyboardInterrupt:
            await self.stop()

    async def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        await self.android_link.disconnect()
        await self.stm_link.disconnect()
        self.logger.info("Program exited!")

    async def snap_and_rec(self, obstacle_id_with_signal: str):
        """
        Captures an image, sends it over WebSocket for image recognition, and receives the response from FastAPI server.

        :param obstacle_id_with_signal: The current obstacle ID followed by underscore followed by signal.
        """
        obstacle_id, signal = obstacle_id_with_signal.split("_")

        # Initialize camera
        picam2 = Picamera2()
        camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, transform=Transform(vflip=True, hflip=True))
        picam2.configure(camera_config)
        picam2.start()

        # Capture an image
        frame = picam2.capture_array()

        # Encode the image to JPEG format
        result, imgencode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        img_data = imgencode.tobytes()
        img_size = len(img_data)
        print(f"Image size: {img_size} bytes")

        # Send image over WebSocket and receive the response
        async with websockets.connect(WS_SERVER_URI) as websocket:
            try:
                # Step 1: Send the image size first
                await websocket.send(struct.pack('i', img_size))

                # Step 2: Send image in chunks
                for i in range(0, img_size, CHUNK_SIZE):
                    chunk = img_data[i:i + CHUNK_SIZE]
                    is_last_chunk = 1 if (i + CHUNK_SIZE) >= img_size else 0
                    await websocket.send(struct.pack('B', is_last_chunk))  # Flag to indicate if it's the last chunk
                    await websocket.send(chunk)  # Send the chunk

                print(f"Image sent for obstacle {obstacle_id} and signal {signal}.")

                # Step 3: Wait for a response from the server (e.g., image ID or detection result)
                response = await websocket.recv()
                response_data = json.loads(response)
                print(f"Response from FastAPI: {response_data}")

            except Exception as e:
                print(f"Error during WebSocket communication: {e}")

            finally:
                picam2.stop()

    async def recv_android(self) -> None:
        """
        [Child Task] Processes the messages received from Android.
        """
        while True:
            try:
                msg_str = await self.android_link.recv_async()
                if msg_str:
                    message = json.loads(msg_str)
                    await self.process_android_message(message)
            except OSError:
                self.logger.error("Android link is down!")

    async def process_android_message(self, message: dict):
        """Process received message from Android"""
        if message['cat'] == "obstacles":
            await self.rpi_action_queue.put(PiAction(**message))
        elif message['cat'] == "control" and message['value'] == "start":
            await self.command_follower()

    async def recv_stm(self) -> None:
        """
        [Child Task] Receive acknowledgement messages from STM32, and release the movement lock.
        """
        while True:
            message = await self.stm_link.recv_async()

            if message.startswith("ACK"):
                self.logger.debug("ACK from STM32 received.")
                cur_location = await self.path_queue.get()
                self.current_location.update(cur_location)
                self.android_queue.put_nowait(AndroidMessage('location', cur_location))

    async def android_sender(self) -> None:
        """
        [Child Task] Responsible for sending messages to Android.
        """
        while True:
            message = await self.android_queue.get()
            await self.android_link.send_async(message)

    async def command_follower(self) -> None:
        """
        [Child Task] Follow commands from STM32 and process them accordingly.
        """
        while True:
            command = await self.command_queue.get()

            if command.startswith("SNAP"):
                obstacle_id_with_signal = command.replace("SNAP", "")
                await self.snap_and_rec(obstacle_id_with_signal)

    async def rpi_action(self) -> None:
        """
        [Child Task] Handle RPi actions based on PiAction queue.
        """
        while True:
            action = await self.rpi_action_queue.get()

            if action.cat == "obstacles":
                await self.process_obstacles(action.value)
            elif action.cat == "snap":
                await self.snap_and_rec(action.value)

    async def process_obstacles(self, obstacles):
        """Processes obstacles"""
        self.obstacles.update({obs['id']: obs for obs in obstacles})
        await self.request_algo(obstacles)

    async def request_algo(self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False):
        """
        Asynchronously requests for path and commands from the Algo API.
        """
        url = f"http://{API_IP}:{API_PORT}/path"
        async with websockets.connect(url) as websocket:
            body = {**data, "robot_x": robot_x, "robot_y": robot_y, "robot_dir": robot_dir, "retrying": retrying}
            await websocket.send(json.dumps(body))
            response = await websocket.recv()
            result = json.loads(response)

            commands = result['commands']
            path = result['path']

            for command in commands:
                await self.command_queue.put(command)

            for p in path[1:]:  # ignore the first position
                await self.path_queue.put(p)
    async def request_stitch(self):
        """Asynchronously requests to stitch images."""
        url = f"http://{API_IP}:{API_PORT}/stitch"
        async with websockets.connect(url) as websocket:
            await websocket.send(json.dumps({"action": "stitch"}))
            response = await websocket.recv()
            self.logger.info("Images stitched!")

if __name__ == "main":
    rpi = RaspberryPi()
    asyncio.run(rpi.start())