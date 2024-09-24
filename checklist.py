#!/usr/bin/env python3
import json
import queue
from datetime import datetime
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_IP_START, API_IP_END, API_PORT
import socket
from picamera2 import Picamera2
from libcamera import Transform
import cv2
import io


class PiAction:
    """
    Class that represents an action that the RPi needs to take.    
    """

    def __init__(self, cat, value):
        """
        :param cat: The category of the action. Can be 'info', 'mode', 'path', 'snap', 'obstacle', 'location', 'failed', 'success'
        :param value: The value of the action. Can be a string, a list of coordinates, or a list of obstacles.
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
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()
        self.android_link = AndroidLink()
        self.stm_link = STMLink()

        self.manager = Manager()

        self.android_dropped = self.manager.Event()
        self.unpause = self.manager.Event()

        self.movement_lock = self.manager.Lock()

        self.android_queue = self.manager.Queue()  # Messages to send to Android
        # Messages that need to be processed by RPi
        self.rpi_action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()

        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None
        self.rs_flag = False
        self.success_obstacles = self.manager.list()
        self.failed_obstacles = self.manager.list()
        self.obstacles = self.manager.dict()
        self.current_location = self.manager.dict()
        self.failed_attempt = False
        
        # Create a global Camera Instance
        self.camera = None
        #self.initialize_camera()
        self.valid_api = API_IP + str(endpoint)

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###

            #self.android_link.connect()
            #self.android_queue.put(AndroidMessage('info', 'You are connected to the RPi!'))
            self.stm_link.connect()
            
            for endpoint in range(API_IP_START, API_IP_END+1):
                self.valid_api = API_IP + str(endpoint)  
                if self.check_api():
                    self.logger.debug("API successfully set up at", self.valid_api)
                    break
                elif endpoint == API_IP_END:
                    self.logger.warning("No valid IP address to connect API")

            # Define child processes
            #self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            #self.proc_android_sender = Process(target=self.android_sender)
            #self.proc_command_follower = Process(target=self.command_follower)
            #self.proc_rpi_action = Process(target=self.rpi_action)

            # Start child processes
            #self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            #self.proc_android_sender.start()
            #self.proc_command_follower.start()
            #self.proc_rpi_action.start()
            #self.rpi_action_queue.put(PiAction(cat="control", value={}))

            self.logger.info("Child Processes started")

            ### Start up complete ###

            # Send success message to Android
            #self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            #self.android_queue.put(AndroidMessage('mode', 'path'))
            #self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        #self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")    

    
    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:

            message: str = self.stm_link.recv()

            if message.startswith("ACK"):
                if self.rs_flag == False:
                    self.rs_flag = True
                    self.logger.debug("ACK for RS00 from STM32 received.")
                    continue
                try:
                    #if self.movement_lock.acquire(timeout=5):
                    self.movement_lock.release()
                    self.logger.debug("Movement lock released after ACK received!")
                    #self.logger.debug("Movement lock released!")
                    try:
                        self.retrylock.release()
                    except:
                        self.logger.debug("Failed to release retry lock")
                        pass

                    cur_location = self.path_queue.get_nowait()

                    self.current_location['x'] = cur_location['x']
                    self.current_location['y'] = cur_location['y']
                    self.current_location['d'] = cur_location['d']
                    # Update the current robot location
                    self.logger.info(
                        f"Current location = {self.current_location}")
                    # Send the new robot location to Andriod to be updated on the screen
                    self.android_queue.put(AndroidMessage('location', {
                        "x": cur_location['x'],
                        "y": cur_location['y'],
                        "d": cur_location['d'],
                    }))
                    
                    
                except:
                    self.logger.warning("Tried to release a released lock!")
            elif message.startswith("SNAP"):
                self.logger.info("Sending API requests to image server")
                _, obstacle_id = message.split('_')
                self.snap_and_rec(obstacle_id)
            else:
                self.logger.warning(
                    f"Ignored unknown message from STM: {message}")

    def android_sender(self) -> None:
        """
        [Child process] Responsible for retrieving messages from android_queue and sending them over the Android link. 
        """
        while True:
            # Retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.android_link.send(message)
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android dropped")

    def command_follower(self) -> None:
        """
        [Child Process] 
        """
        while True:
            # Retrieve next movement command
            
            try:
                command: str = self.command_queue.get()
                self.logger.debug(f"Next Command: {command}")
            except:
                self.logger.debug("Error getting the next command")
            self.logger.debug(f"wait for unpause - {command}")
            # Wait for unpause event to be true [Main Trigger]
            try:
                self.logger.debug(f"Trying to acquire retrylock - {command}")
                self.retrylock.acquire(timeout=1)
                self.retrylock.release()
                self.logger.debug(f"Acquired retrylock - {command}")
            except:
                self.logger.debug(f"Fail to acquire retry lock - {command}")
                self.unpause.wait()
            self.logger.debug(f"wait for movelock - {command}")
            # Acquire lock first (needed for both moving, and snapping pictures)
            self.movement_lock.acquire()
            self.logger.debug(f"Movement lock acquired! - {command}")
            
            # STM32 Commands - Send straight to STM32
            stm32_prefixes = ("FW", "BW", "FL", "FR", "BL",
                              "BR", "RS")
            if command.startswith(stm32_prefixes):
                    self.stm_link.send(command) 
                    self.logger.debug(f"Sending to STM32:{command}")

            # Snap command
            elif command.startswith("SNAP"):
                obstacle_id_with_signal = command.replace("SNAP", "")

                self.rpi_action_queue.put(
                    PiAction(cat="snap", value=obstacle_id_with_signal))

            # End of path
            elif command == "FN":
                self.logger.info(
                    f"At FN, self.failed_obstacles: {self.failed_obstacles}")
                self.logger.info(
                    f"At FN, self.current_location: {self.current_location}")
                if len(self.failed_obstacles) != 0 and self.failed_attempt == False:

                    new_obstacle_list = list(self.failed_obstacles)
                    for i in list(self.success_obstacles):
                        # {'x': 5, 'y': 11, 'id': 1, 'd': 4}
                        i['d'] = 8
                        new_obstacle_list.append(i)

                    self.logger.info("Attempting to go to failed obstacles")
                    self.failed_attempt = True
                    self.request_algo({'obstacles': new_obstacle_list, 'mode': '0'},
                                        self.current_location['x'], self.current_location['y'], self.current_location['d'], retrying=True)
                    self.retrylock = self.manager.Lock()
                    self.movement_lock.release()
                    continue

                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                #self.android_queue.put(AndroidMessage(
                    #"info", "Commands queue finished."))
                #self.android_queue.put(AndroidMessage("status", "finished"))
                #self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
            else:
                raise Exception(f"Unknown command: {command}")

                

    def rpi_action(self):
        """
        [Child Process] 
        """
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "obstacles":
                for obs in action.value['obstacles']:
                    self.obstacles[obs['obstacleNumber']] = obs
                self.request_algo(action.value)
            elif action.cat == "snap":
                self.snap_and_rec(obstacle_id_with_signal=action.value)
            elif action.cat == "stitch":
                self.request_stitch()
                
           
    def snap_and_rec(self, obstacle_id: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID and signal combined
        """
        #obstacle_id, signal = obstacle_id_with_signal.split('_')
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        #self.android_queue.put(AndroidMessage("info", f"Capturing image for obstacle id: {obstacle_id}"))

        try:
            # Initialize and configure the camera each time
            self.camera = Picamera2()
            camera_config = self.camera.create_still_configuration()
            self.camera.configure(camera_config)
            self.camera.start()

            # Capture the image
            frame = self.camera.capture_array()
            file_path = f'{datetime.now().strftime("%Y%m%d_%H%M%S")}_{obstacle_id}.jpg'
            cv2.imwrite(file_path, frame)
            self.logger.info("Image captured and saved.")

            # Stop and release the camera
            self.camera.stop()
            self.camera.close()
        except Exception as e:
            self.logger.error(f"Error capturing image: {str(e)}")
            #self.android_queue.put(AndroidMessage("error", "Failed to capture image."))
            return

        results = null
        
        while results == null:
            # Proceed with sending the image to the API
            url = f"http://{self.valid_api}:{API_PORT}/image"
                #self.logger.info("Img size: ", len(img))
            img_file = {'files': (file_path, open(file_path, 'rb'), 'image/jpeg')}
            data = {'obstacle_id': str(obstacle_id), 'signal': 'L'}
            
            response = requests.post(url, files=img_file, data=data)
            img_file['files'][1].close()
            if response.status_code == 200:
                self.logger.info("Image-rec API called successfully.")
            else:
                self.logger.error(f"Failed to call image-rec API: {response.status_code}")


            results = json.loads(response.content)
            if results == null:
                self.stm_link.send("BW20")
                moved = True

        # for stopping the robot upon finding a non-bullseye face (checklist: navigating around obstacle)
        if results.get("stop"):
            # stop issuing commands

            self.logger.info("Found non-bullseye face, remaining commands and path cleared.")
            #self.android_queue.put(AndroidMessage("info", "Found non-bullseye face, remaining commands cleared."))
        else: 
            self.stm_link.send("asdd") 

        self.logger.info(f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")

        # notify android of image-rec results
        #self.android_queue.put(AndroidMessage("image-rec", results))

    def request_stitch(self):
        """Sends a stitch request to the image recognition API to stitch the different images together"""
        url = f"http://{self.valid_api}:{API_PORT}/stitch"
        response = requests.get(url)

        # If error, then log, and send error to Android
        if response.status_code != 200:
            # Notify android
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting stitch from the API."))
            self.logger.error(
                "Something went wrong when requesting stitch from the API.")
            return

        self.logger.info("Images stitched!")
        self.android_queue.put(AndroidMessage("info", "Images stitched!"))

    def clear_queues(self):
        """Clear both command and path queues"""
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        """Check whether image recognition and algorithm API server is up and running

        Returns:
            bool: True if running, False if not.
        """
        # Check image recognition API
        url = f"http://{self.valid_api}:{API_PORT}/status"
        try:
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                self.logger.debug("API is up!")
                return True
            return False
        # If error, then log, and return False
        except ConnectionError:
            self.logger.warning("API Connection Error")
            return False
        except requests.Timeout:
            self.logger.warning("API Timeout")
            return False
        except Exception as e:
            self.logger.warning(f"API Exception: {e}")
            return False
    
    def request_algo(self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues
        """
        self.logger.info("Requesting path from algo...")
        self.android_queue.put(AndroidMessage(
            "info", "Requesting path from algo..."))
        self.logger.info(f"data: {data}")
        body = {**data, "big_turn": "0", "robot_x": robot_x,
                "robot_y": robot_y, "robot_dir": robot_dir, "retrying": retrying}
        url = f"http://{self.valid_api}:{API_PORT}/path"
        response = requests.post(url, json=body)

        # Error encountered at the server, return early
        if response.status_code != 200:
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting path from Algo API."))
            self.logger.error(
                "Something went wrong when requesting path from Algo API.")
            return

        # Parse response
        result = json.loads(response.content)['data']
        commands = result['commands']
        path = result['path']
        print(response.content)

        # Log commands received
        self.logger.debug(f"Commands received from API: {commands}")

        # Put commands and paths into respective queues
        self.clear_queues()
        for c in commands:
            self.command_queue.put(c)
        for p in path[1:]:  # ignore first element as it is the starting position of the robot
            self.path_queue.put(p)

        self.android_queue.put(AndroidMessage(
            "info", "Commands and path received Algo API. Robot is ready to move."))
        self.logger.info(
            "Commands and path received Algo API. Robot is ready to move.")

if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
