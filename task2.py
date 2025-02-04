#!/usr/bin/env python3
import json
import queue
import time
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_IP_START, API_PORT
import io
import subprocess
from datetime import datetime
from PIL import Image


class PiAction:
    def __init__(self, cat, value):
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value


class RaspberryPi:
    def __init__(self):
        # Initialize logger and communication objects with Android and STM
        self.logger = prepare_logger()
        self.android_link = AndroidLink()
        self.stm_link = STMLink()
        

        # For sharing information between child processes
        self.manager = Manager()

        # Set robot mode to be 1 (Path mode)
        self.robot_mode = self.manager.Value('i', 1)

        # Events
        self.android_dropped = self.manager.Event()  # Set when the android link drops
        # commands will be retrieved from commands queue when this event is set
        self.unpause = self.manager.Event()

        # Movement Lock
        self.movement_lock = self.manager.Lock()

        # Queues
        self.android_queue = self.manager.Queue() # Messages to send to Android
        self.rpi_action_queue = self.manager.Queue() # Messages that need to be processed by RPi
        self.command_queue = self.manager.Queue() # Messages that need to be processed by STM32, as well as snap commands

        # Define empty processes
        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None

        self.ack_count = 0
        self.near_flag = self.manager.Lock()
        self.small_direction = None
        self.big_direction = None
        self.retry_flag = False

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            # Establish Bluetooth connection with Android
            self.android_link.connect()
            self.android_queue.put(AndroidMessage('info', 'You are connected to the RPi!'))

            # Establish connection with STM32
            self.stm_link.connect()

            # Check Image Recognition and Algorithm API status
            for endpoint in API_IP_START:
                self.valid_api = API_IP + str(endpoint)  
                if self.check_api():
                    self.logger.debug("API successfully set up at", self.valid_api)
                    break 
            
            #self.small_direction = self.snap_and_rec("Small")
            #self.logger.info(f"PREINFER small direction is: {self.small_direction}")

            # Define child processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # Start child processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            self.proc_android_sender.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            self.logger.info("Child Processes started")

            ### Start up complete ###

            # Send success message to Android
            self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))
            
            
            
            # Handover control to the Reconnect Handler to watch over Android connection
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")

    def reconnect_android(self):
        """Handles the reconnection to Android in the event of a lost connection."""
        self.logger.info("Reconnection handler is watching...")

        while True:
            # Wait for android connection to drop
            self.android_dropped.wait()

            self.logger.error("Android link is down!")

            # Kill child processes
            self.logger.debug("Killing android child processes")
            self.proc_android_sender.kill()
            self.proc_recv_android.kill()

            # Wait for the child processes to finish
            self.proc_android_sender.join()
            self.proc_recv_android.join()
            assert self.proc_android_sender.is_alive() is False
            assert self.proc_recv_android.is_alive() is False
            self.logger.debug("Android child processes killed")

            # Clean up old sockets
            self.android_link.disconnect()

            # Reconnect
            self.android_link.connect()

            # Recreate Android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # Start previously killed processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage("info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))

            self.android_dropped.clear()
            
        
    def recv_android(self) -> None:
        """
        [Child Process] Processes the messages received from Android
        """
       
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            # If an error occurred in recv()
            if msg_str is None:
                continue

            message: dict = json.loads(msg_str)

            ## Command: Start Moving ##
            if message['cat'] == "control":
                if message['value'] == "start":
        
                    if not self.check_api():
                        self.logger.error("API is down! Start command aborted.")

                    self.clear_queues()
                    # Go forward to the small block
                    self.command_queue.put("GO00") # ack_count = 1
                    self.command_queue.put("RW01") # stm will send back SNAP1
                    #self.near_flag.acquire() 
                    
                    # # Small object direction detection
                    # self.small_direction = self.snap_and_rec("small")
                    # self.logger.info(f"HERE small direction is: {self.small_direction}")
                    # if self.small_direction == "Left Arrow": 
                    #     self.command_queue.put("OB01") # ack_count = 3
                    #     self.command_queue.put("UL00") # ack_count = 5
                    # elif self.small_direction == "Right Arrow":
                    #     self.command_queue.put("OB01") # ack_count = 3
                    #     self.command_queue.put("UR00") # ack_count = 5

                    # elif self.small_direction == None or self.small_direction == 'None':
                    #     self.logger.info("Acquiring near_flag log")
                    #     self.near_flag.acquire()             
                        
                    #     self.command_queue.put("OB01") # ack_count = 3
                        
                    self.logger.info("Start command received, starting robot on task 2!")
                    self.android_queue.put(AndroidMessage('status', 'running'))

                    # Commencing path following | Main trigger to start movement #
                    self.unpause.set()
                    
    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:

            message: str = self.stm_link.recv()
            # Acknowledgement from STM32
            if message.startswith("ACK"):
                self.ack_count += 1
                # Release movement lock
                try:
                    self.movement_lock.release()
                except Exception:
                    self.logger.warning("Tried to release a released lock!")
                self.logger.info(f"ACK from STM32 received, ACK count now:{self.ack_count}")   

            # Robot in position to do image rec
            elif message.startswith("SNAP"):
                self.logger.info("Sending API requests to image server")
                _, obstacle_id = message.split('_')
                if message.endswith("1"): # Reached first obstacle
                    self.logger.info("Robot reached first obstacle!")
                    self.small_direction = self.snap_and_rec("small")
                    
                    if self.small_direction == "Left": 
                        # When we retry, we move back to original position after image rec
                        #if self.retry_flag is True:
                            #self.command_queue.put("FW10")
                            #self.retry_flag = False
                        self.command_queue.put("BW07") # ack_count = 2
                        self.command_queue.put("HL00") # ack_count = 3
                        self.command_queue.put("FW15") # ack_count = 4
                        self.command_queue.put("RR00") # ack_count = 4
                        #self.command_queue.put("FW17") # ack_count = 4 For outdoors
                        self.command_queue.put("FW15") # ack_count = 4 For indoors
                        self.command_queue.put("HL00") # ack_count = 5
                        self.command_queue.put("GF00") # ack_count = 6
                        self.command_queue.put("PW02") # ack_count = 7
                        self.logger.info("Commands pushed to queue!")
                    elif self.small_direction == "Right":
                        # When we retry, we move back to original position after image rec
                        #if self.retry_flag is True:
                            #self.command_queue.put("FW10")
                            #self.retry_flag = False
                        self.command_queue.put("HR00") # ack_count = 2
                        self.command_queue.put("FW12") # ack_count = 3
                        
                        self.command_queue.put("LL00") # ack_count = 4
                        self.command_queue.put("FW10") # ack_count = 4
                        self.command_queue.put("HR00") # ack_count = 5
                        self.command_queue.put("GF00") # ack_count = 6
                        self.command_queue.put("PW02") # ack_count = 7
                        self.logger.info("Commands pushed to queue!")
                    # Retry logic: If image rec fail, robot will reverse then send back SNAP1
                    #else: # We dont care about non-left/right arrow
                        #self.logger.debug("Error detecting. Retry again.")
                        #self.retry_flag = True
                        #command = "FA0" + obstacle_id
                        #self.command_queue.put(command)
                if message.endswith("2"): # Reached second obstacle
                    self.logger.info("Robot reached second obstacle!")
                    self.big_direction = self.snap_and_rec("big")
                    #self.logger.debug("Big Direction:", self.big_direction)
                    if self.big_direction == "Left": 
                        # When we retry, we move back to original position after image rec
                        #if self.retry_flag is True:
                            #self.command_queue.put("FW10")
                            #self.retry_flag = False
                        self.command_queue.put("LL00") # ack_count = 8
                        self.command_queue.put("UR00") # ack_count = 9
                        self.command_queue.put("FW55") # ack_count = 10
                        self.command_queue.put("RR00") # ack_count = 11
                        
                        # Car will be at the edge facing carpark
                        # Next set of commands for car to park at the carpark
                        self.command_queue.put("EN00") # ack_count = 12
                        self.command_queue.put("RR00") # ack_count = 13
                        #self.command_queue.put("FW10")
                        self.command_queue.put("LL00") # ack_count = 14
                        self.command_queue.put("GG00") # ack_count = 15
                        self.command_queue.put("FN")
                    
                    elif self.big_direction == "Right":
                        # When we retry, we move back to original position after image rec
                        #if self.retry_flag is True:
                            #self.command_queue.put("FW10")
                            #self.retry_flag = False
                        self.command_queue.put("RR00") # ack_count = 8
                        self.command_queue.put("UL00") # ack_count = 9
                        self.command_queue.put("FW60") # ack_count = 10 Indoors
                        self.command_queue.put("LL00") # ack_count = 11
                        
                        # Car will be at the edge facing carpark
                        # Next set of commands for car to park at the carpark
                        self.command_queue.put("EN00") # ack_count = 12
                        self.command_queue.put("LL00") # ack_count = 13
                        self.command_queue.put("BW08") # ack_count = 13
                        self.command_queue.put("RR00") # ack_count = 14
                        self.command_queue.put("GG00") # ack_count = 15
                        self.command_queue.put("FN") 
                    # Retry logic: If image rec fail, robot will reverse then send back SNAP1
                    #else: # We dont care about non-left/right arrow
                        #self.retry_flag = True
                        #command = "FA0" + obstacle_id
                        #self.command_queue.put(command)
                try:
                    self.movement_lock.release()
                    self.logger.info("Movement lock released")
                except Exception:
                    self.logger.warning("Tried to release a released lock!")
            else:
                self.logger.warning(
                    f"Ignored unknown message from STM: {message}")

    def android_sender(self) -> None:
        while True:
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
        
        while True:
            command: str = self.command_queue.get()
            self.unpause.wait()
            self.logger.info("Unpuase has been set!")
            self.movement_lock.acquire()
            stm32_prefixes = ("GO", "RW", "HL", "FW", "RR", "HR", "LL", "GG", "FA", "UL", "UR", "EN", "GF", "BW", "PW")
            if command.startswith(stm32_prefixes):
                self.stm_link.send(command)
            elif command == "FN":
                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                self.android_queue.put(AndroidMessage("info", "Commands queue finished."))
                self.android_queue.put(AndroidMessage("status", "finished"))
                #self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
                self.request_stitch()
            else:
                raise Exception(f"Unknown command: {command}")

    def rpi_action(self):
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(f"PiAction retrieved from queue: {action.cat} {action.value}")
            if action.cat == "snap": self.snap_and_rec(obstacle_id=action.value)
            elif action.cat == "stitch": self.request_stitch()

    def snap_and_rec(self, obstacle_id: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id: the current obstacle ID
        """
        
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        self.android_queue.put(AndroidMessage("info", f"Capturing image for obstacle id: {obstacle_id}"))
        signal = "C"
        url = f"http://{API_IP}:{API_PORT}/image"
        filename = f"{int(time.time())}_{obstacle_id}.jpg"
        
        
        config_file = "/home/pi/rpi/PiLCConfig530_outdoor.txt"

        extns        = ['jpg','png','bmp','rgb','yuv420','raw']
        shutters     = [-2000,-1600,-1250,-1000,-800,-640,-500,-400,-320,-288,-250,-240,-200,-160,-144,-125,-120,-100,-96,-80,-60,-50,-48,-40,-30,-25,-20,-15,-13,-10,-8,-6,-5,-4,-3,0.4,0.5,0.6,0.8,1,1.1,1.2,2,3,4,5,6,7,8,9,10,11,15,20,25,30,40,50,60,75,100,112,120,150,200,220,230,239,435]
        meters       = ['centre','spot','average']
        awbs         = ['off','auto','incandescent','tungsten','fluorescent','indoor','daylight','cloudy']
        denoises     = ['off','cdn_off','cdn_fast','cdn_hq']

        config = []
        with open(config_file, "r") as file:
            line = file.readline()
            while line:
                config.append(line.strip())
                line = file.readline()
            config = list(map(int,config))
        mode        = config[0]
        speed       = config[1]
        gain        = config[2]
        brightness  = config[3]
        contrast    = config[4]
        red         = config[6]
        blue        = config[7]
        ev          = config[8]
        extn        = config[15]
        saturation  = config[19]
        meter       = config[20]
        awb         = config[21]
        sharpness   = config[22]
        denoise     = config[23]
        quality     = config[24]
        
        #retry_count = 0
        
        #while True:
        
            #retry_count += 1
        
        shutter = shutters[speed]
        if shutter < 0:
            shutter = abs(1/shutter)
        sspeed = int(shutter * 1000000)
        if (shutter * 1000000) - int(shutter * 1000000) > 0.5:
            sspeed +=1
                
        """rpistr = "libcamera-still -e " + extns[extn] + " -n -t 100 -o " + filename
        rpistr += " --brightness " + str(brightness/100) + " --contrast " + str(contrast/100)
        rpistr += " --shutter " + str(sspeed)
        if ev != 0:
            rpistr += " --ev " + str(ev)
        if sspeed > 1000000 and mode == 0:
            rpistr += " --gain " + str(gain) + " --immediate "
        else:    
            rpistr += " --gain " + str(gain)
            if awb == 0:
                rpistr += " --awbgains " + str(red/10) + "," + str(blue/10)
            else:
                rpistr += " --awb " + awbs[awb]
        rpistr += " --metering " + meters[meter]
        rpistr += " --saturation " + str(saturation/10)
        rpistr += " --sharpness " + str(sharpness/10)
        rpistr += " --quality " + str(quality)
        rpistr += " --denoise "    + denoises[denoise]
        rpistr += " --metadata - --metadata-format txt >> PiLibtext.txt"

        os.system(rpistr)"""
        
        start_time = time.perf_counter()
        rpistr = [
                    "libcamera-jpeg" ,
                    "-e", extns[extn],
                    "-n",
                    "-t", "500",
                    "-o", "-",
                    "--brightness", str(brightness/100),
                    "--contrast", str(contrast/100),
                    "--shutter", str(sspeed),
                    "--gain", str(gain),
                    "--metering",  meters[meter],
                    "--saturation", str(saturation/10),
                    "--sharpness", str(sharpness/10),
                    "--quality", str(quality),
                    "--denoise", denoises[denoise]
                    
        ]
            
        if ev != 0:
            rpistr += " --ev " + str(ev)
        if sspeed > 1000000 and mode == 0:
            rpistr += " --gain " + str(gain) + " --immediate "
        else:    
            rpistr += " --gain " + str(gain)
            if awb == 0:
                rpistr += " --awbgains " + str(red/10) + "," + str(blue/10)
            else:
                rpistr += " --awb " + awbs[awb]
            
        #Execute the command
        process = subprocess.Popen(rpistr, stdout=subprocess.PIPE)
        image_data, _ = process.communicate()
        if process.returncode != 0:
            self.logger.error(f"Libcamera-still failed.")
            
            
        #img = Image.open(io.BytesIO(image_data))
        #compressed_image_io = io.BytesIO()
        #img.save(compressed_image_io, format="JPEG", quality=85)
        
        #compressed_image_io.seek(0)
            
            
        self.logger.debug("Requesting from image API")
        # Proceed with sending the image to the API
        url = f"http://{self.valid_api}:{API_PORT}/image"
        #img_file = {'files': (file_path, open(file_path, 'rb'), 'image/jpeg')}
        img_file = {"files": (f'/home/pi/rpi/{datetime.now().strftime("%Y%m%d_%H%M%S")}_{obstacle_id}.jpg', io.BytesIO(image_data), 'image/jpeg')}
        
        #img_file = {"files": (f'/home/pi/rpi/{datetime.now().strftime("%Y%m%d_%H%M%S")}_{obstacle_id}.jpg', compressed_image_io, 'image/jpeg')}
        data = {'obstacle_id': str(obstacle_id), 'signal': signal}
        end_time = time.perf_counter()
        self.logger.info(f"Total time taken: {end_time - start_time:.2f} seconds")
        try:
            response = requests.post(url, files=img_file, data=data)
        except Exception as e:
            self.logger.error(f"Error with image API: {e}")
            return "Right"
        img_file['files'][1].close()

        if response.status_code != 200:
            self.logger.error("Something went wrong when requesting path from image-rec API. Please try again.")
            return "Right"

        results = json.loads(response.content)

            # Higher brightness retry
            
        """if results['image_id'] != 'NA' or retry_count > 6:
            break
        elif retry_count <= 2:
            self.logger.info(f"Image recognition results: {results}")
            self.logger.info("Recapturing with same shutter speed...")
        elif retry_count <= 4:
            self.logger.info(f"Image recognition results: {results}")
            self.logger.info("Recapturing with lower shutter speed...")
            speed -= 1
        elif retry_count == 5:
            self.logger.info(f"Image recognition results: {results}")
            self.logger.info("Recapturing with lower shutter speed...")
            speed += 3"""
        
        self.logger.info(f"Image recognition results: {results} ")
        return results["class_name"] if results["class_name"] != "NA" else "Right"

    def request_stitch(self):
        url = f"http://{self.valid_api}:{API_PORT}/stitch"
        response = requests.get(url)
        if response.status_code != 200:
            self.logger.error("Something went wrong when requesting stitch from the API.")
            return
        self.logger.info("Images stitched!")

    def clear_queues(self):
        while not self.command_queue.empty():
            self.command_queue.get()


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

if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
