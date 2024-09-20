import requests
import cv2
from picamera2 import Picamera2

def capture_image():
    """Function to capture an image using Picamera2 and return it as a numpy array."""
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)
    picam2.start()

    # Capture an image as a numpy array
    frame = picam2.capture_array()

    # Stop the camera
    picam2.stop()
    
    file_path = 'test.jpg'
    cv2.imwrite(file_path, frame)
    return file_path

def send_image(file_path, obstacle_id, url):
    """Function to send an image and obstacle ID to a server via HTTP POST."""
    img_file = {'file': (file_path, open(file_path, 'rb'), 'image/jpeg')}

    # Data payload with the obstacle_id
    data = {'obstacle_id': obstacle_id}

    # Sending post request to the server with the image and data
    response = requests.post(url, data=data, files=img_file)

    img_file['file'][1].close()
    
    # Return server response
    return response

# Server endpoint
url = 'http://192.168.48.49:5000/image'

# Dummy obstacle ID for testing
obstacle_id = '123'

# Capture image using Picamera2
captured_image = capture_image()

# Send the captured image to the server
server_response = send_image(captured_image, obstacle_id, url)

# Print the response from the server
print(server_response.content)
