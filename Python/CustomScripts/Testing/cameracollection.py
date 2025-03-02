import os
import cv2
import threading, time, json
import ALSLib.TCPClient, ALSLib.ALSClient
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg

########################
# In this demo we load a situation that has a vehicle and a camera, then
# we connect to the camera stream and display it in another window using OpenCV
# while also saving each frame to disk.
########################

# WORKING 2024-12-12

# Write your own message handler to decide how to react to collisions, triggers etc..
def myMessageHandler(rawMessage):
    str_msg = rawMessage.decode('utf-8')
    cmdList = str_msg.split(" ")
    if cmdList[0].startswith("EndCondition"):
        TestContext.client.request_destroy_situation()
        TestContext.lock.acquire()
        TestContext.testEnded = True
        print("setting TestEnded")
        TestContext.lock.release()


HOST = '127.0.0.1'

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), myMessageHandler)


if __name__ == "__main__":
    # Connect and load the scenario
    TestContext.client.connect()
    TestContext.client.request_load_scenario('AboatICT')

    # Pause briefly to ensure scenario is fully loaded
    time.sleep(1)

    # Retrieve the sensor list
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)

    # Identify the first camera (Sensors.[0]) and extract its port/ip
    for x in parsed_json['sensors']:
        if x['path'] == 'Sensors.[0]':
            sensor = x
            break

    camera_port = sensor['sensor_port']
    host = sensor['sensor_ip']
    print("host:", host, "port:", camera_port)

    # The camera port is defined inside the situation INI files (can be changed in Sensor Editor).
    print("Connecting sensor socket to", host, str(camera_port))
    client = ALSLib.TCPClient.TCPClient(host, camera_port, 5)  # Arg #3: timeout in seconds
    client.connect(5)

    # Ensure the directory for saving images exists
    save_directory = "SensorData/imgs"
    os.makedirs(save_directory, exist_ok=True)

    imageNum = 0
    while imageNum < 1500:
        data = client.read()
        index = 0
        imageNum += 1

        # Extract the image from the stream
        img, index, width, height = ALSFunc.ReadImage_Stream(data, index)

        # Display the image (optional; remove if you don't need real-time display)
        ALSImg.JustDisplay(img)

        # Save the image to SensorData/imgs with a unique filename
        save_path = os.path.join(save_directory, f"frame_{imageNum:04d}.png")
        cv2.imwrite(save_path, img)

    print("Finished saving images.")
