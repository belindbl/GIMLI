import threading
import time, json, struct, math, numpy as np, os, cv2
import ALSLib.TCPClient, ALSLib.ALSClient
import ALSLib.ALSHelperLidarLibrary as ALSLidar
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg

# Omitted file writing
# Seems quicker than merge_test5.py, and the frames from the different modalities are synchronised

HOST = '127.0.0.1'
ABOAT = "EgoVehicleSettings\\Aboat.SensorProfile.ini"
# LIDAR configuration (if needed)
LIDAR = "ConfigFiles\\Sensors\\LidarSettings\\LIDARTest.xml"

# Limits (we expect 10 Hz LiDAR and want the same number of camera frames)
MAX_LIDAR_PCDS = 300
MAX_IMAGES = 300

# Global stop event
stop_event = threading.Event()

# Two events for synchronizing LiDAR and camera threads:
# lidar_event is set when a LiDAR frame is ready
# camera_event is set when the camera has captured its corresponding frame.
lidar_event = threading.Event()
camera_event = threading.Event()


def myMessageHandler(rawMessage):
    msg = rawMessage.decode('utf-8')
    if msg.startswith("EndCondition"):
        TestContext.client.request_destroy_situation()
        stop_event.set()


class TestContext:
    lock = threading.Lock()
    client = ALSLib.ALSClient.Client((HOST, 9000), myMessageHandler)


def lidar_thread():
    client = ALSLib.TCPClient.TCPClient(HOST, 8881, 5)  # Assuming LiDAR is on port 8881
    client.connect(5)
    lidar_count = 0
    sizeofFloat = 4
    header_format = '<fffffffffff'
    header_size = struct.calcsize(header_format)

    while lidar_count < MAX_LIDAR_PCDS and not stop_event.is_set():
        try:
            data = client.read()
        except Exception as e:
            print("LiDAR read exception:", e)
            break

        if len(data) < header_size:
            continue  # skip if header data is incomplete

        # Parse LiDAR header (but no file writing or long processing)
        header = struct.unpack(header_format, data[:header_size])
        posX, posY, posZ, quatW, quatX, quatY, quatZ, numPoints, timeStart, timeEnd, numberOfBeams = header
        print(f"LiDAR frame {lidar_count} captured at {time.time():.3f}")
        lidar_count += 1

        # Signal that a LiDAR frame is available
        lidar_event.set()
        # Wait (with a timeout) until the camera thread captures its corresponding image
        if not camera_event.wait(timeout=5):
            print("Camera did not respond in time.")
        camera_event.clear()

    stop_event.set()


def camera_thread():
    # Get camera sensor info from the simulation
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)
    sensor = None
    for x in parsed_json['sensors']:
        if x['path'] == 'Sensors.[0]':
            sensor = x
            break
    if sensor is None:
        print("Camera sensor not found.")
        stop_event.set()
        return

    camera_port = sensor['sensor_port']
    host = sensor['sensor_ip']
    print("Connecting camera socket to", host, camera_port)
    client = ALSLib.TCPClient.TCPClient(host, camera_port, 5)
    client.connect(5)

    image_count = 0
    while image_count < MAX_IMAGES and not stop_event.is_set():
        # Wait for LiDAR signal (with a timeout)
        if not lidar_event.wait(timeout=5):
            continue  # no LiDAR frame signal, try again
        # Once LiDAR data is ready, capture one image
        data = client.read()
        index = 0
        img, index, width, height = ALSFunc.ReadImage_Stream(data, index)
        ALSImg.JustDisplay(img)
        print(f"Camera frame {image_count} displayed at {time.time():.3f}")
        image_count += 1

        # Clear the LiDAR event and signal back to the LiDAR thread
        lidar_event.clear()
        camera_event.set()

    stop_event.set()


if __name__ == "__main__":
    TestContext.client.connect()
    TestContext.client.request_load_scenario("Aboat")
    # We remove any file-writing operations; only display is performed.

    # (Optionally, if ALSLidar.create_sensor_data_folders() is not needed, omit it)
    ALSLidar.create_sensor_data_folders()

    # Start the threads
    lidar_t = threading.Thread(target=lidar_thread)
    camera_t = threading.Thread(target=camera_thread)
    lidar_t.start()
    camera_t.start()

    # Wait for both threads to finish
    lidar_t.join()
    camera_t.join()
    print("All threads finished.")
