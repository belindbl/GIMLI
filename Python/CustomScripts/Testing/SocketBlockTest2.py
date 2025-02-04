import threading
import time
import json
import struct
import numpy as np
import queue
import ALSLib.TCPClient
import ALSLib.ALSClient
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg
from ALSLib.ALSHelperFunctionLibrary import get_sensordata_path
import math

########################
# Camera: Near Real-Time
# LiDAR: Buffered, Processed Separately
# Still slight delay, but works better than previous attempts
########################

HOST = '127.0.0.1'
MAX_LIDAR_PCDS = 300
lidar_pcd_count = 0
stop_threads_event = threading.Event()

# Queue to buffer LiDAR data for async processing
lidar_queue = queue.Queue(maxsize=10)

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

def SerializeToPCLFileContent(numPoints, posX, posY, posZ, quatW, quatX, quatY, quatZ, point_array):
    pclFileContent = '# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\n'\
    'SIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT %f %f %f %f %f %f %f\n'\
    'POINTS %d\nDATA ascii\n' % (int(numPoints), posX, posY, posZ, quatW, quatX, quatY, quatZ, int(numPoints))
    
    for p in point_array:
        intensity = 1000
        if not math.isinf(p[3]) and p[3] != 0:
            intensity = int(p[3])
        pclFileContent += '%.5f %.5f %.5f %d\n' % (p[0], -p[1], p[2], intensity) 
    return pclFileContent

def camera_thread():
    """Thread for reading and displaying camera frames (Real-time priority)."""
    camera_client = ALSLib.TCPClient.TCPClient(camera_host, camera_port, 2)  # Shorter timeout
    camera_client.connect(2)
    imageNum = 0

    while imageNum < 1000 and not stop_threads_event.is_set():
        try:
            cam_data = camera_client.read()
            if cam_data:
                index = 0
                imageNum += 1
                img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                ALSImg.JustDisplay(img)
        except Exception as e:
            print(f"Camera read error: {e}")

        time.sleep(0.05)  # Lower delay ensures higher frame rate

def lidar_thread():
    """Thread for reading LiDAR data and adding it to the processing queue."""
    lidar_client = ALSLib.TCPClient.TCPClient(HOST, 8881, 2)
    lidar_client.connect(2)

    while not stop_threads_event.is_set():
        try:
            lidar_data = lidar_client.read()
            if lidar_data:
                if not lidar_queue.full():
                    lidar_queue.put(lidar_data, block=False)
        except Exception as e:
            print(f"LiDAR read error: {e}")

        time.sleep(0.1)  # Slightly slower, since LiDAR data rate is lower

def lidar_processing_thread():
    """Thread for processing LiDAR point clouds (Buffered)."""
    global lidar_pcd_count

    while lidar_pcd_count < MAX_LIDAR_PCDS and not stop_threads_event.is_set():
        try:
            lidar_data = lidar_queue.get(timeout=1)  # Wait for new data
            if lidar_data:
                sizeofFloat = 4
                index = 11
                posX, posY, posZ, quatW, quatX, quatY, quatZ, numPoints, timeStart, timeEnd, numberOfBeams = struct.unpack(
                    '<fffffffffff', lidar_data[:index * sizeofFloat])

                pointCloudData = lidar_data[index * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.dtype("float32"))
                point_array = np.reshape(point_array, (-1, 4))

                pclFileContent = SerializeToPCLFileContent(numPoints, posX, posY, posZ, quatW, quatX, quatY, quatZ, point_array)
                filename = get_sensordata_path(f"/pcl/pcl{lidar_pcd_count}.pcd")

                try:
                    with open(filename, mode='w') as fileObject:
                        fileObject.write(pclFileContent)
                    print(f"Point cloud {lidar_pcd_count} saved to {filename}")
                    lidar_pcd_count += 1
                except Exception as e:
                    print(f"Failed to save point cloud: {e}")

        except queue.Empty:
            pass  # No LiDAR data to process, continue loop

if __name__ == "__main__":
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Aboat')

    # Get sensor list
    time.sleep(1)
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)

    # Find camera sensor
    for x in parsed_json['sensors']:
        if x['path'] == 'Sensors.[0]':
            camera_sensor = x
            break
    camera_port = camera_sensor['sensor_port']
    camera_host = camera_sensor['sensor_ip']

    print("Connecting to Camera:", camera_host, camera_port)
    print("Connecting to LiDAR:", HOST, 8881)

    # Start threads
    camera_thread_instance = threading.Thread(target=camera_thread, daemon=True)
    lidar_thread_instance = threading.Thread(target=lidar_thread, daemon=True)
    lidar_processing_instance = threading.Thread(target=lidar_processing_thread, daemon=True)

    camera_thread_instance.start()
    lidar_thread_instance.start()
    lidar_processing_instance.start()

    try:
        while camera_thread_instance.is_alive() or lidar_thread_instance.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads_event.set()
        camera_thread_instance.join()
        lidar_thread_instance.join()
        lidar_processing_instance.join()
