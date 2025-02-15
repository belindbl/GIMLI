import threading
import time
import json
import struct
import ALSLib.TCPClient
import numpy as np
import open3d as o3d
import ALSLib.ALSClient
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg
import ALSLib.UDPClient

HOST = '127.0.0.1'
stop_threads_event = threading.Event()

# Shared data structure for storing the latest sensor data.
# Protected by a lock to avoid race conditions.
sensor_data = {
    'camera': None,
    'lidar': None,
}
data_lock = threading.Lock()

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

def camera_data_reader():
    """
    Reader thread for camera data.
    Connects to the camera sensor via UDP (or TCP as required)
    and continuously updates the shared sensor_data dictionary.
    """
    camera_client = ALSLib.TCPClient.TCPClient(camera_host, 8880, 5)
    camera_client.connect(5)
    
    while not stop_threads_event.is_set():
        try:
            cam_data = camera_client.read()
            if cam_data:
                with data_lock:
                    sensor_data['camera'] = cam_data
        except Exception as e:
            print(f"Camera read error: {e}")
        time.sleep(0.02)  # Adjust for desired frame rate

def lidar_data_reader():
    """
    Reader thread for LiDAR data.
    Connects to the LiDAR sensor via UDP (or TCP as required)
    and continuously updates the shared sensor_data dictionary.
    """
    lidar_client = ALSLib.TCPClient.TCPClient(HOST, 8881, 5)
    lidar_client.connect(5)
    
    while not stop_threads_event.is_set():
        try:
            lidar_data = lidar_client.read()
            if lidar_data:
                with data_lock:
                    sensor_data['lidar'] = lidar_data
        except Exception as e:
            print(f"LiDAR read error: {e}")
        time.sleep(0.1)  # LiDAR rate is typically lower

def camera_processing_thread():
    """
    Processing thread for camera data.
    Periodically retrieves the latest camera data from the shared variable,
    processes it and displays the image.
    """
    imageNum = 0
    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
        if cam_data:
            try:
                index = 0
                imageNum += 1
                img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                ALSImg.JustDisplay(img)
            except Exception as e:
                print(f"Camera processing error: {e}")
        time.sleep(0.02)

def lidar_processing_thread():
    """
    Processing thread for LiDAR data.
    Periodically retrieves the latest LiDAR data from the shared variable,
    processes it and updates the Open3D visualisation.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualisation")
    point_cloud = o3d.geometry.PointCloud()
    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 300])
    ctr.set_zoom(0.02)
    ctr.set_front([-1, 0, 1])
    ctr.set_up([1, 0, 1])
    point_cloud_added = False

    while not stop_threads_event.is_set():
        with data_lock:
            lidar_data = sensor_data.get('lidar')
        if lidar_data:
            try:
                sizeofFloat = 4
                header_count = 11  # Number of header floats
                index = header_count
                # Unpack header: positions, orientation, point count, time stamps etc.
                unpacked = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
                # The remainder is the point cloud data.
                pointCloudData = lidar_data[header_count * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                point_array = np.reshape(point_array, (-1, 4))
                # Invert the Y-coordinate to align coordinate systems.
                points = np.stack([point_array[:, 0], -point_array[:, 1], point_array[:, 2]], axis=1)
                point_cloud.points = o3d.utility.Vector3dVector(points)
                
                if not point_cloud_added:
                    vis.add_geometry(point_cloud)
                    point_cloud_added = True
                else:
                    vis.update_geometry(point_cloud)

                # Reapply viewpoint settings.
                ctr.set_lookat([0, 0, 300])
                ctr.set_zoom(0.02)
                ctr.set_front([-1, 0, 1])
                ctr.set_up([1, 0, 1])
                vis.poll_events()
                vis.update_renderer()
            except Exception as e:
                print(f"LiDAR processing error: {e}")
        time.sleep(0.1)
    vis.destroy_window()

if __name__ == "__main__":
    # Initialise the simulation environment.
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Aboat')
    time.sleep(1)  # Allow some time for the simulation to initialise

    # Retrieve sensor list via TCP.
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)

    # Find the camera sensor in the list.
    for x in parsed_json['sensors']:
        if x['path'] == 'Sensors.[0]':
            camera_sensor = x
            break
    camera_port = camera_sensor['sensor_port']
    camera_host = camera_sensor['sensor_ip']

    print("Connecting to Camera:", camera_host, camera_port)
    print("Connecting to LiDAR:", HOST, 8881)

    # Start reader and processor threads.
    camera_reader_thread = threading.Thread(target=camera_data_reader, daemon=True)
    lidar_reader_thread = threading.Thread(target=lidar_data_reader, daemon=True)
    camera_processor_thread = threading.Thread(target=camera_processing_thread, daemon=True)
    lidar_processor_thread = threading.Thread(target=lidar_processing_thread, daemon=True)

    camera_reader_thread.start()
    lidar_reader_thread.start()
    camera_processor_thread.start()
    lidar_processor_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads_event.set()
        camera_reader_thread.join()
        lidar_reader_thread.join()
        camera_processor_thread.join()
        lidar_processor_thread.join()
