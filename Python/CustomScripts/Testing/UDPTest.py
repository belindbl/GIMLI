import threading
import time
import json
import struct
import numpy as np
import queue
import open3d as o3d
import ALSLib.TCPClient
import ALSLib.ALSClient
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg
import ALSLib.UDPClient

HOST = '127.0.0.1'
stop_threads_event = threading.Event()

# Queue to buffer LiDAR data for async processing
lidar_queue = queue.Queue(maxsize=10)

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

def camera_thread():
    """Thread for receiving and displaying camera frames over UDP."""
    camera_client = ALSLib.UDPClient.UDPClient(camera_host, camera_port)  
    camera_client.connect()  # Call connect() to initialize the socket

    imageNum = 0
    while not stop_threads_event.is_set():
        try:
            cam_data = camera_client.readSimple()  # Read latest frame
            if cam_data:
                index = 0
                imageNum += 1
                img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                ALSImg.JustDisplay(img)
        except Exception as e:
            print(f"Camera UDP read error: {e}")

        time.sleep(0.02)



def lidar_thread():
    """Thread for receiving LiDAR data over UDP."""
    lidar_client = ALSLib.UDPClient.UDPClient(HOST, 8881)  
    lidar_client.connect()  # Call connect() to initialize the socket

    while not stop_threads_event.is_set():
        try:
            lidar_data = lidar_client.readSimple()  
            if lidar_data:
                if not lidar_queue.full():
                    lidar_queue.put(lidar_data, block=False)
        except Exception as e:
            print(f"LiDAR UDP read error: {e}")

        time.sleep(0.1)


def lidar_processing_thread():
    """Thread for processing and displaying LiDAR point clouds in Open3D."""
    
    # Open3D visualization setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualization")
    point_cloud = o3d.geometry.PointCloud()

    # Get the view control **only once**
    ctr = vis.get_view_control()

    # Apply initial viewpoint settings
    ctr.set_lookat([0, 0, 300])
    ctr.set_zoom(0.02)
    ctr.set_front([-1, 0, 1])  # Set front direction // Same as the normal visualiser
    ctr.set_up([1, 0, 1])     # Set upward direction

    point_cloud_added = False  # Track whether the point cloud has been added

    while not stop_threads_event.is_set():
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

                # Extract points and INVERT the Y-coordinate to match coordinate systems between Open3d and UE4
                points = np.stack([point_array[:, 0], -point_array[:, 1], point_array[:, 2]], axis=1)

                # Assign points to the Open3D point cloud
                point_cloud.points = o3d.utility.Vector3dVector(points)

                # Fix: Only add the point cloud once
                if not point_cloud_added:
                    vis.add_geometry(point_cloud)
                    point_cloud_added = True
                else:
                    vis.update_geometry(point_cloud)

                # Reapply viewpoint settings to persist
                ctr.set_lookat([0, 0, 300])  
                ctr.set_zoom(0.02)
                ctr.set_front([-1, 0, 1])  
                ctr.set_up([1, 0, 1])     

                vis.poll_events()
                vis.update_renderer()

        except queue.Empty:
            pass  # No LiDAR data to process, continue loop

    vis.destroy_window()


if __name__ == "__main__":
    stop_event = threading.Event()

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

    print("Connecting to Camera:", HOST, camera_port)
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
