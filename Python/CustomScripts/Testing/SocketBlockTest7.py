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
import torch
import cv2
import os
import queue

HOST = '127.0.0.1'
stop_threads_event = threading.Event()

# Shared data structure for storing the latest sensor data.
# Protected by a lock to avoid race conditions.
sensor_data = {
    'camera': None,
    'lidar': None,
}
data_lock = threading.Lock()

# Create a queue for frames (only hold the latest frame)
frame_queue = queue.Queue(maxsize=1)

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

# YOLO model initialisation.
# Specify the desired weight file path.
ALSweights = "E:/Team1Docs/yolov5/weights_ailivesim/best.pt"
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

# Update process_frame to accept the model as a parameter.
def process_frame(image, model):
    if len(image.shape) == 2:  # Grayscale
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:  # RGBA
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    results = model(image)
    processed_image = results.render()[0]
    processed_image = cv2.cvtColor(processed_image, cv2.COLOR_RGB2RGBA)
    return processed_image

def yolo_inference_thread():
    """Thread dedicated to running YOLO inference on frames from the queue."""
    print("Loading YOLOv5 model in YOLO thread...", flush=True)
    try:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=MixedWeights,
                                force_reload=False, source='github')
        # Force initialisation by running a dummy inference
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = model(dummy_image)
        model.conf = 0.4
        model.iou = 0.5

        # Move the model to GPU if available.
        if torch.cuda.is_available():
            model = model.to('cuda')
            print("YOLOv5 model loaded and moved to GPU.", flush=True)
        else:
            print("CUDA not available; YOLOv5 model running on CPU.", flush=True)
    except Exception as e:
        print("Error loading YOLOv5 model in YOLO thread:", e, flush=True)
        return

    while not stop_threads_event.is_set():
        try:
            # Wait for a frame, with a short timeout.
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        # Process the frame with YOLO
        processed_img = process_frame(frame, model)
        ALSImg.JustDisplay(processed_img)



def camera_data_reader():
    """
    Reader thread for camera data.
    Connects to the camera sensor via TCP and continuously updates the shared sensor_data dictionary.
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
        time.sleep(0.02)

def lidar_data_reader():
    """
    Reader thread for LiDAR data.
    Connects to the LiDAR sensor via TCP and continuously updates the shared sensor_data dictionary.
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
        time.sleep(0.1)

def camera_processing_thread():
    """Thread to extract frames from sensor_data and push to the inference queue."""
    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
        if cam_data:
            try:
                index = 0
                img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                # Replace any stale frame in the queue
                if frame_queue.full():
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                frame_queue.put(img)
            except Exception as e:
                print(f"Camera processing error: {e}", flush=True)
        time.sleep(0.02)

def lidar_processing_thread():
    """
    Processing thread for LiDAR data.
    Retrieves the latest LiDAR data, processes it and updates the Open3D visualisation.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualisation")

    # Set the background color to light gray and the point size to 2.
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([0.8, 0.8, 0.8])  # Light gray
    render_option.point_size = 2

    point_cloud = o3d.geometry.PointCloud()
    # Set the point cloud colour to dark blue.
    point_cloud.paint_uniform_color([0.0, 0.0, 0.5])  # Dark blue

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
                header_count = 11  # Number of header floats.
                index = header_count
                # Unpack header information.
                unpacked = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
                pointCloudData = lidar_data[header_count * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                point_array = np.reshape(point_array, (-1, 4))
                # Adjust coordinate system by inverting the Y-axis.
                points = np.stack([point_array[:, 0], -point_array[:, 1], point_array[:, 2]], axis=1)
                point_cloud.points = o3d.utility.Vector3dVector(points)
                
                if not point_cloud_added:
                    vis.add_geometry(point_cloud)
                    point_cloud_added = True
                else:
                    vis.update_geometry(point_cloud)

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
    time.sleep(1)  # Allow time for initialisation

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
    yolo_thread = threading.Thread(target=yolo_inference_thread, daemon=True)  # <-- Start YOLO inference thread

    camera_reader_thread.start()
    lidar_reader_thread.start()
    camera_processor_thread.start()
    lidar_processor_thread.start()
    yolo_thread.start()  # <-- Start YOLO inference thread

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads_event.set()
        camera_reader_thread.join()
        lidar_reader_thread.join()
        camera_processor_thread.join()
        lidar_processor_thread.join()
