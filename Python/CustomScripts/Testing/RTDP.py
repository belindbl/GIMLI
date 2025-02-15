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
from sklearn.cluster import DBSCAN  # Added for segmentation

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
ALSweights = "E:/Team1Docs/yolov5/weights_ailivesim/best.pt"
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

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
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = model(dummy_image)  # Force initialisation
        model.conf = 0.4
        model.iou = 0.5

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
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

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
    Retrieves the latest LiDAR data, applies segmentation with persistent tracking, 
    and updates the Open3D visualisation.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualisation")

    # Set visualisation options: light gray background and point size 2.
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([0.8, 0.8, 0.8])
    render_option.point_size = 2

    # Create a point cloud object.
    point_cloud = o3d.geometry.PointCloud()
    # Initially paint it dark blue.
    point_cloud.paint_uniform_color([0, 0, 0.5])
    
    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 300])
    ctr.set_zoom(0.02)
    ctr.set_front([-1, 0, 1])
    ctr.set_up([1, 0, 1])
    
    point_cloud_added = False  # Flag to check if the geometry has been added

    # --- Persistent cluster tracking variables ---
    tracked_clusters = {}  # persistent id -> last known centroid (np.array)
    next_cluster_id = 0
    max_tracking_distance = 100.0  # Threshold to match clusters between frames

    # Predefined custom colours for first few persistent IDs.
    custom_colors = {
        0: [1, 0, 0],      # red
        1: [0, 1, 0],      # green
        2: [0, 0, 1],      # blue
        3: [1, 1, 0],      # yellow
        4: [1, 0, 1],      # magenta
        5: [0, 1, 1],      # cyan
        6: [0.5, 0, 0],    # dark red
        7: [0, 0.5, 0],    # dark green
        8: [0, 0, 0.5],    # dark blue
        9: [1, 0.75, 0]    # orange
    }

    while not stop_threads_event.is_set():
        with data_lock:
            lidar_data = sensor_data.get('lidar')
        if lidar_data:
            try:
                sizeofFloat = 4
                header_count = 11  # Number of header floats.
                index = header_count
                # Unpack header and get point cloud data.
                _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
                pointCloudData = lidar_data[header_count * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                point_array = np.reshape(point_array, (-1, 4))
                # Adjust coordinate system: invert Y.
                points = np.stack([point_array[:, 0], -point_array[:, 1], point_array[:, 2]], axis=1)

                # --- Segmentation begins here ---
                # Define boat bounding box.
                min_coords = np.array([-206, -159, 162])
                max_coords = np.array([513, 159, 321])
                # Create mask: True for points outside the boat.
                mask = ~(
                    (points[:, 0] >= min_coords[0]) & (points[:, 0] <= max_coords[0]) &
                    (points[:, 1] >= min_coords[1]) & (points[:, 1] <= max_coords[1]) &
                    (points[:, 2] >= min_coords[2]) & (points[:, 2] <= max_coords[2])
                )
                # Points outside the boat:
                filtered_points = points[mask]
                if filtered_points.shape[0] > 0:
                    dbscan = DBSCAN(eps=200, min_samples=4)
                    labels = dbscan.fit_predict(filtered_points)
                else:
                    labels = np.array([])

                # For the points inside the boat, we keep them as is.
                # --- Now compute persistent cluster IDs for the filtered points ---
                # Compute new centroids for each DBSCAN cluster.
                new_cluster_centroids = {}
                for label in np.unique(labels):
                    if label == -1:
                        continue  # Skip noise.
                    indices = np.where(labels == label)[0]
                    cluster_points = filtered_points[indices]
                    centroid = cluster_points.mean(axis=0)
                    new_cluster_centroids[label] = centroid

                # Map DBSCAN labels to persistent IDs.
                persistent_ids = {}
                for label, centroid in new_cluster_centroids.items():
                    best_match = None
                    best_distance = np.inf
                    # Try to find a match from previous frame.
                    for pid, old_centroid in tracked_clusters.items():
                        distance = np.linalg.norm(centroid - old_centroid)
                        if distance < best_distance:
                            best_distance = distance
                            best_match = pid
                    if best_distance < max_tracking_distance:
                        persistent_ids[label] = best_match
                    else:
                        persistent_ids[label] = next_cluster_id
                        next_cluster_id += 1

                # Update tracker with new centroids.
                for label, pid in persistent_ids.items():
                    tracked_clusters[pid] = new_cluster_centroids[label]

                # --- Assign colours based on persistent IDs ---
                point_colors = np.zeros((points.shape[0], 3))
                # Boat points (inside the bounding box) get dark blue.
                point_colors[~mask] = [0, 0, 0.5]
                # For each filtered point, assign a colour.
                filtered_indices = np.where(mask)[0]
                for i, idx in enumerate(filtered_indices):
                    db_label = labels[i] if i < len(labels) else -1
                    if db_label == -1:
                        point_colors[idx] = [0, 0, 0]  # noise as black
                    else:
                        pid = persistent_ids.get(db_label, db_label)
                        if pid in custom_colors:
                            point_colors[idx] = custom_colors[pid]
                        else:
                            # Generate a random but deterministic colour using the persistent id.
                            np.random.seed(pid)
                            point_colors[idx] = np.random.rand(3)
                # --- Segmentation ends here ---

                # Update the point cloud with new points and colours.
                point_cloud.points = o3d.utility.Vector3dVector(points)
                point_cloud.colors = o3d.utility.Vector3dVector(point_colors)
                
                # Add or update the geometry in the visualiser.
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
    yolo_thread = threading.Thread(target=yolo_inference_thread, daemon=True)

    camera_reader_thread.start()
    lidar_reader_thread.start()
    camera_processor_thread.start()
    lidar_processor_thread.start()
    yolo_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads_event.set()
        camera_reader_thread.join()
        lidar_reader_thread.join()
        camera_processor_thread.join()
        lidar_processor_thread.join()
