# This code implements the lidar projection on the camera image precisely. 
# Fixing and separating the Yolo thread is still required
# Here is the output of YOLO + Brightening + lidar projection, BUT it is with a delay
# next: CUDA acceleration and clustering

# Looking into filtering points before applying the projection

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
from sklearn.cluster import DBSCAN

# Global settings and shared objects
HOST = '127.0.0.1'
stop_threads_event = threading.Event()

# Shared sensor data dictionary (protected by a lock)
sensor_data = {'camera': None, 'lidar': None}
data_lock = threading.Lock()

# Queue for passing camera frames (only stores the latest frame)
frame_queue = queue.Queue(maxsize=1)

# Global variable to hold the YOLO output image and bounding boxes
yolo_boxes = []
yolo_img_lock = threading.Lock()
yolo_output_img = None

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

# YOLO model weight paths
ALSweights = "E:/Team1Docs/yolov5/weights_ailivesim/best.pt"
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

# Image enhancement to improve YOLO accuracy
def apply_clahe(image):
    # Convert the image to LAB color space
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    # Create a CLAHE object with a clip limit of 5.0 and a tile grid size of 3x3
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(3,3))
    cl = clahe.apply(l)
    # Merge the CLAHE enhanced L channel back with the a and b channels
    merged = cv2.merge((cl, a, b))
    # Convert back to BGR color space
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

######################################
# New Thread: Overlay LiDAR on Camera
######################################
def overlay_lidar_on_camera_thread():
    """
    This thread fetches the latest camera image and LiDAR data,
    projects the LiDAR 3D points onto the 2D image using camera calibration,
    and overlays the points onto the image.
    It now clusters the valid LiDAR points using DBSCAN and uses the cluster labels for coloring.
    """
    # Define camera calibration parameters
    image_width, image_height = 720, 480
    fov_angle = 90
    focal_length = image_width / (2 * np.tan(np.radians(fov_angle) / 2))
    cx, cy = image_width / 2, image_height / 2
    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])
    
    # Define extrinsic transformation (world-to-camera)
    t_cam = np.array([105.941, -0.13, 118.285])
    R_wc = np.array([
        [ 0, 1,  0],  # X_world -> Z_cam
        [ 0,  0, -1],  # Y_world -> X_cam (with sign flip)
        [ 1,  0,  0]   # Z_world -> Y_cam
    ])
    T = np.eye(4)
    T[:3, :3] = R_wc
    T[:3, 3] = -R_wc @ t_cam

    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
            lidar_data = sensor_data.get('lidar')
        if cam_data is None or lidar_data is None:
            time.sleep(0.02)
            continue
        try:
            # Use the YOLO output image if available; otherwise, get the raw image.
            with yolo_img_lock:
                if yolo_output_img is not None:
                    img = yolo_output_img.copy()
                else:
                    index = 0
                    img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                    img = cv2.resize(img, (image_width, image_height), interpolation=cv2.INTER_LINEAR)
            
            # Process LiDAR data: Remove header and decode points
            sizeofFloat = 4
            header_count = 11
            _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
            pointCloudData = lidar_data[header_count * sizeofFloat:]
            point_array = np.frombuffer(pointCloudData, dtype=np.float32)
            point_array = np.reshape(point_array, (-1, 4))
            points = point_array[:, :3]  # Use only x, y, z
            intensity = point_array[:, 3]  # Extract intensity

            # Project 3D points to camera image plane
            num_points = points.shape[0]
            points_hom = np.hstack((points, np.ones((num_points, 1))))  # Convert to homogeneous coords
            points_cam_hom = (T @ points_hom.T).T
            points_cam = points_cam_hom[:, :3]

            # Keep points with positive depth
            valid_mask = points_cam[:, 2] > 0
            points_cam = points_cam[valid_mask]
            intensity = intensity[valid_mask]

            if points_cam.shape[0] == 0:
                time.sleep(0.05)
                continue

            # Run DBSCAN clustering on the original valid 3D points (world coordinates)
            valid_points = points[valid_mask]
            dbscan = DBSCAN(eps=200, min_samples=4)
            labels = dbscan.fit_predict(valid_points)

            # Project onto 2D image
            proj = (K @ points_cam.T).T
            proj[:, 0] /= proj[:, 2]
            proj[:, 1] /= proj[:, 2]
            proj_points = proj[:, :2]

            # Define cluster colors (same as used in the 3D visualization)
            custom_colors = {
                0: (0, 0, 255),    # Red
                1: (0, 255, 0),    # Green
                2: (255, 0, 0),    # Blue
                3: (255, 255, 0),  # Yellow
                4: (255, 0, 255),  # Magenta
                5: (0, 255, 255),  # Cyan
                -1: (100, 100, 100)  # Noise points (gray)
            }

            # Overlay projected points with their cluster colors
            for i, pt in enumerate(proj_points):
                x, y = int(round(pt[0])), int(round(pt[1]))
                if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                    cluster_label = labels[i] if i < len(labels) else -1
                    color = custom_colors.get(cluster_label, (255, 255, 255))  # Default white
                    cv2.circle(img, (x, y), 2, color, thickness=-1)

            # Display the resulting clustered LiDAR projection
            ALSImg.JustDisplay(img)
        except Exception as e:
            print("Overlay LiDAR on camera error:", e)
        time.sleep(0.05)

#############################
# YOLO Inference Functions  #
#############################
def process_frame(image, model):
    """Apply YOLO detection on the image."""
    if len(image.shape) == 2:  # Grayscale
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:  # RGBA
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    results = model(image)
    processed_image = results.render()[0]
    return cv2.cvtColor(processed_image, cv2.COLOR_RGB2RGBA), results

def yolo_inference_thread():
    """Thread dedicated to running YOLO inference on frames from the queue."""
    print("Loading YOLOv5 model in YOLO thread...", flush=True)
    try:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=MixedWeights,
                                force_reload=False, source='github')
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        _, _ = process_frame(dummy_image, model)  # Force initialization
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

    global yolo_boxes, yolo_output_img
    while not stop_threads_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        frame = apply_clahe(frame)
        processed_img, results = process_frame(frame, model)
        # Extract bounding boxes (each row: [xmin, ymin, xmax, ymax, conf, class])
        try:
            boxes = results.xyxy[0].cpu().numpy()
        except Exception as e:
            boxes = []
        with yolo_img_lock:
            yolo_output_img = processed_img.copy()
        # (Optionally, display the YOLO processed image)
        # ALSImg.JustDisplay(processed_img)
        
#############################
# Camera Functions          #
#############################
def camera_data_reader():
    """Continuously read camera data from TCP and update the shared sensor_data."""
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

def camera_processing_thread():
    """Extract frames from sensor_data and push them to the inference queue."""
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

#############################
# LiDAR Processing Helpers  #
#############################
def setup_visualizer():
    """Initializes the Open3D visualizer, point cloud, and view control."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualisation")
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([0.8, 0.8, 0.8])
    render_option.point_size = 2

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.paint_uniform_color([0, 0, 0.5])

    ctr = vis.get_view_control()
    ctr.set_lookat([0, 0, 300])
    ctr.set_zoom(0.1)
    ctr.set_front([-1, 0, 1])
    ctr.set_up([1, 0, 1])
    return vis, point_cloud, ctr

def filter_boat_points(points, min_coords, max_coords):
    """Create a mask for points outside the boat's bounding box."""
    mask = ~(
        (points[:, 0] >= min_coords[0]) & (points[:, 0] <= max_coords[0]) &
        (points[:, 1] >= min_coords[1]) & (points[:, 1] <= max_coords[1]) &
        (points[:, 2] >= min_coords[2]) & (points[:, 2] <= max_coords[2])
    )
    return mask, points[mask]

def compute_cluster_centroids(filtered_points, labels):
    """Compute centroids for each DBSCAN cluster."""
    centroids = {}
    for label in np.unique(labels):
        if label == -1:
            continue
        indices = np.where(labels == label)[0]
        centroids[label] = filtered_points[indices].mean(axis=0)
    return centroids

def update_persistent_ids(new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance):
    """
    Map new DBSCAN cluster labels to persistent IDs.
    Returns a mapping {db_label: persistent_id} along with updated tracked_clusters and next_cluster_id.
    """
    persistent_ids = {}
    for label, centroid in new_centroids.items():
        best_match = None
        best_distance = np.inf
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
    for label, pid in persistent_ids.items():
        tracked_clusters[pid] = new_centroids[label]
    return persistent_ids, tracked_clusters, next_cluster_id

def assign_cluster_colors(points, mask, labels, persistent_ids, custom_colors):
    """Assign colors to points based on their persistent cluster IDs."""
    point_colors = np.zeros((points.shape[0], 3))
    point_colors[~mask] = [0, 0, 0.5]
    filtered_indices = np.where(mask)[0]
    for i, idx in enumerate(filtered_indices):
        db_label = labels[i] if i < len(labels) else -1
        if db_label == -1:
            point_colors[idx] = [0, 0, 0]
        else:
            pid = persistent_ids.get(db_label, db_label)
            if pid in custom_colors:
                point_colors[idx] = custom_colors[pid]
            else:
                np.random.seed(pid)
                point_colors[idx] = np.random.rand(3)
    return point_colors

#############################
# LiDAR Processing Thread   #
#############################
def lidar_processing_thread():
    """Process LiDAR data, perform segmentation with persistent tracking, and update visualisation."""
    vis, point_cloud, ctr = setup_visualizer()
    point_cloud_added = False

    tracked_clusters = {}
    next_cluster_id = 0
    max_tracking_distance = 200

    custom_colors = {
        0: [1, 0, 0], 1: [0, 1, 0], 2: [0, 0, 1],
        3: [1, 1, 0], 4: [1, 0, 1], 5: [0, 1, 1],
        6: [0.5, 0, 0], 7: [0, 0, 0.5], 8: [0, 0, 0],
        9: [1, 0.75, 0]
    }

    eps = 200
    min_samples = 4
    min_coords = np.array([-206, -159, 162])
    max_coords = np.array([513, 159, 321])

    while not stop_threads_event.is_set():
        with data_lock:
            lidar_data = sensor_data.get('lidar')
        if lidar_data:
            try:
                sizeofFloat = 4
                header_count = 11
                _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
                pointCloudData = lidar_data[header_count * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                point_array = np.reshape(point_array, (-1, 4))
                points = np.stack([point_array[:, 0], point_array[:, 1], point_array[:, 2]], axis=1)

                mask, filtered_points = filter_boat_points(points, min_coords, max_coords)

                if filtered_points.shape[0] > 0:
                    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
                    labels = dbscan.fit_predict(filtered_points)
                else:
                    labels = np.array([])

                new_centroids = compute_cluster_centroids(filtered_points, labels)

                persistent_ids, tracked_clusters, next_cluster_id = update_persistent_ids(
                    new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance
                )

                point_colors = assign_cluster_colors(points, mask, labels, persistent_ids, custom_colors)

                point_cloud.points = o3d.utility.Vector3dVector(points)
                point_cloud.colors = o3d.utility.Vector3dVector(point_colors)

                if not point_cloud_added:
                    vis.add_geometry(point_cloud)
                    point_cloud_added = True
                else:
                    vis.update_geometry(point_cloud)

                ctr.set_lookat([0, 0, 300])
                ctr.set_zoom(0.02)
                ctr.set_front([0, 0, -1])
                ctr.set_up([1, 0, 0])
                vis.poll_events()
                vis.update_renderer()
            except Exception as e:
                print(f"LiDAR processing error: {e}")
        time.sleep(0.1)
    vis.destroy_window()

#############################
# LiDAR Data Reader         #
#############################
def lidar_data_reader():
    """Continuously read LiDAR data from TCP and update the shared sensor_data."""
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

#############################
# Main Script Initialization#
#############################
if __name__ == "__main__":
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Aboat')
    time.sleep(1)

    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)
    for sensor in parsed_json['sensors']:
        if sensor['path'] == 'Sensors.[0]':
            camera_sensor = sensor
            break
    camera_port = camera_sensor['sensor_port']
    camera_host = camera_sensor['sensor_ip']
    print("Connecting to Camera:", camera_host, camera_port)
    print("Connecting to LiDAR:", HOST, 8881)

    camera_reader_thread = threading.Thread(target=camera_data_reader, daemon=True)
    lidar_reader_thread = threading.Thread(target=lidar_data_reader, daemon=True)
    camera_processor_thread = threading.Thread(target=camera_processing_thread, daemon=True)
    lidar_processor_thread = threading.Thread(target=lidar_processing_thread, daemon=True)
    yolo_thread = threading.Thread(target=yolo_inference_thread, daemon=True)
    overlay_thread = threading.Thread(target=overlay_lidar_on_camera_thread, daemon=True)

    camera_reader_thread.start()
    lidar_reader_thread.start()
    camera_processor_thread.start()
    lidar_processor_thread.start()
    yolo_thread.start()
    overlay_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_threads_event.set()
        camera_reader_thread.join()
        lidar_reader_thread.join()
        camera_processor_thread.join()
        lidar_processor_thread.join()
        overlay_thread.join()
