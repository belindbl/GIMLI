'''Experimental version'''

# TODO Attempt to make the clustering algorithm distance-agnostic.
# TODO Verify that the overlay thread is working after breaking it down into smaller helper functions

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

# Global variable to share the latest centroids from LiDAR processing
latest_centroids = {}
# Global variable to store overlay texts for distance/angle (persist until update)
overlay_texts = {}

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

# YOLO model weight paths
ALSweights = "E:/Team1Docs/yolov5/weights_ailivesim/best.pt"
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

RADIUS_EARTH = 637100000

def linear_to_angular_distance(linear_distance_meters):
    """
    Converts a linear distance (in cm) to an angular distance in degrees.
    """
    angular_distance_degrees = (linear_distance_meters / RADIUS_EARTH) * (180 / np.pi)
    # Convert degrees to minutes (1 degree = 60 minutes)
    angular_distance_minutes = angular_distance_degrees * 60
    return angular_distance_minutes

# Image enhancement to improve YOLO accuracy
def apply_clahe(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(3, 3))
    cl = clahe.apply(l)
    merged = cv2.merge((cl, a, b))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

######################################
# New Thread: Overlay LiDAR on Camera
######################################

# --- Helper Functions ---

def get_camera_intrinsics(image_width, image_height, fov_angle):
    """
    Calculate and return the camera intrinsic matrix.
    """
    focal_length = image_width / (2 * np.tan(np.radians(fov_angle) / 2))
    cx, cy = image_width / 2, image_height / 2
    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])
    return K

def get_extrinsic_transform(t_cam, R_wc):
    """
    Compute the world-to-camera transformation matrix.
    """
    T = np.eye(4)
    T[:3, :3] = R_wc
    T[:3, 3] = -R_wc @ t_cam
    return T

def fetch_image(cam_data, image_width, image_height):
    """
    Retrieve an image from the camera data.
    Uses YOLO output if available; otherwise falls back to raw stream.
    """
    global yolo_output_img, yolo_img_lock  # Assumes these globals are defined elsewhere.
    with yolo_img_lock:
        if yolo_output_img is not None:
            img = yolo_output_img.copy()
        else:
            index = 0
            # ALSFunc.ReadImage_Stream should return (image, index, width, height)
            img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
            img = cv2.resize(img, (image_width, image_height), interpolation=cv2.INTER_LINEAR)
    return img

def process_lidar_data(lidar_data):
    """
    Decode LiDAR binary data.
    Removes header information and extracts the 3D points and intensity.
    """
    sizeofFloat = 4
    header_count = 11
    _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
    pointCloudData = lidar_data[header_count * sizeofFloat:]
    point_array = np.frombuffer(pointCloudData, dtype=np.float32)
    point_array = np.reshape(point_array, (-1, 4))
    points = point_array[:, :3]
    intensity = point_array[:, 3]
    return points, intensity

def project_points(points, K, T):
    """
    Projects 3D points into the camera image plane using the provided intrinsic (K)
    and extrinsic (T) matrices.
    
    Returns the 2D projected points and a validity mask for points with positive depth.
    """
    num_points = points.shape[0]
    points_hom = np.hstack((points, np.ones((num_points, 1))))
    points_cam_hom = (T @ points_hom.T).T
    points_cam = points_cam_hom[:, :3]
    valid_mask = points_cam[:, 2] > 0

    # Only project valid points.
    valid_points_cam = points_cam[valid_mask]
    proj = (K @ valid_points_cam.T).T
    proj[:, 0] /= proj[:, 2]
    proj[:, 1] /= proj[:, 2]
    proj_points = proj[:, :2]
    return proj_points, valid_mask

def cluster_points(points, valid_mask):
    """
    Run DBSCAN clustering on the valid LiDAR 3D points.
    """
    valid_points = points[valid_mask]
    dbscan = DBSCAN(eps=400, min_samples=4)
    labels = dbscan.fit_predict(valid_points)
    return labels

def draw_projected_points(image, proj_points, labels, custom_colors):
    """
    Draw the projected 2D LiDAR points onto the image using cv2.circle().
    The color is determined based on the cluster label.
    """
    for i, pt in enumerate(proj_points):
        x, y = int(round(pt[0])), int(round(pt[1]))
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            # Use label if available, else default to -1.
            cluster_label = labels[i] if i < len(labels) else -1
            color = custom_colors.get(cluster_label, (255, 255, 255))
            cv2.circle(image, (x, y), 2, color, thickness=-1)

def update_overlay_texts(T, K, boat_center, latest_centroids, last_text_update, overlay_texts):
    """
    Update overlay texts every 1.5 seconds based on cluster centroids.
    The text shows the relative distance (in meters) and angle.
    """
    current_time = time.time()
    if current_time - last_text_update > 1.5:
        overlay_texts = {}
        for label, centroid in latest_centroids.items():
            relative = centroid - boat_center
            # Convert distance from centimetres to metres.
            distance_cm = np.sqrt(relative[0]**2 + relative[1]**2)
            distance_m = distance_cm * 0.01
            angle = np.degrees(np.arctan2(relative[1], relative[0]))
            text = f"{distance_m:.1f}m, {angle:.1f} deg"

            centroid_hom = np.append(centroid, 1)
            centroid_cam = T @ centroid_hom
            if centroid_cam[2] <= 0:
                continue
            proj_centroid = K @ centroid_cam[:3]
            proj_centroid[0] /= proj_centroid[2]
            proj_centroid[1] /= proj_centroid[2]
            image_x, image_y = int(round(proj_centroid[0])), int(round(proj_centroid[1]))
            # Position the text 40 pixels above the projected centroid.
            overlay_texts[label] = (text, (image_x, image_y - 50))
        last_text_update = current_time
    return overlay_texts, last_text_update

def draw_overlay_texts(image, overlay_texts, font, font_scale, thickness):
    """
    Draw overlay text on the image.
    """
    for text, pos in overlay_texts.values():
        cv2.putText(image, text, pos, font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

# --- Main Thread Function ---

def overlay_lidar_on_camera_thread():
    """
    This thread fetches the latest camera image and LiDAR data,
    projects the LiDAR 3D points onto the 2D image using calibration data,
    overlays the points and cluster text on the image, and displays the result.
    """
    global overlay_texts, sensor_data, stop_threads_event, yolo_output_img, yolo_img_lock, latest_centroids, data_lock

    # Camera calibration parameters.
    image_width, image_height = 720, 480
    fov_angle = 90
    K = get_camera_intrinsics(image_width, image_height, fov_angle)
    
    # Extrinsic transformation (world-to-camera).
    t_cam = np.array([105.941, -0.13, 118.285])
    R_wc = np.array([
        [0, 1,  0],
        [0, 0, -1],
        [1, 0,  0]
    ])
    T = get_extrinsic_transform(t_cam, R_wc)
    
    last_text_update = 0
    # Define boat bounding box and compute its centre (in centimetres).
    min_coords = np.array([-206, -159, 162])
    max_coords = np.array([513, 159, 321])
    boat_center = (min_coords + max_coords) / 2

    # Font settings for overlay text.
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 2

    custom_colors = {
        0: (0, 0, 255),
        1: (0, 255, 0),
        2: (255, 0, 0),
        3: (255, 255, 0),
        4: (255, 0, 255),
        5: (0, 255, 255),
        -1: (100, 100, 100)
    }

    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
            lidar_data = sensor_data.get('lidar')
        if cam_data is None or lidar_data is None:
            time.sleep(0.02)
            continue

        try:
            # Fetch and resize the image.
            img = fetch_image(cam_data, image_width, image_height)
            
            # Process LiDAR data.
            points, intensity = process_lidar_data(lidar_data)
            num_points = points.shape[0]
            if num_points == 0:
                time.sleep(0.05)
                continue

            # Project LiDAR points onto the image.
            proj_points, valid_mask = project_points(points, K, T)
            if proj_points.shape[0] == 0:
                time.sleep(0.05)
                continue

            # Cluster the valid 3D LiDAR points.
            labels = cluster_points(points, valid_mask)

            # Draw the projected points onto the image.
            draw_projected_points(img, proj_points, labels, custom_colors)

            # Update and draw overlay texts.
            overlay_texts, last_text_update = update_overlay_texts(T, K, boat_center, latest_centroids,
                                                                     last_text_update, overlay_texts)
            draw_overlay_texts(img, overlay_texts, font, font_scale, thickness)

            # Display the final image.
            ALSImg.JustDisplay(img)

        except Exception as e:
            print("Overlay LiDAR on camera error:", e)

        time.sleep(0.05)

#############################
# YOLO Inference Functions  #
#############################
def process_frame(image, model):
    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    results = model(image)
    processed_image = results.render()[0]
    return cv2.cvtColor(processed_image, cv2.COLOR_RGB2RGBA), results

def yolo_inference_thread():
    print("Loading YOLOv5 model in YOLO thread...", flush=True)
    try:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=MixedWeights,
                                force_reload=False, source='github')
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        _, _ = process_frame(dummy_image, model)
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
        try:
            boxes = results.xyxy[0].cpu().numpy()
        except Exception as e:
            boxes = []
        with yolo_img_lock:
            yolo_output_img = processed_img.copy()
        # Optionally display the processed frame
        # ALSImg.JustDisplay(processed_img)

#############################
# Camera Functions          #
#############################
def camera_data_reader():
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
    mask = ~(
        (points[:, 0] >= min_coords[0]) & (points[:, 0] <= max_coords[0]) &
        (points[:, 1] >= min_coords[1]) & (points[:, 1] <= max_coords[1]) &
        (points[:, 2] >= min_coords[2]) & (points[:, 2] <= max_coords[2])
    )
    return mask, points[mask]

def compute_cluster_centroids(filtered_points, labels):
    centroids = {}
    for label in np.unique(labels):
        if label == -1:
            continue
        indices = np.where(labels == label)[0]
        centroids[label] = filtered_points[indices].mean(axis=0)
    return centroids

def update_persistent_ids(new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance_meters):
    persistent_ids = {}
    max_tracking_distance_angular = linear_to_angular_distance(max_tracking_distance_meters)

    for label, centroid in new_centroids.items():
        best_match = None
        best_distance = np.inf
        for pid, old_centroid in tracked_clusters.items():
            # Calculate angular distance
            distance = np.linalg.norm(centroid - old_centroid)
            angular_distance = linear_to_angular_distance(distance)
            
            if angular_distance < best_distance:
                best_distance = angular_distance
                best_match = pid

        if best_distance < max_tracking_distance_angular:
            persistent_ids[label] = best_match
        else:
            persistent_ids[label] = next_cluster_id
            next_cluster_id += 1

    for label, pid in persistent_ids.items():
        tracked_clusters[pid] = new_centroids[label]

    return persistent_ids, tracked_clusters, next_cluster_id


def assign_cluster_colors(points, mask, labels, persistent_ids, custom_colors):
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
    global latest_centroids
    vis, point_cloud, ctr = setup_visualizer()
    point_cloud_added = False

    tracked_clusters = {}
    next_cluster_id = 0
    max_tracking_distance = 600

    custom_colors = {
        0: [1, 0, 0], 1: [0, 1, 0], 2: [0, 0, 1],
        3: [1, 1, 0], 4: [1, 0, 1], 5: [0, 1, 1],
        6: [0.5, 0, 0], 7: [0, 0, 0.5], 8: [0, 0, 0],
        9: [1, 0.75, 0]
    }

    
    eps = 200
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
                    dbscan = DBSCAN(eps=eps, min_samples=4)
                    labels = dbscan.fit_predict(filtered_points)
                else:
                    labels = np.array([])

                new_centroids = compute_cluster_centroids(filtered_points, labels)
                latest_centroids = new_centroids.copy()

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
