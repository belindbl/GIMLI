'''
Experimental script to make the clustering dynamic based on the distance from the boat.

This script loads the custom scenario "Aboat" and connects to the camera and LiDAR sensors.
It starts separate threads for reading and processing camera and LiDAR data.
The camera thread processes the camera data and runs YOLOv5 object detection on the frames.
The LiDAR thread processes the LiDAR data, performs DBSCAN clustering, and visualizes the point cloud.
The overlay thread projects the LiDAR points onto the camera image and overlays the relative distance and angle.
'''

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
    """
    Context class for managing test-related settings and client connection.
    """
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), lambda msg: None)

# YOLO model weight paths
ALSweights = "E:/Team1Docs/yolov5/weights_ailivesim/best.pt"
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

RADIUS = 637100000  # Earth's radius in centimeters

def linear_to_angular_distance(linear_distance_cm):
    """
    Converts a linear distance to an angular distance in minutes.
        
    Returns:
        angular_distance_minutes (float): Angular distance in minutes (1 degree = 60 minutes).
    """
    # Convert linear distance to angular distance in degrees
    angular_distance_degrees = (linear_distance_cm / RADIUS) * (180 / np.pi)
    # Convert degrees to minutes (1 degree = 60 minutes)
    angular_distance_minutes = angular_distance_degrees * 60
    return angular_distance_minutes

def apply_clahe(image):
    """
    Applies Contrast Limited Adaptive Histogram Equalization (CLAHE) to enhance the image.
    This improves the contrast of the image for better YOLO object detection.
    """
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)    # Convert image to LAB color space
    l, a, b = cv2.split(lab)                         # Split channels
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(3, 3))  # Create CLAHE object
    cl = clahe.apply(l)                              # Apply CLAHE to the L-channel
    merged = cv2.merge((cl, a, b))                   # Merge channels back together
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)    # Convert back to BGR color space

######################################
# New Thread: Overlay LiDAR on Camera
######################################
def overlay_lidar_on_camera_thread():
    """
    This thread fetches the latest camera image and LiDAR data,
    """
    global overlay_texts
    # Camera calibration parameters
    image_width, image_height = 720, 480
    fov_angle = 90
    focal_length = image_width / (2 * np.tan(np.radians(fov_angle) / 2))
    cx, cy = image_width / 2, image_height / 2
    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])
    
    # Define extrinsic transformation from world to camera coordinates
    t_cam = np.array([105.941, -0.13, 118.285])  # Translation vector (in centimeters)
    R_wc = np.array([
        [0, 1,  0],
        [0, 0, -1],
        [1, 0,  0]
    ])
    T = np.eye(4)
    T[:3, :3] = R_wc
    T[:3, 3] = -R_wc @ t_cam

    last_text_update = 0

    # Define boat bounding box and compute its center (in centimeters)
    min_coords = np.array([-206, -159, 162])
    max_coords = np.array([513, 159, 321])
    boat_center = (min_coords + max_coords) / 2

    # Font settings for overlay text
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 2

    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
            lidar_data = sensor_data.get('lidar')
        # Skip iteration if no sensor data is available
        if cam_data is None or lidar_data is None:
            time.sleep(0.02)
            continue
        try:
            # Retrieve the latest image: prefer YOLO output if available, else use raw camera image
            with yolo_img_lock:
                if yolo_output_img is not None:
                    img = yolo_output_img.copy()
                else:
                    index = 0
                    img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                    img = cv2.resize(img, (image_width, image_height), interpolation=cv2.INTER_LINEAR)
            
            # Process LiDAR data: remove header and decode binary data into point cloud and intensity values
            sizeofFloat = 4
            header_count = 11
            _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
            pointCloudData = lidar_data[header_count * sizeofFloat:]
            point_array = np.frombuffer(pointCloudData, dtype=np.float32)
            point_array = np.reshape(point_array, (-1, 4))
            points = point_array[:, :3]      # Extract x, y, z coordinates
            intensity = point_array[:, 3]      # Extract intensity values

            # Project 3D LiDAR points into camera coordinates
            num_points = points.shape[0]
            points_hom = np.hstack((points, np.ones((num_points, 1))))  # Convert to homogeneous coordinates
            points_cam_hom = (T @ points_hom.T).T
            points_cam = points_cam_hom[:, :3]

            # Filter out points with non-positive depth
            valid_mask = points_cam[:, 2] > 0
            points_cam = points_cam[valid_mask]
            intensity = intensity[valid_mask]

            if points_cam.shape[0] == 0:
                time.sleep(0.05)
                continue

            # Apply DBSCAN clustering on the valid 3D points (in world coordinates)
            valid_points = points[valid_mask]
            dbscan = DBSCAN(eps=400, min_samples=4)
            labels = dbscan.fit_predict(valid_points)

            # Project valid 3D points onto the 2D image plane using camera intrinsics
            proj = (K @ points_cam.T).T
            proj[:, 0] /= proj[:, 2]
            proj[:, 1] /= proj[:, 2]
            proj_points = proj[:, :2]

            # Define colors for different clusters (including noise with label -1)
            custom_colors = {
                0: (0, 0, 255),
                1: (0, 255, 0),
                2: (255, 0, 0),
                3: (255, 255, 0),
                4: (255, 0, 255),
                5: (0, 255, 255),
                -1: (100, 100, 100)
            }

            # Draw each projected point on the image as a small circle with color based on cluster label
            for i, pt in enumerate(proj_points):
                x, y = int(round(pt[0])), int(round(pt[1]))
                if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                    cluster_label = labels[i] if i < len(labels) else -1
                    color = custom_colors.get(cluster_label, (255, 255, 255))
                    cv2.circle(img, (x, y), 2, color, thickness=-1)

            # Update overlay texts (distance and angle) every 1.5 seconds using cluster centroids
            current_time = time.time()
            if current_time - last_text_update > 1.5:
                overlay_texts = {}
                for label, centroid in latest_centroids.items():
                    relative = centroid - boat_center
                    # Calculate distance (convert centimeters to meters)
                    distance_cm = np.sqrt(relative[0]**2 + relative[1]**2)
                    distance_m = distance_cm * 0.01
                    angle = np.degrees(np.arctan2(relative[1], relative[0]))
                    text = f"{distance_m:.1f}m, {angle:.1f} deg"

                    # Project the centroid onto the image for text overlay
                    centroid_hom = np.append(centroid, 1)
                    centroid_cam = T @ centroid_hom
                    if centroid_cam[2] <= 0:
                        continue
                    proj_centroid = K @ centroid_cam[:3]
                    proj_centroid[0] /= proj_centroid[2]
                    proj_centroid[1] /= proj_centroid[2]
                    image_x, image_y = int(round(proj_centroid[0])), int(round(proj_centroid[1]))
                    # Position text 40 pixels above the projected centroid
                    overlay_texts[label] = (text, (image_x, image_y - 50))
                last_text_update = current_time

            # Draw overlay texts onto the image
            for text, pos in overlay_texts.values():
                cv2.putText(img, text, pos, font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

            # Display the final image with overlays using the ALSImg library
            ALSImg.JustDisplay(img)
        except Exception as e:
            print("Overlay LiDAR on camera error:", e)
        time.sleep(0.05)

#############################
# YOLO Inference Functions  #
#############################
def process_frame(image, model):
    """
    Processes a single image frame using the YOLOv5 model.
    Converts image to the proper format if necessary, runs inference,
    and returns the processed image along with the detection results.

    Parameters:
        image (numpy.ndarray): The input image frame.
        model: The YOLOv5 model instance.
    
    Returns:
        processed_image (numpy.ndarray): The image with rendered detection outputs.
        results: The raw results object from the YOLOv5 model.
    """
    # Convert image to RGB
    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    results = model(image)
    processed_image = results.render()[0]
    # Convert the processed image back to RGBA for consistency
    return cv2.cvtColor(processed_image, cv2.COLOR_RGB2RGBA), results

# Inference thread which loads the model and processes frames
def yolo_inference_thread():
    print("Loading YOLOv5 model in YOLO thread...", flush=True)
    try:
        # Load custom YOLOv5 model using Torch Hub
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
    # Continuously process frames from the queue until a stop signal is received
    while not stop_threads_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        frame = apply_clahe(frame)  # Enhance frame for better detection
        processed_img, results = process_frame(frame, model)
        try:
            boxes = results.xyxy[0].cpu().numpy()  # Extract bounding boxes
        except Exception as e:
            boxes = []
        with yolo_img_lock:
            yolo_output_img = processed_img.copy()
        # Optionally, display the processed frame using ALSImg.JustDisplay(processed_img)
        
#############################
# Camera Functions          #
#############################
def camera_data_reader():
    """
    Connects to the camera sensor using ALSLib's TCP client.
    Continuously reads data from the camera and stores it in the global sensor_data dictionary.
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

def camera_processing_thread():
    """
    Processes raw camera data retrieved from the sensor.
    Extracts individual frames using ALSFunc and ensures that only the latest frame is queued for YOLO processing.
    """
    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
        if cam_data:
            try:
                index = 0
                img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                # Ensure the queue only holds the latest frame by clearing it if full
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
    """
    Initializes and configures the Open3D visualizer for displaying the LiDAR point cloud.
    
    Returns:
        vis: The Open3D Visualizer instance.
        point_cloud: The Open3D PointCloud object.
        ctr: The view control object for the visualizer.
    """
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
    """
    Filters out points that belong to the boat by applying boundary constraints.
    
    Parameters:
        points (numpy.ndarray): Array of 3D points.
        min_coords (numpy.ndarray): Minimum x, y, z values defining the boat's boundary.
        max_coords (numpy.ndarray): Maximum x, y, z values defining the boat's boundary.
        
    Returns:
        mask (numpy.ndarray): Boolean mask of points not belonging to the boat.
        filtered_points (numpy.ndarray): Points that are outside the boat's boundaries.
    """
    mask = ~(
        (points[:, 0] >= min_coords[0]) & (points[:, 0] <= max_coords[0]) &
        (points[:, 1] >= min_coords[1]) & (points[:, 1] <= max_coords[1]) &
        (points[:, 2] >= min_coords[2]) & (points[:, 2] <= max_coords[2])
    )
    return mask, points[mask]

def compute_cluster_centroids(filtered_points, labels):
    """
    Computes the centroids of clusters obtained from DBSCAN.
    
    Parameters:
        filtered_points (numpy.ndarray): Array of points after filtering.
        labels (numpy.ndarray): Array of cluster labels corresponding to filtered_points.
        
    Returns:
        centroids (dict): Dictionary mapping each cluster label to its centroid (average position).
    """
    centroids = {}
    for label in np.unique(labels):
        if label == -1:  # Ignore noise points
            continue
        indices = np.where(labels == label)[0]
        centroids[label] = filtered_points[indices].mean(axis=0)
    return centroids

def update_persistent_ids(new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance_meters):
    """
    Assigns persistent IDs to clusters to track them over time.
    If the angular distance between a new centroid and an existing tracked centroid is below a threshold,
    the existing persistent ID is retained; otherwise, a new ID is assigned.
    
    Parameters:
        new_centroids (dict): Current centroids from clustering.
        tracked_clusters (dict): Previously tracked centroids.
        next_cluster_id (int): Next available persistent ID.
        max_tracking_distance_meters (float): Maximum allowed tracking distance (in meters).
    
    Returns:
        persistent_ids (dict): Mapping from cluster labels to persistent IDs.
        tracked_clusters (dict): Updated tracked clusters with new centroids.
        next_cluster_id (int): Updated next available persistent ID.
    """
    persistent_ids = {}
    max_tracking_distance_angular = linear_to_angular_distance(max_tracking_distance_meters)

    for label, centroid in new_centroids.items():
        best_match = None
        best_distance = np.inf
        for pid, old_centroid in tracked_clusters.items():
            # Calculate angular distance between centroids
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
    """
    Assigns colors to points for visualization based on their cluster's persistent ID.
    Points that are filtered out (e.g., belonging to the boat) are assigned a default color.
    
    Parameters:
        points (numpy.ndarray): Original array of 3D points.
        mask (numpy.ndarray): Boolean mask for points to be colored.
        labels (numpy.ndarray): Cluster labels for the filtered points.
        persistent_ids (dict): Mapping from DBSCAN labels to persistent IDs.
        custom_colors (dict): Predefined colors for persistent IDs.
        
    Returns:
        point_colors (numpy.ndarray): Array of colors for each point.
    """
    point_colors = np.zeros((points.shape[0], 3))
    point_colors[~mask] = [0, 0, 0.5]  # Color for points filtered out (e.g., boat points)
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
    """
    Processes LiDAR data by filtering out points belonging to the boat,
    clustering the remaining points using DBSCAN, computing cluster centroids,
    assigning persistent IDs for tracking, and visualizing the point cloud with colors.
    The visualizer is updated in real time.
    """
    global latest_centroids
    vis, point_cloud, ctr = setup_visualizer()
    point_cloud_added = False

    tracked_clusters = {}  # Dictionary for storing previously tracked cluster centroids
    next_cluster_id = 0 # Counter for assigning new persistent IDs
    max_tracking_distance = 600  # Maximum distance (in meters) for tracking clusters

    custom_colors = {
        0: [1, 0, 0], 1: [0, 1, 0], 2: [0, 0, 1],
        3: [1, 1, 0], 4: [1, 0, 1], 5: [0, 1, 1],
        6: [0.5, 0, 0], 7: [0, 0, 0.5], 8: [0, 0, 0],
        9: [1, 0.75, 0]
    }

    eps = 200  # DBSCAN parameter: maximum distance between points in a cluster
    min_coords = np.array([-206, -159, 162])  # Minimum coordinates to filter boat points
    max_coords = np.array([513, 159, 321]) # Maximum coordinates to filter boat points

    while not stop_threads_event.is_set():
        with data_lock:
            lidar_data = sensor_data.get('lidar')
        if lidar_data:
            try:
                # Decode LiDAR data: remove header and reshape binary data into a point cloud array
                sizeofFloat = 4
                header_count = 11
                _ = struct.unpack('<fffffffffff', lidar_data[:header_count * sizeofFloat])
                pointCloudData = lidar_data[header_count * sizeofFloat:]
                point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                point_array = np.reshape(point_array, (-1, 4))
                points = np.stack([point_array[:, 0], point_array[:, 1], point_array[:, 2]], axis=1)

                # Filter out points that belong to the boat
                mask, filtered_points = filter_boat_points(points, min_coords, max_coords)

                if filtered_points.shape[0] > 0:
                    dbscan = DBSCAN(eps=eps, min_samples=4)
                    labels = dbscan.fit_predict(filtered_points)
                else:
                    labels = np.array([])

                # Compute centroids of each cluster and update global latest_centroids
                new_centroids = compute_cluster_centroids(filtered_points, labels)
                latest_centroids = new_centroids.copy()

                # Update persistent IDs for tracking clusters over time
                persistent_ids, tracked_clusters, next_cluster_id = update_persistent_ids(
                    new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance
                )

                # Assign colors to points for visualization
                point_colors = assign_cluster_colors(points, mask, labels, persistent_ids, custom_colors)

                # Update the Open3D point cloud
                point_cloud.points = o3d.utility.Vector3dVector(points)
                point_cloud.colors = o3d.utility.Vector3dVector(point_colors)

                if not point_cloud_added:
                    vis.add_geometry(point_cloud)
                    point_cloud_added = True
                else:
                    vis.update_geometry(point_cloud)

                # Update view parameters for the visualizer
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
    """
    Connects to the LiDAR sensor using ALSLib's TCP client.
    Continuously reads LiDAR data and stores it in the global sensor_data dictionary.
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

#############################
# Main Script Initialization#
#############################
if __name__ == "__main__":
    # Connects to the simulation and loads the specified custom scenario
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Aboat')
    time.sleep(1)

    # Retrieve sensor configuration for the camera from the simulation client
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

    # Create and start all necessary threads
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
        # Signal all threads to stop and wait for them to finish cleanly
        stop_threads_event.set()
        camera_reader_thread.join()
        lidar_reader_thread.join()
        camera_processor_thread.join()
        lidar_processor_thread.join()
        overlay_thread.join()
        yolo_thread.join()