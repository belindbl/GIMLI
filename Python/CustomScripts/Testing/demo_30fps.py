import threading
import time
import json
import struct
import numpy as np
import open3d as o3d
import torch
import cv2
import os
import queue
from sklearn.cluster import DBSCAN

import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg

######################################################
# CONFIG: Folders for camera images & LiDAR PCD files
######################################################
IMAGE_FOLDER = "./SensorData/ICTimgs"
PCD_FOLDER = "./SensorData/pcd"

# Global stop signal for all threads
stop_threads_event = threading.Event()

# Shared data (protected by lock)
sensor_data = {'camera': None, 'lidar': None}
data_lock = threading.Lock()

# Single-frame queue for YOLO (camera frames)
frame_queue = queue.Queue(maxsize=1)

# YOLO model
MixedWeights = "E:/Team1Docs/yolov5/weights_mixed/best.pt"

# YOLO output image/bboxes (shared)
yolo_boxes = []
yolo_img_lock = threading.Lock()
yolo_output_img = None

############################################################
# Optional: If you need a TestContext or scenario load stub
############################################################
class TestContext:
    lock = threading.Lock()
    testEnded = False

######################################################
# Image enhancement to help YOLO accuracy (CLAHE)
######################################################
def apply_clahe(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(3,3))
    cl = clahe.apply(l)
    merged = cv2.merge((cl, a, b))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

######################################################
# YOLO Inference
######################################################
def process_frame(image, model):
    # Convert to an RGB image if needed
    if len(image.shape) == 2:  # grayscale
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.shape[2] == 4:  # RGBA
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    results = model(image)
    processed_image = results.render()[0]
    # YOLO returns BGR, so convert it if needed
    return cv2.cvtColor(processed_image, cv2.COLOR_RGB2RGBA), results

def yolo_inference_thread():
    """ Continuously pull frames from frame_queue, run YOLO, and store the output image. """
    global yolo_boxes, yolo_output_img
    print("Loading YOLOv5 model in YOLO thread...", flush=True)
    try:
        # Load YOLO model from ultralytics
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=MixedWeights,
                               force_reload=False, source='github')
        # Dummy inference to force model init
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        _, _ = process_frame(dummy_image, model)
        model.conf = 0.4
        model.iou = 0.5
        if torch.cuda.is_available():
            model = model.to('cuda')
            print("YOLOv5 model loaded on GPU.", flush=True)
        else:
            print("CUDA not available; YOLOv5 model on CPU.", flush=True)
    except Exception as e:
        print("Error loading YOLOv5 model:", e, flush=True)
        return

    while not stop_threads_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        # Enhance then run YOLO
        frame = apply_clahe(frame)
        processed_img, results = process_frame(frame, model)

        # Extract YOLO bounding boxes
        try:
            boxes = results.xyxy[0].cpu().numpy()
        except Exception:
            boxes = []

        # Store output image for overlay
        with yolo_img_lock:
            yolo_output_img = processed_img.copy()

######################################################
# Thread: Overlay LiDAR on camera image
######################################################
def overlay_lidar_on_camera_thread():
    """
    Fetches the latest camera image and LiDAR data from sensor_data,
    projects the 3D LiDAR points onto the 2D image, and overlays them.
    Clusters points with DBSCAN and uses label-based coloring.
    Also writes each resulting overlay frame to a 30 FPS video.
    """
    # Camera intrinsic calibration
    image_width, image_height = 720, 480
    fov_angle = 90
    focal_length = image_width / (2 * np.tan(np.radians(fov_angle) / 2))
    cx, cy = image_width / 2, image_height / 2
    K = np.array([[focal_length, 0, cx],
                  [0, focal_length, cy],
                  [0, 0, 1]])
    
    # Example extrinsic: world -> camera
    t_cam = np.array([105.941, -0.13, 118.285])
    R_wc = np.array([
        [ 0,  1,  0],
        [ 0,  0, -1],
        [ 1,  0,  0]
    ])
    T = np.eye(4)
    T[:3, :3] = R_wc
    T[:3, 3] = -R_wc @ t_cam

    # --- NEW: Setup a VideoWriter to record at 30 FPS ---
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')    # or 'XVID', 'avc1', etc.
    video_writer = cv2.VideoWriter('overlay_output.mp4',
                                   fourcc,
                                   30.0,  # 30 FPS
                                   (image_width, image_height))
    if not video_writer.isOpened():
        print("Warning: Could not open VideoWriter to save overlay video.")

    try:
        while not stop_threads_event.is_set():
            with data_lock:
                cam_data = sensor_data.get('camera')
                lidar_data = sensor_data.get('lidar')  # Nx4 = [x, y, z, intensity]
            if cam_data is None or lidar_data is None:
                time.sleep(0.02)
                continue
            
            try:
                # Use YOLO output image if available; otherwise, just use the raw image
                with yolo_img_lock:
                    if yolo_output_img is not None:
                        img = yolo_output_img.copy()
                    else:
                        # If there's no YOLO image yet, assume cam_data is already an image
                        img = cam_data.copy()

                # Resize to match the set resolution
                img = cv2.resize(img, (image_width, image_height), interpolation=cv2.INTER_LINEAR)

                # LiDAR data is Nx4 [x, y, z, intensity]
                point_array = lidar_data
                points = point_array[:, :3]
                intensity = point_array[:, 3]

                # Project 3D -> camera
                num_points = points.shape[0]
                points_hom = np.hstack((points, np.ones((num_points, 1))))
                points_cam_hom = (T @ points_hom.T).T
                points_cam = points_cam_hom[:, :3]

                # Keep only points with positive Z in camera frame
                valid_mask = points_cam[:, 2] > 0
                points_cam = points_cam[valid_mask]
                valid_points_world = points[valid_mask]
                intensity = intensity[valid_mask]
                
                if points_cam.shape[0] == 0:
                    time.sleep(0.05)
                    continue

                # Clustering in world coords
                dbscan = DBSCAN(eps=200, min_samples=4)
                labels = dbscan.fit_predict(valid_points_world)

                # Intrinsic projection
                proj = (K @ points_cam.T).T
                proj[:, 0] /= proj[:, 2]
                proj[:, 1] /= proj[:, 2]
                proj_points = proj[:, :2]

                # Some cluster colors
                custom_colors = {
                    0: (0, 0, 255),    # Red
                    1: (0, 255, 0),    # Green
                    2: (255, 0, 0),    # Blue
                    3: (255, 255, 0),  # Yellow
                    4: (255, 0, 255),  # Magenta
                    5: (0, 255, 255),  # Cyan
                    -1: (100, 100, 100)  # Noise
                }

                # Draw each valid point
                for i, pt in enumerate(proj_points):
                    x, y = int(round(pt[0])), int(round(pt[1]))
                    if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                        cluster_label = labels[i]
                        color = custom_colors.get(cluster_label, (255, 255, 255))
                        cv2.circle(img, (x, y), 2, color, thickness=-1)

                # Display the overlay in a window
                ALSImg.JustDisplay(img)

                # --- NEW: Write the overlayed frame to the video ---
                if video_writer.isOpened():
                    # Ensure the frame has the correct shape (720x480)
                    # If shapes mismatch, we might need to re-resize or skip writing
                    if img.shape[1] == image_width and img.shape[0] == image_height:
                        video_writer.write(img)
                    else:
                        # Optionally handle mismatch or re-resize:
                        resized = cv2.resize(img, (image_width, image_height))
                        video_writer.write(resized)

            except Exception as e:
                print("Overlay LiDAR on camera error:", e)

            time.sleep(0.05)

    finally:
        # --- NEW: Release the writer when done ---
        video_writer.release()
        print("Overlay video writer stopped and file saved.")

######################################################
# Thread: Camera (Image) File Reader
######################################################
def camera_file_reader():
    """
    Reads images from a folder sequentially and stores them in sensor_data['camera'].
    You can loop forever or stop when out of images.
    """
    image_files = sorted([
        f for f in os.listdir(IMAGE_FOLDER)
        if any(f.lower().endswith(ext) for ext in ['.png','.jpg','.jpeg','.bmp'])
    ])
    idx = 0
    num_files = len(image_files)
    while not stop_threads_event.is_set():
        if num_files == 0:
            print("No images found in", IMAGE_FOLDER)
            time.sleep(1)
            continue
        # Read next image
        filename = image_files[idx]
        img_path = os.path.join(IMAGE_FOLDER, filename)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Could not read image {img_path}")
            time.sleep(0.2)
            continue
        
        # Update shared data
        with data_lock:
            sensor_data['camera'] = img
        
        idx = (idx + 1) % num_files  # loop around or increment to stop at end
        time.sleep(0.1)  # small delay to mimic real-time

######################################################
# Thread: Camera Processing 
# (If you want a separate step for putting frames in YOLO queue)
######################################################
def camera_processing_thread():
    """Extract frames from sensor_data['camera'] and push them to the YOLO queue."""
    while not stop_threads_event.is_set():
        with data_lock:
            cam_data = sensor_data.get('camera')
        if cam_data is not None:
            try:
                # If the queue is full, discard the oldest
                if frame_queue.full():
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                # Put the current camera image in the queue
                frame_queue.put(cam_data.copy())
            except Exception as e:
                print("Camera processing error:", e)
        time.sleep(0.02)

######################################################
# Thread: LiDAR (PCD) File Reader
######################################################
def lidar_file_reader():
    """
    Reads .pcd files from a folder in sequence.
    Converts them to Nx4 arrays [x, y, z, intensity=0].
    Stores them in sensor_data['lidar'].
    """
    pcd_files = sorted([
        f for f in os.listdir(PCD_FOLDER)
        if f.lower().endswith('.pcd')
    ])
    idx = 0
    num_files = len(pcd_files)
    while not stop_threads_event.is_set():
        if num_files == 0:
            print("No PCD files found in", PCD_FOLDER)
            time.sleep(1)
            continue

        filename = pcd_files[idx]
        pcd_path = os.path.join(PCD_FOLDER, filename)

        try:
            pcd = o3d.io.read_point_cloud(pcd_path)
            pts = np.asarray(pcd.points, dtype=np.float32)
            if pts.shape[0] == 0:
                # No points
                time.sleep(0.2)
                continue
            # If no intensity is stored, just set to zero
            # If you have pcd.colors or pcd.intensities, adapt accordingly
            lidar_array = np.zeros((pts.shape[0], 4), dtype=np.float32)
            lidar_array[:, :3] = pts
            lidar_array[:, 3] = 0.0  # default intensity
        except Exception as e:
            print(f"Error reading {pcd_path}: {e}")
            time.sleep(0.2)
            continue
        
        # Store in sensor_data
        with data_lock:
            sensor_data['lidar'] = lidar_array
        
        idx = (idx + 1) % num_files  # loop or go to next
        time.sleep(0.2)

######################################################
# Thread: LiDAR Processing (3D Visualization)
######################################################
def setup_visualizer():
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=900, height=700, window_name="PCD Visualization")
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
    """Mask out points that lie within a bounding box (e.g. boat)."""
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

def update_persistent_ids(new_centroids, tracked_clusters, next_cluster_id, max_tracking_distance):
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
    point_colors = np.zeros((points.shape[0], 3))
    # By default, non-filtered points get a color
    point_colors[~mask] = [0, 0, 0.5]
    filtered_indices = np.where(mask)[0]
    for i, idx in enumerate(filtered_indices):
        db_label = labels[i] if i < len(labels) else -1
        if db_label == -1:
            point_colors[idx] = [0, 0, 0]  # noise
        else:
            pid = persistent_ids.get(db_label, db_label)
            if pid in custom_colors:
                point_colors[idx] = custom_colors[pid]
            else:
                # random color if none assigned
                np.random.seed(pid)
                point_colors[idx] = np.random.rand(3)
    return point_colors

def lidar_processing_thread():
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
    # Example bounding box for "boat" area
    min_coords = np.array([-206, -159, 162])
    max_coords = np.array([513, 159, 321])

    while not stop_threads_event.is_set():
        with data_lock:
            lidar_data = sensor_data.get('lidar')  # Nx4
        if lidar_data is not None:
            try:
                # Nx4 => first 3 columns are (x, y, z)
                points = lidar_data[:, :3]

                # Filter boat region
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
                print("LiDAR processing error:", e)
        time.sleep(0.1)

    vis.destroy_window()

######################################################
# MAIN
######################################################
if __name__ == "__main__":
    # If you no longer need to connect to ALS or load scenarios, you can remove these lines:
    # context = TestContext()
    # context.testEnded = False
    # (Or keep them if your environment still requires scenario loads.)
    
    print("Starting camera and LiDAR file-based reading...")

    # Start our data-reading threads (reading from disk)
    camera_reader_thread = threading.Thread(target=camera_file_reader, daemon=True)
    lidar_reader_thread = threading.Thread(target=lidar_file_reader, daemon=True)

    # Start threads for processing
    camera_processor_thread = threading.Thread(target=camera_processing_thread, daemon=True)
    lidar_processor_thread = threading.Thread(target=lidar_processing_thread, daemon=True)
    yolo_thread = threading.Thread(target=yolo_inference_thread, daemon=True)
    overlay_thread = threading.Thread(target=overlay_lidar_on_camera_thread, daemon=True)

    # Launch them
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
        yolo_thread.join()
        overlay_thread.join()
        print("Stopped all threads.")
