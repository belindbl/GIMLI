import queue
import struct
import numpy as np
import open3d as o3d
import ALSLib.TCPClient

class LiDARProcessor:
    """
    Handles LiDAR data collection, queuing, processing, and visualization using Open3D.
    """

    def __init__(self, host="127.0.0.1", port=8881, queue_size=10):
        """
        Initializes the LiDAR processor.

        :param host: IP address of the LiDAR sensor.
        :param port: Port on which LiDAR data is received.
        :param queue_size: Maximum number of LiDAR frames stored before processing.
        """
        self.host = host
        self.port = port
        self.lidar_queue = queue.Queue(maxsize=queue_size)
        self.stop_event = None
        self.client = ALSLib.TCPClient.TCPClient(self.host, self.port, 2)

    def start(self, stop_event):
        """
        Connects to the LiDAR sensor and prepares to receive data.

        :param stop_event: Threading event to stop execution.
        """
        self.stop_event = stop_event
        self.client.connect(2)

    def fetch_data(self):
        """
        Continuously reads LiDAR data and adds it to the queue for processing.
        """
        while not self.stop_event.is_set():
            try:
                lidar_data = self.client.read()
                if lidar_data and not self.lidar_queue.full():
                    self.lidar_queue.put(lidar_data, block=False)
            except Exception as e:
                print(f"LiDAR read error: {e}")

    def process_data(self):
        """
        Processes LiDAR point cloud data and visualizes it in Open3D.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=800, height=600)
        point_cloud = o3d.geometry.PointCloud()

        # Get view control settings
        ctr = vis.get_view_control()
        ctr.set_lookat([0, 0, 300])
        ctr.set_zoom(0.02)
        ctr.set_front([-1, 0, 1])
        ctr.set_up([1, 0, 1])

        point_cloud_added = False  # Tracks whether point cloud was added

        while not self.stop_event.is_set():
            try:
                lidar_data = self.lidar_queue.get(timeout=1)
                if lidar_data:
                    sizeofFloat = 4
                    index = 11
                    pointCloudData = lidar_data[index * sizeofFloat:]
                    point_array = np.frombuffer(pointCloudData, dtype=np.float32)
                    point_array = np.reshape(point_array, (-1, 4))

                    # Extract XYZ points and invert Y-coordinate
                    points = np.stack([point_array[:, 0], -point_array[:, 1], point_array[:, 2]], axis=1)

                    # Assign points to Open3D point cloud
                    point_cloud.points = o3d.utility.Vector3dVector(points)

                    if not point_cloud_added:
                        vis.add_geometry(point_cloud)
                        point_cloud_added = True
                    else:
                        vis.update_geometry(point_cloud)

                    vis.poll_events()
                    vis.update_renderer()
            except queue.Empty:
                pass  # Continue looping if no data is available

        vis.destroy_window()
