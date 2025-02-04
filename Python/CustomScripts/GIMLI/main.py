import threading

from lidar_processor import LiDARProcessor
from camera_processor import CameraProcessor
from simulation_controller import SimulationController
from navigation_controller import NavigationController

# Currently not working, create duplicates of SocketBlockTest4.py to test implementations instead. May need to migrate to UDP entirely

if __name__ == "__main__":
    stop_event = threading.Event()

    # Initialize simulation
    sim = SimulationController()
    sim.initialize()
    cam_host, cam_port = sim.get_camera_info()

    # Initialize processing modules
    lidar = LiDARProcessor()
    camera = CameraProcessor(cam_host, cam_port)
    navigation = NavigationController()

    # Start sensor data collection
    lidar.start(stop_event)
    camera.start(stop_event)

    # Start threads
    lidar_thread = threading.Thread(target=lidar.fetch_data, daemon=True)
    lidar_processing_thread = threading.Thread(target=lidar.process_data, daemon=True)
    camera_thread = threading.Thread(target=camera.process_frames, daemon=True)

    lidar_thread.start()
    lidar_processing_thread.start()
    camera_thread.start()

    try:
        while True:
            pass  # Future navigation logic can be placed here
    except KeyboardInterrupt:
        stop_event.set()
        lidar_thread.join()
        lidar_processing_thread.join()
        camera_thread.join()
