# GIMLI Installation and User Guide

GIMLI (Guided Integrated Maritime Logistics and Intelligence) is a software solution developed to navigation support for a semi-autonomous vessel. This guide outlines the necessary components, installation steps, configuration details, and instructions for running the solution in a simulated environment.

---

## 1. Introduction

### 1.1 Overview
- **Purpose:**  
  GIMLI integrates LIDAR and camera sensor data to deliver real-time obstacle detection and classification for safe maritime navigation.

### 1.2 Key Objectives
- Implement reliable obstacle detection via sensor fusion.
- Synchronise real-time image and LIDAR data.
- Integrate and validate the system using the **AILiveSim** simulator.

---

## 2. System Components

### 2.1 Simulator: AILiveSim
- **Description:**  
  AILiveSim provides a realistic maritime simulation environment built on Unreal Engine 4.
- **License Requirement:**  
  A valid product key is required. Contact [AILiveSim](https://www.ailivesim.com/) for licensing details.
- **Documentation:**  
  Follow the instructions in the [AILiveSim documentation](https://portal.ailivesim.com/documentation).

### 2.2 Software Libraries
- **Python Dependencies:**  
  All required Python libraries are listed in the `requirements.txt` file.
- **Key Modules:**  
  - **ALSLib:** For sensor data acquisition and control.
  - **Open3D & OpenCV:** For 3D visualisation and image processing.
  - **PyTorch & YOLOv5:** For object detection within the camera feed.
  - **Scikit-learn:** For DBSCAN clustering.

### 2.3 Configuration Files
- **Simulation Configuration:**  
  Underlying config files (e.g., sensor settings, extrinsic/intrinsic parameters) must be set up as required.
- **User Modifications:**  
  Adjust file paths and sensor configurations within these files to match your environment.

### 2.4 Final Code
- **Project Repository:**  
  Contains the complete implementation including:
  - Data acquisition threads.
  - Sensor fusion and calibration modules.
  - YOLO inference integration.
  - Visualisation components.
- **Code Location:**  
  The code is provided within this repository along with this guide.

---

## 3. Setup and Installation

### 3.1 Prerequisites
- **Hardware:**  
  A system capable of running the simulator and process real-time sensor data.
- **Operating System:**  
  Any OS that supports Python and the AILiveSim simulator.
- **Software Requirements:**  
  - Python 3.11.9 or newer.

### 3.2 Installing AILiveSim
1. **Obtain License:**  
   Contact [AILiveSim](https://www.ailivesim.com/) to receive a valid product key.
2. **Download & Install:**  
   Follow the AILiveSim documentation to install the simulator on your machine.
3. **Activate Simulator:**  
   Place the license key in the approapriate folder and start the AILiveSim executable.

### 3.3 Setting Up the Python Environment
1. **Clone the Repository:**  
   ```bash
   git clone https://github.com/belindbl/gimli.git
   cd gimli
   ```
2. **Create a virtual environment:**
   ```bash
   python -m venv venv
   ```
   Then activate the virtual environment:
   ```
   venv\Scripts\activate
   ```
3. **Install dependencies:**
      ```bash
   pip install -r requirements.txt
   ```
   This ensures all necessary dependencies are installed.

### 3.4 Configuring Simulation Files
- **Edit Config Files:**   
   Locate and adjust the simulation configuration files (e.g., sensor settings, extrinsic/intrinsic parameters) as necessary.
- **Verify Settings:**  
   Ensure that the sensor configuration matches the setup described in the project.
---
## 4. Executing the Final Code

### 4.1 Starting the Simulator
- Launch the AILiveSim executable.
- Ensure the simulator is running and that the desired  scenario (e.g., 'DemoAllSensors') is loaded.

### 4.2 Launching GIMLI

1. **Run the Main Script:**    
Execute the final code to start the sensor fusion and visualisation processes:  
```bash
python \GIMLI\Python\CustomScripts\GIMLI\RTDP9.py # RTDP9 is the current main script
```

2. **Monitoring Output:**   
- A window should display the camera feed overlaid with the projected LiDAR points and annotation texts.
- A separate window will visualise the point cloud in real time.
3. **User Interaction:**    
Use the interface to monitor real-time data fusion and observe obstacle detection and collision avoidance overlays.

