import os
import threading, struct, math, time, json
import numpy as np
import matplotlib.pyplot as plt

import ALSLib.TCPClient
import ALSLib.ALSClient 
import ALSLib.ALSHelperLidarLibrary as ALSLidar
from ALSLib.ALSHelperFunctionLibrary import get_sensordata_path

""" WORKING SCRIPT!!!!!"""

###############################################################################
# This demo shows how to collect LiDAR data and store it as PCD.
#
# A premade SensorProfile is used where sensors are pre-configured. We
# enable only the LiDAR sensor (Sensors.[1]) via overrides.
###############################################################################

HOST = '127.0.0.1'

def myMessageHandler(rawMessage):
    str_msg = rawMessage.decode('utf-8')
    cmdList = str_msg.split(" ")
    if cmdList[0].startswith("EndCondition"):
        TestContext.client.request_destroy_situation()
        TestContext.lock.acquire()
        TestContext.testEnded = True
        print("setting TestEnded")
        TestContext.lock.release()

class TestContext:
    lock = threading.Lock()
    testEnded = False
    simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((HOST, simulation_control_port), myMessageHandler)

def SerializeToPCLFileContent(numPoints, posX, posY, posZ, quatW, quatX, quatY, quatZ, point_array):
    """
    Serializes an array of point data into ASCII PCD format.
    """
    pclFileContent = (
        '# .PCD v.7 - Point Cloud Data file format\n'
        'VERSION .7\n'
        'FIELDS x y z rgb\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F U\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {int(numPoints)}\n'
        'HEIGHT 1\n'
        f'VIEWPOINT {posX} {posY} {posZ} {quatW} {quatX} {quatY} {quatZ}\n'
        f'POINTS {int(numPoints)}\n'
        'DATA ascii\n'
    )
    for p in point_array:
        # For demonstration, we use the fourth value as intensity
        intensity = 1000
        if not math.isinf(p[3]):
            intensity = int(p[3])
        pclFileContent += f'{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {intensity}\n'
    return pclFileContent

fig = plt.figure()

def plot_ply(points):
    """
    Plots the point cloud in a 3D matplotlib window.
    """
    ax = fig.add_subplot(111, projection='3d')
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    ax.scatter(x, y, z, marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    ax.set_xlim(-10000, 10000)
    ax.set_ylim(-10000, 10000)
    ax.set_zlim(-10000, 10000)

    def set_equal_aspect(ax):
        extents = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
        centers = np.mean(extents, axis=1)
        ranges = np.max(extents[:, 1] - extents[:, 0])
        for ctr, rng, set_lim in zip(centers, [ranges]*3,
                                     [ax.set_xlim3d, ax.set_ylim3d, ax.set_zlim3d]):
            set_lim(ctr - rng/2, ctr + rng/2)

    set_equal_aspect(ax)

    # Freeze limits on resizing
    def freeze_limits(event):
        ax.set_xlim(-10000, 10000)
        ax.set_ylim(-10000, 10000)
        ax.set_zlim(-10000, 10000)

    fig.canvas.mpl_connect('resize_event', freeze_limits)

    # Show the plot briefly, then clear so next cloud can be drawn
    plt.pause(0.07)
    plt.clf()

if __name__ == "__main__":
    # 1) Connect and load scenario
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Default_Scenario')

    # Create base SensorData folders (internal to ALSLib)
    ALSLidar.create_sensor_data_folders()

    # 2) Set overrides for LiDAR sensor (Sensors.[1]) so it streams to network
    sensorprofile_path = "EgoVehicleSettings\\ABoat.SensorProfile.ini"
    sensor_path_in_file = "Sensors.Sensors.[1]"
    overrides = f"{sensorprofile_path};{sensor_path_in_file}.StreamToNetwork;True"
    TestContext.client.request_load_situation_layer_with_overrides('AboatICT', overrides)
    
    # Pause to ensure scenario + layers loaded
    time.sleep(1)

    # 3) Retrieve LiDAR sensor info
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)

    # Find the host and port of the LiDAR sensor
    for x in parsed_json["sensors"]:
        if x["path"] == "Sensors.[1]":
            sensor = x
            break
    lidar_port = sensor["sensor_port"]
    host = sensor["sensor_ip"]
    
    print(f"Connecting sensor socket to {host} {lidar_port}")
    client = ALSLib.TCPClient.TCPClient(HOST, lidar_port, 5)
    client.connect(5)

    # 4) Ensure "SensorData/pcd" directory exists
    save_directory = "SensorData/pcd"
    os.makedirs(save_directory, exist_ok=True)

    imageNum = 0
    while True:
        data = client.read()
        sizeofFloat = 4
        
        # The first 11 floats in the stream: position/orientation/numPoints/time
        index = 11
        posX, posY, posZ, quatW, quatX, quatY, quatZ, \
        numPoints, timeStart, timeEnd, numberOfBeams = \
            struct.unpack('<fffffffffff', data[0 : index * sizeofFloat])

        print('Lidar position: X=%.2f Y=%.2f Z=%.2f' % (posX, posY, posZ))
        
        # The rest are the points: 4 floats per point => [x, y, z, intensity]
        pointCloudData = data[index * sizeofFloat : ]
        point_array = np.frombuffer(pointCloudData, dtype=np.float32)
        point_array = np.reshape(point_array, (-1, 4))

        # Optional: visualize the point cloud
        plot_ply(point_array)

        # Serialize to ASCII PCD
        pclFileContent = SerializeToPCLFileContent(
            numPoints, posX, posY, posZ, quatW, quatX, quatY, quatZ, point_array
        )

        # 5) Save to "SensorData/pcd" directory
        filename = os.path.join(save_directory, f"pcl_{imageNum}.pcd")
        with open(filename, 'w') as fileObject:
            fileObject.write(pclFileContent)

        imageNum += 1
