import threading
import struct
import math
import numpy as np
import matplotlib.pyplot as plt
import ALSLib.TCPClient
import ALSLib.ALSClient 
import ALSLib.ALSHelperLidarLibrary as ALSLidar
from ALSLib.ALSHelperFunctionLibrary import get_sensordata_path
"""import os
from datetime import datetime"""


#WORKING 2025-01-08 10:22
#NOTE: The vl16.xml sensor profile has the config: <_spanWidthStep>200.0</_spanWidthStep>, adjusted from the default 100.

HOST = '127.0.0.1'
VL16 = "EgoVehicleSettings\DemoAllSensors.SensorProfile.ini"
OS1_64 = "EgoVehicleSettings\CustomLidar.SensorProfile.ini"

def myMessageHandler(rawMessage):
    str = rawMessage.decode('utf-8')
    # print ('\n---> recieved Raw message: '+str)
    
    cmdList = str.split(" ")
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
    pclFileContent = '# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\n'\
    'SIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT %f %f %f %f %f %f %f\n'\
    'POINTS %d\nDATA ascii\n' % (int(numPoints), posX, posY, posZ, quatW, quatX, quatY, quatZ, int(numPoints))
    
    for p in point_array:
        intensity = 1000
        if not math.isinf(p[3]) and p[3] != 0:
            intensity = int(p[3])
        pclFileContent += '%.5f %.5f %.5f %d\n' % (p[0], -p[1], p[2], intensity) 
    return pclFileContent

fig = plt.figure()
def plot_ply(points):
    # Create or reuse the 3D subplot
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract x, y, z coordinates
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    # Plot points
    ax.scatter(x, y, z, c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Set fixed axis limits
    ax.set_xlim(-8000, 8000)
    ax.set_ylim(-8000, 8000)
    ax.set_zlim(-8000, 8000)

    # Equalize aspect ratio (workaround for 3D)
    def set_equal_aspect(ax):
        extents = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
        centers = np.mean(extents, axis=1)
        ranges = np.max(extents[:, 1] - extents[:, 0])
        for ctr, rng, set_lim in zip(centers, [ranges] * 3, 
                                     [ax.set_xlim3d, ax.set_ylim3d, ax.set_zlim3d]):
            set_lim(ctr - rng / 2, ctr + rng / 2)

    set_equal_aspect(ax)

    # Optional: Freeze limits on resizing (not always needed in 3D)
    def freeze_limits(event):
        ax.set_xlim(-10000, 10000)
        ax.set_ylim(-10000, 10000)
        ax.set_zlim(-10000, 10000)

    fig.canvas.mpl_connect('resize_event', freeze_limits)

    # Show the plot temporarily and clear
    plt.pause(0.07)
    plt.clf()
	#plt.show()

if __name__ == "__main__":
    TestContext.client.connect()
    TestContext.client.request_load_scenario('Default_Scenario')

    ALSLidar.create_sensor_data_folders()

    sensorprofile_path = VL16
    sensor_path_in_file = "Sensors.Sensors.[1]"
    cam_path_in_file = "Sensors.Sensors.[0]"
    overrides = "{file_path};{sensor_path}.StreamToNetwork;True".format(file_path=sensorprofile_path, sensor_path=sensor_path_in_file)
    TestContext.client.request_load_situation_layer_with_overrides('DemoSensorsTest', overrides)

    lazer_proximity_port = 8881
    cam_port = 8880
    client = ALSLib.TCPClient.TCPClient(HOST, lazer_proximity_port, 5)
    client2 = ALSLib.TCPClient.TCPClient(HOST, cam_port, 5)
    client.connect(5)
    client2.connect(5)
    """current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    folder_name = f'/pcl_{current_time}'
    os.makedirs(folder_name, exist_ok=True)"""

    num = 0
    while num < 1:
        data = client.read()
        data2 = client2.read()
        print(data)
        
        #help(data)
        #print(data)
        # Assuming each data point is a 4-byte float
        data_format = 'f'  # 'f' represents a 4-byte float in struct format
        num_points = len(data) // struct.calcsize(data_format)

        # Unpack the byte stream into floats
        data_points = struct.unpack(f'{num_points}f', data)

        #print(data_points)
        print(len(data_points))
        print(len(data2))
        num = num+1
