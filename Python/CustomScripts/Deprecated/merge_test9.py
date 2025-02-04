def lidar_thread():
    lidar_pcd_count = 0
    client = ALSLib.UDPClient.UDPClient(HOST, 8881)
    client.connect(5)

    while lidar_pcd_count < MAX_LIDAR_PCDS and not stop_threads_event.is_set():
        try:
            data = client.readSimple()
            if data is None:
                print("No data received from LiDAR")
                continue  # skip this iteration if no data

            sizeofFloat = 4
            header_format = '<fffffffffff'
            header_size = struct.calcsize(header_format)
            header = struct.unpack(header_format, data[:header_size])
            posX, posY, posZ, quatW, quatX, quatY, quatZ, numPoints, timeStart, timeEnd, numberOfBeams = header

            pointCloudData = data[header_size:]
            point_array = np.frombuffer(pointCloudData, dtype=np.float32)
            point_array = np.reshape(point_array, (-1, 4))

            pclFileContent = SerializeToPCLFileContent(numPoints, posX, posY, posZ,
                                                        quatW, quatX, quatY, quatZ, point_array)
            filename = get_sensordata_path(f"/pcl/pcl{lidar_pcd_count}.pcd")
            with open(filename, 'w') as fileObject:
                fileObject.write(pclFileContent)
            print(f"Point cloud {lidar_pcd_count} saved to {filename}")
            lidar_pcd_count += 1

            time.sleep(0.005)  # Adjust sleep time if necessary
        except Exception as e:
            print(f"Error in lidar_thread: {e}")

    stop_threads_event.set()

def camera_thread():
    CAMERA_SAVE_PATH = r"E:\AILiveSim_1_9_7\SensorData\imgs"
    if not os.path.exists(CAMERA_SAVE_PATH):
        os.makedirs(CAMERA_SAVE_PATH)

    time.sleep(0.1)  # Allow time for sensor initialization
    sensorlist = TestContext.client.get_sensor_list()
    parsed_json = json.loads(sensorlist)
    for x in parsed_json['sensors']:
        if x['path'] == 'Sensors.[0]':
            sensor = x
            break
    camera_port = sensor['sensor_port']
    host = sensor['sensor_ip']
    print("Connecting sensor socket to " + host + " " + str(camera_port))
    client = ALSLib.UDPClient.UDPClient(host, camera_port)
    client.connect(5)

    image_num = 0
    while image_num < IMAGE_NUM and not stop_threads_event.is_set():
        try:
            data = client.readSimple()
            if data is None:
                print("No data received from Camera")
                continue  # skip if no data

            index = 0
            image_num += 1
            img, index, width, height = ReadImage_Stream(data, index)
            ALSImg.JustDisplay(img)
            image_filename = os.path.join(CAMERA_SAVE_PATH, f"image_{image_num}.jpg")
            cv2.imwrite(image_filename, img)
            print(f"Image {image_num} saved to {image_filename}")
            time.sleep(0.005)
        except Exception as e:
            print(f"Error in camera_thread: {e}")

    stop_threads_event.set()
