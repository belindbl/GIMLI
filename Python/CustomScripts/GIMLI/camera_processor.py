import ALSLib.TCPClient
import ALSLib.ALSHelperFunctionLibrary as ALSFunc
import ALSLib.ALSHelperImageLibrary as ALSImg

class CameraProcessor:
    """
    Handles camera data collection and visualization.
    Future expansion: Integrate YOLO processing for object detection.
    """

    def __init__(self, host, port):
        """
        Initializes the camera processor.

        :param host: IP address of the camera sensor.
        :param port: Port number of the camera sensor.
        """
        self.host = host
        self.port = port
        self.client = ALSLib.TCPClient.TCPClient(self.host, self.port, 2)
        self.stop_event = None

    def start(self, stop_event):
        """
        Connects to the camera sensor and prepares to receive frames.

        :param stop_event: Threading event to stop execution.
        """
        self.stop_event = stop_event
        self.client.connect(2)

    def process_frames(self):
        """
        Continuously reads and displays camera frames.
        """
        while not self.stop_event.is_set():
            try:
                cam_data = self.client.read()
                if cam_data:
                    index = 0
                    img, index, width, height = ALSFunc.ReadImage_Stream(cam_data, index)
                    ALSImg.JustDisplay(img)
            except Exception as e:
                print(f"Camera read error: {e}")
