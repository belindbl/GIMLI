import time
import json
import ALSLib.ALSClient

class SimulationController:
    """
    Manages simulation environment initialization and scenario control.
    """

    def __init__(self, host="127.0.0.1", control_port=9000):
        """
        Initializes the simulation controller.

        :param host: IP address of the simulation environment.
        :param control_port: Port number to communicate with the simulator.
        """
        self.host = host
        self.control_port = control_port
        self.client = ALSLib.ALSClient.Client((self.host, self.control_port), lambda msg: None)

    def initialize(self):
        """
        Connects to the simulation and loads the scenario.
        """
        self.client.connect()
        self.client.request_load_scenario('Aboat')

    def get_camera_info(self):
        """
        Retrieves camera sensor details from the simulation.

        :return: Tuple (camera IP, camera port)
        """
        time.sleep(1)  # Allow time for the simulation to initialize sensors
        sensor_list = self.client.get_sensor_list()
        parsed_json = json.loads(sensor_list)

        for sensor in parsed_json['sensors']:
            if sensor['path'] == 'Sensors.[0]':
                return sensor['sensor_ip'], sensor['sensor_port']
        return None, None
