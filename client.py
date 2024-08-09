import socket
import re
import copy
import pickle
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import FluidPressure


class SensorBridge(Node):
    def __init__(self, sensor_count):
        super().__init__("sensor_bridge")
        self.publishers_ = []
        for i in range(sensor_count):
            publisher_name = "/bluerov2/pressure/sensor" + str(i)
            self.publishers_.append(
                self.create_publisher(FluidPressure, publisher_name, 30)
            )

    def callback(self, sensor_measurements):
        for key, value in sensor_measurements.items():
            self.header_time = self.get_clock().now().to_msg()
            self.frame_id = "base_link"
            self.header = Header(stamp=self.header_time, frame_id=self.frame_id)

            # Absolute pressure reading in Pascals
            if not value["Pressure"]:
                continue
            else:
                # Publish most recent pressure value
                self.fluid_pressure = float(value["Pressure"][-1])

            # 0 is interpreted as variance unknown
            variance = 0
            self.variance = float(variance)

            msg = FluidPressure(
                header=self.header,
                fluid_pressure=self.fluid_pressure,
                variance=self.variance,
            )
            self.publishers_[key].publish(msg)

            self.get_logger().info(
                f"Published: pressure: {msg.fluid_pressure}, variance: {msg.variance}"
            )


def main():
    s = socket.socket()

    ### LOCAL DEVICE HOST
    HOST = socket.gethostbyname(socket.gethostname())
    # HOST = "192.168.2.11"
    print("HOST:", HOST)

    input_PORT = input("PORT# (press Enter to run default): ")
    PORT = 12345 if input_PORT == "" else int(input_PORT)

    s.connect((HOST, PORT))
    sensor_count = int(s.recv(64).decode())

    # PRESSURE SENSOR DICT
    pressure_sensors = {}
    pressure_sensor = {
        "SensorName": "",
        "Temperature": [],
        "Depth": [],
        "Pressure": [],
        "Altitude": [],
    }
    for i in range(sensor_count):
        pressure_sensors.update({i: copy.deepcopy(pressure_sensor)})

    rclpy.init(args=None)
    sensor_bridge = SensorBridge(sensor_count=sensor_count)

    RECORDING = True
    while RECORDING:
        sensor_output = s.recv(3096)

        if not sensor_output:
            print("NO DATA COLLECTED, GOODBYE")
            break
        sensor_output = pickle.loads(sensor_output)

        print("RECIEVED:")
        print(sensor_output)
        print()
        print(len(pressure_sensors))

        for sensor_measurement in sensor_output:
            # COLLECT MEASUREMENTS
            collect_sensor_name = re.search(
                "Bar02 Pressure Sensor ([A-Z])", sensor_measurement
            ).group(1)
            collect_pressure = re.search(
                "Pressure: ([0-9.-]+)", sensor_measurement
            ).group(1)
            collect_temperature = re.search(
                "Temperature: ([0-9.-]+)", sensor_measurement
            ).group(1)
            collect_depth = re.search("Depth: (-?[0-9.-]+)", sensor_measurement).group(
                1
            )
            collect_altitude = re.search(
                "Altitude: ([0-9.-]+)", sensor_measurement
            ).group(1)

            # SORT INTO DICT
            idx = ord(collect_sensor_name[0].upper()) - 65
            pressure_sensors[idx]["SensorName"] = collect_sensor_name
            pressure_sensors[idx]["Pressure"].append(collect_pressure)
            pressure_sensors[idx]["Temperature"].append(collect_temperature)
            pressure_sensors[idx]["Depth"].append(collect_depth)
            pressure_sensors[idx]["Altitude"].append(collect_altitude)

        sensor_bridge.callback(pressure_sensors)

    s.close()
