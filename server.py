import socket
import time
import random
import pickle

import serial
from serial.tools.list_ports import comports


class pressure_sensor_reader:
    ### TESTING LOCALLY
    def generate_sensor(self):
        sensor_measurements = []
        for sensor_name in ["A", "B", "C", "D", "E"]:
            altitude = "%.2f" % (random.random() * 100)
            pressure = "%.2f" % (random.random() * 10000)
            temperature = "%.2f" % (random.random() * 100)
            depth = "%.2f" % (random.random() * -0.1)
            sensor_output = f"Bar02 Pressure Sensor {sensor_name} \nPressure: {pressure} mbar \nTemperature: {temperature} deg C \nDepth: {depth} m \nAltitude: {altitude} m above mean sea level \n"

            sensor_measurements.append(sensor_output)

        return sensor_measurements

    # RETURN LIST OF ACTIVE SERIAL PORTS '/dev/ACM{0-99}'
    def active_serial_ports(self):
        serial_ports = []
        for port in comports():
            if "/dev/ttyACM" in port[0]:
                ser = serial.Serial(port=port[0])
                if ser.is_open:
                    serial_ports.append(ser)
        return serial_ports

    # RETURN LIST OF SENSOR MEASUREMENTS
    def get_sensor(self, serial_ports):
        sensor_measurements = []

        for ser in serial_ports:
            sensor_output = ""

            line = ser.readline().decode("utf-8")
            if line[0] == "B":
                for _ in range(5):
                    sensor_output += line
                    line = ser.readline().decode("utf-8")

            if sensor_output != "":
                sensor_measurements.append(sensor_output)

        return sensor_measurements


def main():
    p = pressure_sensor_reader()
    serial_ports = p.active_serial_ports()

    s = socket.socket()
    print("SOCKET CREATED")

    input_PORT = input("PORT# (press Enter to run default): ")
    PORT = 12345 if input_PORT == "" else int(input_PORT)

    s.bind(("", PORT))
    print(f"socket binded to {PORT}")

    s.listen(5)
    print("socket is listening")

    while True:
        c, addr = s.accept()
        print("Got connection from", addr)
        # c.send(str(5).encode())
        c.send(str(len(serial_ports)).encode())

        while True:
            # response = p.generate_sensor()
            response = p.get_sensor(serial_ports=serial_ports)

            if not response:
                continue

            print(f"SENT: {response}\n")

            response = pickle.dumps(response)
            c.send(response)
            time.sleep(0.05)

        c.close()


if __name__ == "__main__":
    main()
