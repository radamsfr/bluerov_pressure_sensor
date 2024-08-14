import socket
import time
import random
import pickle
# import ms5837

import serial
from serial.tools.list_ports import comports


class pressure_sensor_reader:
    # def __init__(self):
    #     self.i2c_sensor = ms5837.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 4)
    #     self.initialize_i2c_sensor()

    # def initialize_i2c_sensor(self):
    #     if not self.i2c_sensor.init():
    #         print("Sensor could not be initialized")
    #         return

    #     if not self.i2c_sensor.read():
    #         print("Sensor read failed!")
    #         return

    #     freshwaterDepth = self.i2c_sensor.depth()  # default is freshwater
    #     self.i2c_sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
    #     saltwaterDepth = self.i2c_sensor.depth()
    #     self.i2c_sensor.setFluidDensity(1000)
    #     print(
    #         ("Depth: %.3f m (freshwater)  %.3f m (saltwater)")
    #         % (freshwaterDepth, saltwaterDepth)
    #     )

    # def read_i2c_sensor(self):
    #     if self.i2c_sensor.read():
    #         depth = "%.2f" % (self.i2c_sensor.depth())
    #         pressure = "%.2f" % (self.i2c_sensor.pressure())
    #         temperature = "%.2f" % (self.i2c_sensor.temperature())
    #         altitude = "%.2f" % (self.i2c_sensor.altitude())

    #         sensor_output = "Bar30 Pressure Sensor E\n"
    #         sensor_output += f"Pressure: {pressure} mbar\n"
    #         sensor_output += f"Temperature: {temperature} deg C\n"
    #         sensor_output += f"Depth: {depth} m\n"
    #         sensor_output += f"Altitude: {altitude} m above mean sea level\n"
    #         return sensor_output

    #     else:
    #         return None
    
    ### TESTING LOCALLY
    def generate_sensor(self):
        sensor_measurements = []
        for sensor_name in ["A", "B", "C", "D", "E"]:
            altitude = "%.2f" % (random.random() * 100)
            pressure = "%.2f" % (random.random() * 10000)
            temperature = "%.2f" % (random.random() * 100)
            depth = "%.2f" % (random.random() * -0.1)
            
            sensor_output = f"Bar02 Pressure Sensor {sensor_name} \n"
            sensor_output += f"Pressure: {pressure} mbar \n"
            sensor_output += f"Temperature: {temperature} deg C \n"
            sensor_output += f"Depth: {depth} m \n"
            sensor_output += f"Altitude: {altitude} m above mean sea level \n"

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
            i = 0
            if line[0] == "B":
                while line[0] != "A":
                    if line[0] == "B" and i != 0:
                        line = ser.readline().decode("utf-8")
                        continue
                    else:
                        sensor_output += line
                        line = ser.readline().decode("utf-8")
                        i += 1

                sensor_output += line

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

        c.send(str(5).encode())
        # c.send(str(len(serial_ports) + 1).encode())

        while True:
            response = p.generate_sensor()
            # response = p.get_sensor(serial_ports=serial_ports)

            if not response:
                continue

            # i2c_response = p.read_i2c_sensor()
            # if i2c_response is not None:
            #     response.append(i2c_response)

            print(f"SENT: {response}\n")

            response = pickle.dumps(response)
            c.send(response)
            time.sleep(0.05)

        c.close()


if __name__ == "__main__":
    main()
