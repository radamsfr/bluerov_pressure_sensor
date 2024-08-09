import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.subscription = self.create_subscription(
            FluidPressure, "/bluerov2/pressure/sensor0", self.listener_callback, 30
        )

    def listener_callback(self, msg):
        # print('FluidPressure:', msg)
        self.get_logger().info(
            f"Collected: pressure: {msg.fluid_pressure}, variance: {msg.variance}"
        )


def main():
    rclpy.init(args=None)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
