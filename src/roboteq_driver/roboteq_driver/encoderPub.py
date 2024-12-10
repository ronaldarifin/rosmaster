import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.publisher = self.create_publisher(Int32, 'encoder_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_encoder)

    def read_encoder(self):
        self.serial_port.write(b'?C 1\r')  # Query encoder counts for Channel 1
        response = self.serial_port.readline().decode().strip()
        if response.isdigit():
            msg = Int32()
            msg.data = int(response)
            self.publisher.publish(msg)

rclpy.init()
node = EncoderReader()
rclpy.spin(node)
