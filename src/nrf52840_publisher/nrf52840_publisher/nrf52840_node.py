import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu_raw', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def read_imu_data(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        try:
            # Split the line into accelerometer and gyroscope parts
            accel_part, gyro_part = line.split('\t')
            
            # Parse accelerometer values
            accel_values = accel_part.replace('Accel:', '').strip().split(',')
            ax = float(accel_values[0])
            ay = float(accel_values[1])
            az = float(accel_values[2])
            
            # Parse gyroscope values
            gyro_values = gyro_part.replace('Gyro:', '').strip().split(',')
            gx = float(gyro_values[0])
            gy = float(gyro_values[1])
            gz = float(gyro_values[2])
            
            # Create and fill IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Set linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Set angular velocity (rad/s)
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Publish the message
            self.publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error parsing IMU data: {e}")

    def timer_callback(self):
        self.read_imu_data()

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    timer_period = 0.01  # 10 ms
    node.create_timer(timer_period, node.timer_callback)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


