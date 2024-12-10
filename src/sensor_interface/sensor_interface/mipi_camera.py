import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from SunriseRobotLib.mipi_camera import Mipi_Camera

class MipiCamera(Node):
    def __init__(self, width=320, height=240, debug=False):
        super().__init__('mipi_camera_publisher')
        
        # Initialize the camera
        self.camera = Mipi_Camera(width=width, height=height, debug=debug)
        if not self.camera.isOpened():
            self.get_logger().error("Failed to open MIPI camera.")
            raise RuntimeError("Camera initialization failed")

        # Create a ROS 2 publisher
        self.publisher = self.create_publisher(Image, '/camera/mipi/image_raw', 10)
        self.bridge = CvBridge()
        
        # Set up a timer to capture and publish frames
        self.timer = self.create_timer(0.1, self.publish_frame)  # Publish at 10 Hz

    def publish_frame(self):
        try:
            frame = self.camera.get_frame()  # Capture a frame from the camera
            if frame is not None:
                # Convert the frame to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(image_msg)
                self.get_logger().info("Published an image frame.")
            else:
                self.get_logger().warning("Received empty frame from the camera.")
        except Exception as e:
            self.get_logger().error(f"Error while capturing or publishing frame: {e}")

    def __del__(self):
        # Release the camera resources
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        mipi_camera_node = MipiCamera(width=640, height=480, debug=True)
        rclpy.spin(mipi_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
