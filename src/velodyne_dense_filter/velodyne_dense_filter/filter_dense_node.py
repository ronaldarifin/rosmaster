import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  
import numpy as np

class VelodyneDenseFilterNode(Node):
    def __init__(self):
        super().__init__('velodyne_dense_filter_node')

        # Declare parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_dense_points')

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Subscribe to Velodyne point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.process_point_cloud,
            10
        )

        # Publisher for dense point cloud
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing dense point clouds to {self.output_topic}')

    def process_point_cloud(self, msg):
        # Read points including the "time" field
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=False))

        # Define the structured dtype for the point cloud
        dtype = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)
        ]

        # Convert to a structured NumPy array
        points_array = np.array(points, dtype=dtype)

        # Filter out invalid points (NaN or infinity in any field)
        valid_points = points_array[
            ~np.isnan(points_array['x']) &
            ~np.isnan(points_array['y']) &
            ~np.isnan(points_array['z']) &
            ~np.isnan(points_array['time'])
        ]

        # Create a new PointCloud2 message with structured data
        header = msg.header
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='ring', offset=16, datatype=pc2.PointField.UINT16, count=1),
            pc2.PointField(name='time', offset=18, datatype=pc2.PointField.FLOAT32, count=1),
        ]

        # Use structured NumPy array to create PointCloud2
        filtered_msg = pc2.create_cloud(header, fields, valid_points)
        filtered_msg.is_dense = True  # Mark as dense

        # Publish the filtered point cloud
        self.publisher.publish(filtered_msg)
        # self.get_logger().info(f'Published dense point cloud with {len(valid_points)} points.')





def main(args=None):
    rclpy.init(args=args)
    node = VelodyneDenseFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
