import rclpy
from rclpy.node import Node

class DynamicParameterNode(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare a dynamic parameter with a string array
        self.declare_parameter('devices_positions', [
            "A0: 0.0, -1.0, 0.0",
            "A1: 1.0, 0.0, 0.0",
            "A2: 1.0, 0.0, 0.0",
            "A3: 1.0, 0.0, 0.0",
            "A4: 1.0, 0.0, 0.0",
            "A5: 1.0, 0.0, 0.0",
            "A6: 1.0, 0.0, 0.0",
            "A7: 0.0, 1.0, 0.0"
        ])

        # Accessing the parameter
        devices_positions = self.get_parameter('devices_positions').get_parameter_value().string_array_value
        # self.get_logger().info(f"Devices positions: {devices_positions}")
        self.get_logger().info(f"Devices positions: {devices_positions[7]}")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicParameterNode()

    # Spin to keep the node alive
    rclpy.spin(node)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()