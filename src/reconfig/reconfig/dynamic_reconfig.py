import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterReconfigureNode(Node):
    def __init__(self):
        super().__init__('parameter_reconfigure_node')

        # Initialize parameters of different types
        self.declare_parameter('int_param', 10)
        self.declare_parameter('float_param', 3.14)
        self.declare_parameter('string_param', 'Hello ROS2')
        self.declare_parameter('bool_param', True)
        self.declare_parameter('int_array_param', [1, 2, 3])
        self.declare_parameter('string_array_param', ['a', 'b', 'c'])
        
        # Set the initial values
        self.get_logger().info(f'Initial int_param: {self.get_parameter("int_param").get_parameter_value().integer_value}')
        self.get_logger().info(f'Initial float_param: {self.get_parameter("float_param").get_parameter_value().double_value}')
        self.get_logger().info(f'Initial string_param: {self.get_parameter("string_param").get_parameter_value().string_value}')
        self.get_logger().info(f'Initial bool_param: {self.get_parameter("bool_param").get_parameter_value().bool_value}')
        self.get_logger().info(f'Initial int_array_param: {self.get_parameter("int_array_param").get_parameter_value().integer_array_value}')
        self.get_logger().info(f'Initial string_array_param: {self.get_parameter("string_array_param").get_parameter_value().string_array_value}')
        
        # Register callback to handle parameter changes dynamically
        self.add_on_set_parameters_callback(self.on_parameter_change)
        
    def on_parameter_change(self, params):
        for param in params:
            if param.name == 'int_param':
                self.get_logger().info(f'Changed int_param: {param.value}')
            elif param.name == 'float_param':
                self.get_logger().info(f'Changed float_param: {param.value}')
            elif param.name == 'string_param':
                self.get_logger().info(f'Changed string_param: {param.value}')
            elif param.name == 'bool_param':
                self.get_logger().info(f'Changed bool_param: {param.value}')
            elif param.name == 'int_array_param':
                self.get_logger().info(f'Changed int_array_param: {param.value}')
            elif param.name == 'string_array_param':
                self.get_logger().info(f'Changed string_array_param: {param.value}')
        return rclpy.parameter.SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = ParameterReconfigureNode()

    # Spin the node to keep it alive and responsive to parameter changes
    rclpy.spin(node)

    # Clean up when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
