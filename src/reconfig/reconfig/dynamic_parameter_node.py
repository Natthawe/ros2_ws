import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

class DynamicParameterNode(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare a dynamically reconfigurable parameter
        self.declare_parameter(
            'speed', 0.5,  # Default value
            ParameterDescriptor(description='Speed of the robot')
        )

        # Timer to print the current parameter value
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Add callback to handle parameter updates
        self.add_on_set_parameters_callback(self.parameter_update_callback)

    def timer_callback(self):
        # Get the current value of the parameter
        speed = self.get_parameter('speed').value
        self.get_logger().info(f'Current speed: {speed}')

    def parameter_update_callback(self, params):
        for param in params:
            if param.name == 'speed':
                self.get_logger().info(f'Parameter updated: {param.name} = {param.value}')
        # Return a successful result
        return SetParametersResult(successful=True)

    # def parameter_update_callback(self, params):
    #     # Handle parameter updates
    #     for param in params:
    #         if param.name == 'speed':
    #             self.get_logger().info(f'Parameter updated: {param.name} = {param.value}')
    #     return rclpy.parameter.ParameterValue()

def main(args=None):
    rclpy.init(args=args)
    node = DynamicParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
