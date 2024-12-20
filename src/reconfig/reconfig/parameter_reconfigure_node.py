import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class ParameterReconfigureNode(Node):
    def __init__(self):
        super().__init__('parameter_reconfigure_node')

        # Declare parameters
        self.declare_parameter('int_array_param', [1, 2, 3])
        self.declare_parameter('string_array_param', ['A0: 0.0, 0.0, 0.0', 'A1: 0.0, 0.0, 0.0', 'A2: 0.0, 0.0, 0.0'])

        self.get_logger().info("Parameter node started")

    def get_parameters(self):
        # Retrieve current parameter values
        int_array_param = self.get_parameter('int_array_param').get_parameter_value().integer_array_value
        string_array_param = self.get_parameter('string_array_param').get_parameter_value().string_array_value
        return int_array_param, string_array_param

    def update_parameters(self, int_array, string_array):
        # Update ROS2 parameters (note the list syntax)
        self.set_parameters([
            Parameter('int_array_param', rclpy.Parameter.Type.INTEGER_ARRAY, int_array),
            Parameter('string_array_param', rclpy.Parameter.Type.STRING_ARRAY, string_array)
        ])
        # Log the updated parameters
        self.get_logger().info(f'Updated int_array_param: {int_array}')
        self.get_logger().info(f'Updated string_array_param: {string_array}')
        # Save parameters to YAML
        self.save_parameters_to_yaml(int_array, string_array)

    # def save_parameters_to_yaml(self, int_array, string_array):
    #     """Save parameters to a YAML file."""
    #     params = {
    #         'int_array_param': list(int_array),
    #         'string_array_param': [str(item) for item in string_array]
    #     }
    #     params_file_path = '/home/natthawe/ros2_tutorials_ws/src/reconfig/config/parameters.yaml'
    #     with open(params_file_path, 'w') as file:
    #         yaml.dump(params, file, default_flow_style=False, sort_keys=False)
    #     self.get_logger().info(f'Parameters saved to {params_file_path}')

    def save_parameters_to_yaml(self, int_array, string_array):
        """Save parameters to a YAML file."""
        params = {
            'int_array_param': list(int_array),
            'string_array_param': [str(item) for item in string_array]
        }

        package_share_directory = get_package_share_directory('reconfig')
        params_file_path = os.path.join(package_share_directory, 'config', 'parameters.yaml')        
        # params_file_path = '/home/natthawe/ros2_tutorials_ws/src/reconfig/config/parameters.yaml'

        # Custom dumper to prevent unnecessary quotes
        class NoQuotesDumper(yaml.SafeDumper):
            def represent_str(self, data):
                if ':' in data or ',' in data:
                    return self.represent_scalar('tag:yaml.org,2002:str', data, style='"')
                return self.represent_scalar('tag:yaml.org,2002:str', data)

        yaml.add_representer(str, NoQuotesDumper.represent_str, Dumper=NoQuotesDumper)

        with open(params_file_path, 'w') as file:
            yaml.dump(params, file, Dumper=NoQuotesDumper, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Parameters saved to {params_file_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterReconfigureNode()

    # Spin the ROS2 node
    rclpy.spin(node)

    # Shutdown the ROS2 node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
