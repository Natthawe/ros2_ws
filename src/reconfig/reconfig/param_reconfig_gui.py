import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QLabel, QGridLayout

class ParameterReconfigureNode(Node):
    def __init__(self):
        super().__init__('parameter_reconfigure_node')

        # Declare parameters
        self.declare_parameter('int_array_param', [1, 2, 3])
        self.declare_parameter('string_array_param', ['A0: 1.0, 0.0, 0.0', 'A1: 2.0, 0.0, 0.0', 'A2: 3.0, 0.0, 0.0'])

        # Set up GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("ROS2 Parameter GUI")
        
        self.layout = QGridLayout()

        # Create widgets for int_array_param
        self.int_array_widgets = []
        for i in range(3):  # Assuming 3 elements in the array
            self.int_array_widgets.append(QLineEdit())
            self.layout.addWidget(QLabel(f"int_array_param_{i}"), i, 0)
            self.layout.addWidget(self.int_array_widgets[i], i, 1)

        # Create widgets for string_array_param
        self.string_array_widgets = []
        for i in range(3):  # Assuming 3 elements in the array
            self.string_array_widgets.append(QLineEdit())
            self.layout.addWidget(QLabel(f"string_array_param_{i}"), i+3, 0)
            self.layout.addWidget(self.string_array_widgets[i], i+3, 1)

        # Update button
        self.update_button = QPushButton("Update Parameters")
        self.update_button.clicked.connect(self.update_parameters)
        self.layout.addWidget(self.update_button, 6, 0, 1, 2)

        self.window.setLayout(self.layout)
        self.window.show()

        # Get initial parameters and populate GUI with them
        self.get_initial_parameters()

        # Spin the ROS node
        self.spin_thread = self.create_spin_thread()
        self.spin_thread.start()

    def get_initial_parameters(self):
        # Retrieve current parameter values
        int_array_param = self.get_parameter('int_array_param').get_parameter_value().integer_array_value
        string_array_param = self.get_parameter('string_array_param').get_parameter_value().string_array_value

        # Populate the GUI with the current parameter values
        for i, value in enumerate(int_array_param):
            self.int_array_widgets[i].setText(str(value))

        for i, value in enumerate(string_array_param):
            self.string_array_widgets[i].setText(value)

    def update_parameters(self):
        # Get the values from the input fields and update parameters
        int_array = [int(widget.text()) for widget in self.int_array_widgets]
        string_array = [widget.text() for widget in self.string_array_widgets]

        # Update ROS2 parameters (note the list syntax)
        self.set_parameters([Parameter('int_array_param', rclpy.Parameter.Type.INTEGER_ARRAY, int_array)])
        self.set_parameters([Parameter('string_array_param', rclpy.Parameter.Type.STRING_ARRAY, string_array)])

        # Log the updated parameters
        self.get_logger().info(f'Updated int_array_param: {int_array}')
        self.get_logger().info(f'Updated string_array_param: {string_array}')

        # Save the updated parameters to a YAML file
        self.save_parameters_to_yaml(int_array, string_array)

    def save_parameters_to_yaml(self, int_array, string_array):
        """Save parameters to a YAML file."""
        # Create the parameters dictionary
        params = {
            'int_array_param': int_array,
            'string_array_param': string_array
        }

        # Define the path to the parameters YAML file
        params_file_path = '/home/natthawe/ros2_tutorials_ws/src/reconfig/config/parameters.yaml'

        # Write the parameters to the YAML file
        with open(params_file_path, 'w') as file:
            # Special format for string_array_param as required
            formatted_params = {
                'int_array_param': int_array,
                'string_array_param': [f"{item}" for item in string_array]  # Ensuring string format for string_array_param
            }
            yaml.dump(formatted_params, file, default_flow_style=False)

        self.get_logger().info(f'Parameters saved to {params_file_path}')

    def create_spin_thread(self):
        """Create a thread to spin the ROS node while the GUI is active"""
        from threading import Thread
        def spin():
            rclpy.spin(self)
        thread = Thread(target=spin)
        return thread

def main(args=None):
    rclpy.init(args=args)

    node = ParameterReconfigureNode()

    # Start the ROS2 node and the custom GUI
    sys.exit(node.app.exec_())

if __name__ == '__main__':
    main()
