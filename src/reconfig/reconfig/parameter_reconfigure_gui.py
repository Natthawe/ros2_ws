import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QLabel, QGridLayout
from reconfig.parameter_reconfigure_node import ParameterReconfigureNode


class ParameterReconfigureGui(QWidget):
    def __init__(self):
        super().__init__()

        # ROS2 Node Setup
        rclpy.init()
        self.node = ParameterReconfigureNode()

        # Set up GUI
        self.setWindowTitle("ROS2 Parameter GUI")
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

        self.setLayout(self.layout)
        self.show()

        # Get and update the parameters in the GUI
        self.get_initial_parameters()

    def get_initial_parameters(self):
        # Get the current parameters from the node
        int_array_param, string_array_param = self.node.get_parameters()

        # Populate the GUI with the current parameter values
        for i, value in enumerate(int_array_param):
            self.int_array_widgets[i].setText(str(value))

        for i, value in enumerate(string_array_param):
            self.string_array_widgets[i].setText(value)

    def update_parameters(self):
        # Get the values from the input fields and update parameters
        int_array = [int(widget.text()) for widget in self.int_array_widgets]
        string_array = [widget.text() for widget in self.string_array_widgets]

        # Update ROS2 parameters via the node
        self.node.update_parameters(int_array, string_array)

        # Log the updated parameters
        print(f'Updated int_array_param: {int_array}')
        print(f'Updated string_array_param: {string_array}')


def main(args=None):
    app = QApplication(sys.argv)
    gui = ParameterReconfigureGui()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
