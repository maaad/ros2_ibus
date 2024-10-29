# Description: ROS2 node that subscribes to the 'joy' topic and plots the axes and buttons data in real-time.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import matplotlib.pyplot as plt

class iBusViewer(Node):
    def __init__(self):
        super().__init__('ibus_viewer_node')

        # Subscription to the 'joy' topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize variables
        self.i = 0
        self.buffer_size = 100

    def listener_callback(self, msg):
        # Initialize data buffer if it doesn't exist
        if not hasattr(self, 'data_buffer'):
            self.data_buffer = {'ax0': [], 'ax1': [], 'ax2': [], 'ax3': [], 'bt0': [], 'bt1': []}
        
        # Append new data to the buffer
        for i in range(4):
            self.data_buffer['ax'+str(i)].append(msg.axes[i])
        for i in range(2):
            self.data_buffer['bt'+str(i)].append(msg.buttons[i])
        
        # Maintain buffer size
        if len(self.data_buffer['ax0']) > self.buffer_size:
            self.data_buffer['ax0'].pop(0)
            self.data_buffer['ax1'].pop(0)
            self.data_buffer['ax2'].pop(0)
            self.data_buffer['ax3'].pop(0)
            self.data_buffer['bt0'].pop(0)
            self.data_buffer['bt1'].pop(0)
            self.i += 1

        # Clear the current plot
        plt.clf()
        plt.subplots_adjust(hspace=0.4)  # Increase padding between plots

        # Plot axes data
        ax = plt.subplot(3, 1, 1)
        x = range(self.i, self.i+len(self.data_buffer['ax0']))
        plt.plot(x, self.data_buffer['ax0'], label='ax0')
        plt.plot(x, self.data_buffer['ax1'], label='ax1')
        plt.plot(x, self.data_buffer['ax2'], label='ax2')
        plt.plot(x, self.data_buffer['ax3'], label='ax3')
        plt.legend(loc='lower left', fontsize=16)
        plt.grid()
        plt.title('Axes', fontsize=16)
        plt.ylim([-1.1, 1.1])
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # Plot buttons data
        ax = plt.subplot(3, 1, 2)
        plt.plot(x, self.data_buffer['bt0'], label='bt0')
        plt.plot(x, self.data_buffer['bt1'], label='bt1')
        plt.legend(loc='lower left', fontsize=16)
        plt.grid()
        plt.title('Buttons', fontsize=16)
        plt.ylim([0, 1023])
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # Display the latest values in a table
        ax = plt.subplot(3, 3, 8)
        cell_text = [
            [f"{self.data_buffer['ax0'][-1]:.3f}" if self.data_buffer['ax0'] else "0.000"],
            [f"{self.data_buffer['ax1'][-1]:.3f}" if self.data_buffer['ax1'] else "0.000"],
            [f"{self.data_buffer['ax2'][-1]:.3f}" if self.data_buffer['ax2'] else "0.000"],
            [f"{self.data_buffer['ax3'][-1]:.3f}" if self.data_buffer['ax3'] else "0.000"],
            [self.data_buffer['bt0'][-1] if self.data_buffer['bt0'] else 0],
            [self.data_buffer['bt1'][-1] if self.data_buffer['bt1'] else 0]
        ]
        ax.axis('tight')
        ax.axis('off')

        columns = ('Value',)
        rows = ['ax0', 'ax1', 'ax2', 'ax3', 'bt0', 'bt1']
        table = plt.table(cellText=cell_text, rowLabels=rows, colLabels=columns, loc='center', fontsize=16)
        table.auto_set_font_size(False)
        table.set_fontsize(16)
        
        # Increase row height
        table.scale(1, 2)  # Scale by width=1 and height=2
        
        plt.axis('off')

        # Pause to update the plot
        plt.pause(0.001)

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create the iBusViewer node
    iBus_viewer = iBusViewer()

    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(iBus_viewer)

    # Destroy the node explicitly
    iBus_viewer.destroy_node()
    rclpy.shutdown()