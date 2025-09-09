# Description: ROS2 node to read data from the iBus and publish Joy messages

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from flySkyiBus import IBus

# Define the iBusPublisher class
class iBusPublisher(Node):
	def __init__(self, bus='/dev/ttyAMA0'):
		super().__init__('ibus_node')

		# Initialize the IBus object with the specified bus
		self.bus = IBus(bus)

		# Create a publisher for the Joy message on the 'joy' topic
		self.publisher_ = self.create_publisher(Joy, 'joy', 1)

		# Main loop to read data from the iBus and publish Joy messages
		while True:
			data = self.bus.read()  # Read data from serial port

			# Check if the data is valid
			if data[0] == 32 and data[1] == 64:
				self.msg = Joy()
				self.msg.header.stamp = self.get_clock().now().to_msg()
				self.msg.header.frame_id = ''

				# Populate the axes field of the Joy message (10 channels)
				for i in range(2, 12):
					self.msg.axes.append(float((data[i] - 1500) / 500))

				# Publish the Joy message
				self.publisher_.publish(self.msg)
			else:
				pass

# Main function to initialize and spin the ROS node
def main(args=None):
	rclpy.init(args=args)

	# Create an instance of the iBusPublisher
	iBus_publisher = iBusPublisher()

	# Spin the node to keep it active
	rclpy.spin(iBus_publisher)

	# Destroy the node explicitly
	iBus_publisher.destroy_node()
	rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
	main()
