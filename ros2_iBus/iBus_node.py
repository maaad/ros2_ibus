import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from flySkyiBus import IBus

class iBusPublisher(Node):
	def __init__(self, bus='/dev/serial0'):
		super().__init__('joy_node')

		self.bus = IBus(bus)
		self.publisher_ = self.create_publisher(Joy, 'joy', 10)

		while True:
			data = self.bus.read()  # Read data from serial port
				
			if data[0]==32 and data[1]==64:
					self.msg = Joy()
					self.msg.header.stamp = self.get_clock().now().to_msg()
					self.msg.header.frame_id = '0'
					
					for i in range(2, 6):
						self.msg.axes.append(float((data[i] - 1000)/1000))
					
					for i in range(6, 8):
						self.msg.buttons.append(int(data[i] - 1000))				
					
					self.publisher_.publish(self.msg)
			else:
				pass

def main(args=None):
    rclpy.init(args=args)

    iBus_publisher = iBusPublisher()

    rclpy.spin(iBus_publisher)

    # Destroy the node explicitly
    iBus_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
