import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from flySkyiBus import IBus

class iBusPublisher(Node):
	def __init__(self, bus='/dev/serial0'):
		super().__init__('rc_channels')
		self.publisher_ = self.create_publisher(Joy, 'topic', 10)
		timer_period = 0.01  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

		self.bus = IBus(bus)

	def timer_callback(self):
		data = self.bus.read()  # Read data from serial port
		
		if data[0]==32 and data[1]==64:
				msg = Joy()
				msg.header = self.i
				msg.data = data[2:9]
				msg.buttons = data[8:11]
				
				self.publisher_.publish(msg)        	
				self.i += 1
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
