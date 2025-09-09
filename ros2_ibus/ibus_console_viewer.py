#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class IBusConsoleViewer(Node):
    def __init__(self):
        super().__init__('ibus_console_viewer')
        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)

    def callback(self, msg):
        print('\033c', end='')  # очистка терминала

        num_axes = min(10, len(msg.axes))  # отображаем до 10 каналов
        for i in range(num_axes):
            # нормируем значения -1..1 в 0..1
            value = (msg.axes[i] + 1) / 2
            bar_length = int(value * 50)
            bar = '█' * bar_length
            print(f'CH{i+1:02d}: {bar:<50} {msg.axes[i]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = IBusConsoleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

