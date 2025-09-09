# ros2_ibus Package
This package allows you to easily read the channels of an RC receiver using Flysky's iBus protocol. It utilizes the Python library [flySkyiBus](https://pypi.org/project/flySkyiBus) to obtain serial readings from the receiver. This package has been tested with a Flysky receiver FS-iA6B and a Raspberry Pi4B, making the following connections:

| Pin on Receiver | Pin on Raspberry Pi | Description          |
|-----------------|---------------------|----------------------|
| VCC             | 5V                  | Power supply         |
| GND             | GND                 | Ground               |
| iBus SERVO      | GPIO 15 (UART RX)   | Serial data receive  |

![FS-iA6B Connections](img/fsia6b_connections.png)

It also provides a console viewer to visualize the readings of the different channels.

## Nodes

### ibus_node

ROS2 node to read data from the iBus and publish Joy messages.

**Publishes:** `/joy`

### ibus_console_viewer_node

ROS2 node that subscribes to the `/joy` topic and displays the axes and buttons data in the console.

**Subscribes:** `/joy`

**Functionality:** This node listens to the `Joy` messages published on the `/joy` topic and provides a real-time console visualization of the axes and buttons data, making it easier to monitor and debug the RC receiver's output.

## Installation

To install this package, clone the repository into your ROS2 workspace and build it using colcon:

```sh
cd ~/ros2_ws/src
git clone https://github.com/maaad/ros2_ibus.git
cd ~/ros2_ws
colcon build
```

## Usage

```sh
ros2 run ros2_ibus ibus_node
ros2 run ros2_ibus ibus_console_viewer_node
```

## Credits

https://github.com/JaimeBravoAlgaba/ros2_ibus
