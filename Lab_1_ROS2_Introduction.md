# Lab 1
## Prerequisites and Workspace
Ubuntu22 + Ros 2 Humble

```
nano ~/.bashrc
alias sb="source ~/ros2_ws/install/setup.bash"
```
### Command that provides all the topics that are currently running:

```
ros2 topic list 
```
### rosout topic where logging messages are sent by all the nodes:
```
ros2 topic echo /rosout
```
### The command that provides the metadata about the topic:
```
ros2 topic info /rosout
```
### Publish message to a topic from command line by using:
```
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello from Group'}"
```
### Measures the frequency of the topic by using:
```
ros2 topic hz /chatter
```
### Publisher.py
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        topic_name = '/topic1'  # TODO: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep messages if the publisher dies
            history=HistoryPolicy.KEEP_LAST,  # Only store the last N messages
            depth=10  # Number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            String,
            topic_name,
            self.qos_profile
        )

        timer_period = 2.0  # TODO: set time in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Pigeon sent {self.i}'  # TODO: customize message content
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
### Subscriber.py
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        topic_name = '/topic1'  # TODO: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep messages if the publisher dies
            history=HistoryPolicy.KEEP_LAST,  # Only store the last N messages
            depth=10  # Number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            String,
            topic_name,
            self.qos_profile
        )

        timer_period = 2.0  # TODO: set time in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Pigeon sent {self.i}'  # TODO: customize message content
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```










































































---------------------------------------------------------------------
### Task 1
