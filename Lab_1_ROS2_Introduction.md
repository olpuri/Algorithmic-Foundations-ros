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
### Create a Package using:
```
mkdir -p Group_3/src
cd Group_3/src
```
### Creating a package named Group_3, setting License and dependencies using:
```
ros2 pkg create --build-type ament_python Group_3 --license Apache-2.0 --
dependencies rclpy std_msgs geometry_msgs
```
### Working with Group_3 in src folder:
### Publisher.py and Required Changes:
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
### Verifying:
```
python3 publisher.py
```
### Registering Executables: Changing entry_point{} in setup.py
```
entry_points={ 
 'console_scripts': [ 
 'publisher = <package_name>.publisher:main', 
 ], 
},
```
### Building, Sourcing, Creating Alias, Launching, and Getting list of topics:
```
colcon build
source install/setup.bash
alias sb="source install/setup.bash"
ros2 run Group_3 publisher
ros2 topic list
```
### Subscriber.py and required changes:
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy,
HistoryPolicy
class MinimalSubscriber(Node):
 def __init__(self):
 super().__init__('subscriber')
 topic_name = '/topic1' # TODO: # subscribe to the publisher’s topic.
 
 self.qos_profile = QoSProfile(
 reliability=ReliabilityPolicy.RELIABLE, # Ensure delivery of
messages
 durability=DurabilityPolicy.TRANSIENT_LOCAL, # Keep messages even
if the publisher dies
 history=HistoryPolicy.KEEP_LAST, # Only store the last
N messages
 depth=10 # The number of
messages to store (N=10)
 )
 self.subscription = self.create_subscription(
 String,
 topic_name,
 self.listener_callback,
 self.qos_profile)
 
 self.subscription
 def listener_callback(self, msg):
 self.get_logger().info(f'I heard: {msg.data}')
def main(args=None):
 rclpy.init(args=args)
minimal_subscriber = MinimalSubscriber()
 rclpy.spin(minimal_subscriber)
 minimal_subscriber.destroy_node()
 rclpy.shutdown()
 
if __name__ == '__main__':
 main()
```
### Changing entry_points for Subscriber:
```
entry_points={ 
 'console_scripts': [ 
 'publisher = Group_3.publisher:main', 
 'subscriber = Group_3.subscriber:main', 
 ], 
},
```
### Navigation to workspace:
```
colcon build 
source install/setup.bash
ros2 run Group_3 subscriber
```
## Running the Code
### Terminal 1
```
source install/setup.bash 
ros2 run Group_3 publisher
```
### Terminal 2
```
source install/setup.bash 
ros2 run Group_3 subscriber
```







































































---------------------------------------------------------------------
## Task 1
### Create pkg task_1
(from the src folder inside your workspace)
```
ros2 pkg create --build-type ament_python <package_name> --license Apache-2.0 -- dependencies rclpy std_msgs geometry_msgs
```
### Create a publisher node 
(make sure you are in ros2_ws/src/task_1/task_1)
```
touch pose_pubslisher.py
nano pose_publisher.py
```
Pose_publisher code:
```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        topic_name = '/pose_topic'  

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic_name,
            self.qos_profile
        )

        timer_period = 0.5  # seconds → publishes at 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0
        self.speed = 0.1  # step size

        self.get_logger().info("Pose publisher started")

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Straight line trajectory: diagonal in x-y
        msg.pose.position.x = self.t * self.speed
        msg.pose.position.y = self.t * self.speed
        msg.pose.position.z = 0.0

        # Identity orientation (no rotation)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing Pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

        self.t += 1.0


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Create a subscriber node 
```
touch subscriber.py
nano subscriber.py
```
Subscriber code:
```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        topic_name = '/pose_topic'  

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.listener_callback,
            self.qos_profile
        )

        self.get_logger().info("Pose subscriber started")

    def listener_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Received Pose → x: {x:.2f}, y: {y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
## Use the corresponding commands to test in the terminal:
```
ros2 run task_1 pose_publisher
```
![Task 1 Output 1](Lab1_pics/task1_output1.png)

### The existence of the topic
´´´
ros2 topic list
´´´
![Task 1 Output 2](Lab1_pics/task1_output2.png)

### The value of the message
´´´
ros2 topic echo /pose_topic
´´´
![Task 1 Output 3](Lab1_pics/task1_output3.png)
### The frequency of the topic
´´´
ros2 topic hz /pose_topic
´´´
![Task 1 Output 4](Lab1_pics/task1_output4.png)
### The number of subscribers and publisher for the topic
´´´
ros2 topic info /pose_topic
´´´
![Task 1 Output 5](Lab1_pics/task1_output5.png)
### OPTIONAL 1:
1. Explain what would be the QoS parameters for critical data in the system?
2. What about live data?
3. What type of data would it make sense to use Transient Local with?


In the pose_publisher we use these parameters:
```
self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
```
1. If we refer to [ROS2 Documentation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html) then can see that Quality of Service (QoS) policies are used to let you set the communication between nodes.
As I understand the critical data in this case is the position date:
pose.position.x and pose.position.y, also could be pose.orientation.
Because robot’s position is essential for navigation or decision-making, our **Reliability** parameter is RELIABLE. 
Otherwise, we can lose samples, if the network is not working well;
***Durability** is TRANSIENT_LOCAL to make sure that late subscribers will still get the last message;
2. **History** KEEP_LAST work well with critical and live data(most recent) like robot pose. And depth=10 keeps the last 10 messages in a buffer.
3. From the ROS2 Documentation we can see that "Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscriptions." Also going through [this source](https://robotics.stackexchange.com/questions/24889/qos-history-policy-in-case-of-durability-volatile-and-reliability-best-effort) and [this](https://docs.clearpathrobotics.com/docs/ros/api/overview/#:~:text=Transient%20Local%E2%80%8B&text=The%20Transient%20Local%20QoS%20Profile,topic%20which%20offers%20system%20logs.) **Transient Local** is used when you want new subscribers, which connect later, to automatically receive the last published message(s) that were saved by the publisher. Then in the case of Task 1, parameter  **Volatile** also could be used, beacause we are not supposed to have late subscribers, but at the same time Volatile only delivers data to subscribers that are connected immediately. Once a subscriber joins, it only gets new messages.
Answering the question: real-time sensor data, robot description/URDF and robot position for starting later nodes.  

