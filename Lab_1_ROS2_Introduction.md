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










































































---------------------------------------------------------------------
### Task 1
