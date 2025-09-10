## Task 2

### Move into src folder in your workspace
```
cd Group_3/src
```

### Create new package for custom message
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 custom_interface --dependencies rclcpp std_msgs
```
[my_folder/Task 2 01.png](https://github.com/olpuri/Algorithmic-Foundations-ros/blob/shivani_chadha/my_folder/Task%202%2001.png)
### In the package make the folder and create a new empty file
```
cd custom_interface
mkdir msg
cd msg
touch person.msg
```
### Check if the new message is available
```
cd ~/Group_3
colcon build --packages-select custom_interface
source install/setup.bash
ros2 interface show custom_interface/msg/Person
```
### Create a new python based package
```
cd src
ros2 pkg create --build-type ament_python customer_package --dependencies rclpy custom_interface
```
https://github.com/olpuri/Algorithmic-Foundations-ros/blob/shivani_chadha/my_folder/Task%202%2003.png
### In the required folder make two empty files
```
cd customer_package/customer_package
touch publisher.py
touch subscriber.py
cd ~/Group_3
```
### Build both your custom message package and your Python nodes package
```
colcon build --packages-select customer_package custom_interface
source install/setup.bash
```
### Run in Terminal 1
```
ros2 run customer_package publisher
```
https://github.com/olpuri/Algorithmic-Foundations-ros/blob/shivani_chadha/my_folder/Task%202%2004.png
### Run in Terminal 2
```
source install/setup.bash
ros2 run customer_package subscriber
```
