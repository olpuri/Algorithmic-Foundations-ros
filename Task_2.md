## Task 2

### Move into src folder in your workspace
```
cd Group_3/src
```

### Create new package for custom message
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 custom_interface --dependencies rclcpp std_msgs
```
<img width="722" height="380" alt="Task 2 01" src="https://github.com/user-attachments/assets/9614c4b4-b1d4-4090-903d-c0277d674dc4" />

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
<img width="662" height="107" alt="image" src="https://github.com/user-attachments/assets/02c90838-1c21-4fbb-a414-28c577178965" />

### Create a new python based package
```
cd src
ros2 pkg create --build-type ament_python customer_package --dependencies rclpy custom_interface
```
<img width="1147" height="647" alt="Task 2 03" src="https://github.com/user-attachments/assets/7a9a17e9-e156-41dc-ba5a-532693b739df" />

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
<img width="737" height="430" alt="Task 2 04" src="https://github.com/user-attachments/assets/25eff618-1e63-4914-92c9-9db9cebb19b8" />

### Run in Terminal 2
```
source install/setup.bash
ros2 run customer_package subscriber
```
<img width="817" height="457" alt="Task 2 05" src="https://github.com/user-attachments/assets/ee94cafd-7180-46aa-9fa9-c6150ab76d92" />

