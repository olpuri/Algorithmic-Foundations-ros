## Task3
### Creating the package: Open the terminal and navigate to your workpace/src folder
```
cd ~/Group_3/src/Task_3
ros2 pkg create --build-type ament_python euclidean_distance --dependencies rclpy geometry_msgs custom_interface
```
### Create a new custom service:
```
cd ~/Group_3/src/Task_3/custom_interface
mkdir srv
cd srv
touch CalDistance.srv

```
### Inside CalDistance.srv
```
geometry_msgs/Point p1
geometry_msgs/Point p2
---
float64 distance
bool success
string message

```
### Modify CMakeList.txt:
### Editing in rosidl_generate_interfaces:
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "srv/CalDistance.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```
### CODE AFTER MODIFYING:
```
cmake_minimum_required(VERSION 3.8)
project(custom_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Person.msg"
"srv/CalDistance.srv"
DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```
### Modify package.xml:
### Adding the dependency
```
<depend>geometry_msgs</depend>
```
### CODE AFTER MODIFYING:
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_interface</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
### Building and checking the service:
```
cd Group_3/src/Task_3
colcon build --packages-select custom_interface
source install/setup.bash
```
### Checking it's existance:
```
ros2 interface show custom_interface/srv/CalDistance
```
### Creating Server Node:
### inside euclidean_distance/euclidean_distance
```
cd ~/Group_3/src/Task_3/cutom_interface/euclidean_distance
mkdir euclidean_distance
cd euclidean_distance
touch __init__.py dis_ser.py dis_client.py
```
### Open dis_ser.py in VS Code and write the following code: dis_ser.py
```
import rclpy
from rclpy.node import Node
from custom_interface.srv import CalDistance

import math

class DistanceServer(Node):
    def __init__(self):
        super().__init__('distance_server')
        self.srv = self.create_service(CalDistance, 'Calculate_distance', self.calculate_callback)
        self.get_logger().info('Distance Service Server started.')

    def calculate_callback(self, request, response):
        dx = request.p1.x - request.p2.x
        dy = request.p1.y - request.p2.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        response.distance = distance
        response.success = True
        response.message = f"Distance calculated successfully: {distance:.2f}"
        
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DistanceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Create Client Node:
### Open dis_client.py in VS Code and write the following code:
```
import rclpy
from rclpy.node import Node
from custom_interface.srv import CalDistance
from geometry_msgs.msg import Point
from functools import partial

class DistanceClient(Node):
    def __init__(self):
        super().__init__('distance_client')
        self.client = self.create_client(CalDistance, 'Calculate_distance')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Example points
        p1 = Point(x=1.0, y=2.0, z=0.0)
        p2 = Point(x=4.0, y=6.0, z=0.0)

        request = CalDistance.Request()
        request.p1 = p1
        request.p2 = p2

        future = self.client.call_async(request)
        future.add_done_callback(partial(self.callback, p1=p1, p2=p2))

    def callback(self, future, p1, p2):
        try:
            response = future.result()
            self.get_logger().info(
                f'Distance between ({p1.x}, {p1.y}) and ({p2.x}, {p2.y}) = {response.distance:.2f}, '
                f'Success={response.success}, Message="{response.message}"'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DistanceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Update setup.py:
### Changing the entry points:
```
entry_points={
    'console_scripts': [
        'dis_ser = euclidean_distance.dis_ser:main',
        'dis_client = euclidean_distance.dis_client:main',
    ],
```
### CODE AFTER MODIFYING:
```
from setuptools import find_packages, setup

package_name = 'euclidean_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'dis_ser = euclidean_distance.dis_ser:main',
        'dis_client = euclidean_distance.dis_client:main',
    ],
},

)
```
### Building and Running:
```
cd Group_3/src/Task_3
colcon build --packages-select euclidean_distance
source install/setup.bash
```
### Running in Terminal 1: Server Side
```
ros2 run euclidean_distance dis_ser
```
### Running in Terminal 2: Client Side
```
cd Group_3/src/Task_3
colcon build --packages-select distance_client                       
source install/setup.bash
ros2 run euclidean_distance dis_client
```









