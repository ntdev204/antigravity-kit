---
name: ros2-humble
description: ROS2 Humble development principles and best practices. Nodes, topics, services, actions, lifecycle management, package creation, launch systems, common packages, and simulation integration. Use when developing ROS2 applications, robotic systems, or autonomous systems with ROS2 Humble.
---

# ROS2 Humble

> Modern robotics middleware for distributed robotic systems.

---

## 1. ROS2 Core Concepts

### Communication Patterns

| Pattern        | Characteristics          | Use Cases                       |
| -------------- | ------------------------ | ------------------------------- |
| **Topics**     | Pub/Sub, many-to-many    | Sensor data, continuous streams |
| **Services**   | Request/Response, 1-to-1 | Commands, queries               |
| **Actions**    | Goal-Feedback-Result     | Long-running tasks              |
| **Parameters** | Configuration            | Runtime settings                |

### Quality of Service (QoS)

| Profile            | Reliability | History     | Use Cases              |
| ------------------ | ----------- | ----------- | ---------------------- |
| **Sensor Data**    | Best effort | Small depth | High-frequency sensors |
| **Services**       | Reliable    | Keep last   | Request/response       |
| **Parameters**     | Reliable    | Keep all    | Configuration          |
| **System Default** | Reliable    | Keep last   | General purpose        |

---

## 2. Package Structure

### Standard ROS2 Package

```
my_package/
├── package.xml          # Package metadata
├── setup.py             # Python package setup
├── setup.cfg            # Package configuration
├── CMakeLists.txt       # C++ build (if applicable)
├── my_package/          # Python source
│   ├── __init__.py
│   └── my_node.py
├── launch/              # Launch files
│   └── my_launch.py
├── config/              # Configuration files
│   └── params.yaml
├── msg/                 # Custom messages
├── srv/                 # Custom services
├── action/              # Custom actions
└── test/                # Tests
```

### Package Creation

```bash
# Python package
ros2 pkg create --build-type ament_python my_package

# C++ package
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp

# Mixed package
ros2 pkg create --build-type ament_cmake my_package \
  --dependencies rclcpp rclpy
```

---

## 3. Node Development

### Python Node Template

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer triggered')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### C++ Node Template

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        RCLCPP_INFO(this->get_logger(), "Node started");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timer_callback, this));
    }

private:
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer triggered");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 4. Publishers & Subscribers

### Python Publisher

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS2'
        self.publisher.publish(msg)
```

### Python Subscriber

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String, 'topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

---

## 5. Services

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### Service Client

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

---

## 6. Actions

### Action Server

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # Provide feedback
        feedback_msg = Fibonacci.Feedback()

        # Execute goal
        result = Fibonacci.Result()
        goal_handle.succeed()
        return result
```

---

## 7. Launch Files

### Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='custom_name',
            parameters=[{'param_name': param_value}],
            arguments=['--ros-args', '--log-level', 'INFO'],
            remappings=[('old_topic', 'new_topic')]
        )
    ])
```

### Advanced Launch Features

| Feature                   | Usage                      |
| ------------------------- | -------------------------- |
| **Include**               | Include other launch files |
| **Group**                 | Namespace/scoping          |
| **SetParameter**          | Set parameters             |
| **DeclareLaunchArgument** | Command-line arguments     |
| **Condition**             | Conditional launching      |

---

## 8. Parameters

### Declaring Parameters

```python
class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter('my_param', 'default_value')
        value = self.get_parameter('my_param').value
```

### Parameter File (YAML)

```yaml
my_node:
    ros__parameters:
        my_param: 42
        another_param: "hello"
        nested:
            value: 3.14
```

### Loading Parameters

```bash
ros2 run my_package my_node --ros-args --params-file params.yaml
```

---

## 9. Custom Messages

### Message Definition (.msg)

```
# my_package/msg/CustomMsg.msg
std_msgs/Header header
int32 id
string name
float64[] data
```

### Service Definition (.srv)

```
# my_package/srv/CustomSrv.srv
int32 request_id
string command
---
bool success
string message
```

### Action Definition (.action)

```
# my_package/action/CustomAction.action
# Goal
int32 target
---
# Result
bool success
---
# Feedback
float32 progress
```

### package.xml Dependencies

```xml
<depend>action_msgs</depend>
<depend>rosidl_default_generators</depend>
<build_depend>std_msgs</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## 10. Lifecycle Nodes

### Lifecycle States

| State            | Description               |
| ---------------- | ------------------------- |
| **Unconfigured** | Initial state             |
| **Inactive**     | Configured but not active |
| **Active**       | Fully operational         |
| **Finalized**    | Shutdown                  |

### Transitions

```
Unconfigured → Configure → Inactive
Inactive → Activate → Active
Active → Deactivate → Inactive
Inactive → Cleanup → Unconfigured
Any → Shutdown → Finalized
```

### Implementation

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state: State):
        self.get_logger().info('Configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating')
        return TransitionCallbackReturn.SUCCESS
```

---

## 11. Common ROS2 Packages

### Core Packages

| Package           | Purpose                |
| ----------------- | ---------------------- |
| **rclcpp**        | C++ client library     |
| **rclpy**         | Python client library  |
| **std_msgs**      | Standard message types |
| **sensor_msgs**   | Sensor message types   |
| **geometry_msgs** | Geometry messages      |
| **tf2**           | Transform library      |
| **nav2**          | Navigation stack       |

### Sensor Messages

| Message         | Use Case              |
| --------------- | --------------------- |
| **Image**       | Camera images         |
| **LaserScan**   | 2D lidar              |
| **PointCloud2** | 3D lidar              |
| **Imu**         | IMU data              |
| **JointState**  | Robot joint positions |

---

## 12. TF2 (Transforms)

### Broadcasting Transforms

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'
        t.transform.translation.x = 1.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
```

### Listening to Transforms

```python
from tf2_ros import Buffer, TransformListener

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'robot', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Transform error: {e}')
```

---

## 13. Simulation

### Gazebo Integration

| Tool               | Purpose                 |
| ------------------ | ----------------------- |
| **gazebo_ros**     | ROS2-Gazebo bridge      |
| **gazebo_plugins** | Sensor/actuator plugins |
| **ros2 launch**    | Launch Gazebo with ROS2 |

### URDF/Xacro

```xml
<!-- robot.urdf.xacro -->
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 0.5"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### RViz2

```bash
ros2 run rviz2 rviz2
```

**Common Displays:**

- Camera
- LaserScan
- PointCloud2
- TF
- Robot Model

---

## 14. Best Practices

### Node Design

- Single responsibility per node
- Use composition for multi-node processes
- Implement lifecycle for managed nodes
- Handle shutdown gracefully

### Topic Naming

```
/robot_name/sensor_type/data
Example: /robot1/camera/image_raw
```

### QoS Selection

- Sensor data → Best effort, small depth
- Commands → Reliable, depth 10
- State → Reliable, transient local

### Performance

- Use composable nodes for efficiency
- Minimize message copies
- Use zero-copy transport when available
- Profile with ros2 topic hz/bw

---

## 15. Debugging Tools

| Tool             | Purpose                 |
| ---------------- | ----------------------- |
| **ros2 topic**   | List, echo, pub, hz, bw |
| **ros2 node**    | List, info              |
| **ros2 param**   | Get, set, dump, load    |
| **ros2 service** | Call, list, type        |
| **ros2 bag**     | Record, play, info      |
| **ros2 run**     | Run nodes               |
| **ros2 launch**  | Launch files            |
| **rqt**          | GUI tools               |

---

## 16. Common Pitfalls

| Problem                  | Solution                            |
| ------------------------ | ----------------------------------- |
| **DDS discovery issues** | Check network, use domain IDs       |
| **QoS incompatibility**  | Match QoS profiles                  |
| **Transform errors**     | Ensure all frames published         |
| **Parameter not found**  | Declare before use                  |
| **Node name conflicts**  | Use unique node names or namespaces |
| **Memory leaks**         | Proper shutdown, destroy nodes      |
| **Build errors**         | Check dependencies in package.xml   |

---

> **Remember:** ROS2 is middleware, not a framework. Design loosely coupled nodes, use appropriate communication patterns, and leverage existing packages when possible.
