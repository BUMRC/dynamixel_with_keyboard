# dynamixel_with_keyboard

Angle incremented and decremented by 36 degree per 1 key press.

Keyboard Control Instructions

Press 'q' to increment position of ID 1.

Press 'a' to decrement position of ID 1.

Press 'w' to increment position of ID 2.

Press 's' to decrement position of ID 2.

Press 'e' to increment position of ID 3.

Press 'd' to decrement position of ID 3.

Press 'r' to increment position of ID 4.

Press 'f' to decrement position of ID 4.

Press 't' to increment position of ID 5.

Press 'g' to decrement position of ID 5.

Press 'y' to increment position of ID 6.

Press 'h' to decrement position of ID 6.

Press 'x' to exit the program and kill key_control_node

All other files other than key_control_node.cpp are from https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/humble-devel/.gitattributes

Steps to run on Jetson

Open 2 terminals
```
cd Desktop/ros_ws_dynamixel
```
```
source install/setup.bash
```
First terminal
```
ros2 run dynamixel_sdk_examples read_write_node
```
Second terminal (2 options: Publish ID & Position to topic (Position Range: 0-4000) or Using Key Press to control each motor)
```
ros2 topic pub  -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
```
```
ros2 run dynamixel_sdk_examples key_control_node
```

