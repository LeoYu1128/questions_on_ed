The target file is arm_control_test. The detail of the problem I faced can be seen on ed forum.

I want to let the function "execute_push" work success, which requires a linear path.

Please run the setupFakeur5e.sh first.

And after build and source, please go into launch file and run "ros2 launch test_launch.py".

mtrn@mtrn-VirtualBox:~/4231/group-assignment-the-monkey-business-gang/test$ source install/local_setup.bash

mtrn@mtrn-VirtualBox:~/4231/group-assignment-the-monkey-business-gang/test$ cd launch

mtrn@mtrn-VirtualBox:~/4231/group-assignment-the-monkey-business-gang/test/launch$ ros2 launch test_launch.py

Basically, line 339 is a callback function "messageRead", it'll be ececute after subscribe the message from the SensorBridge, which will publish a target pose.

Then the "simple_test" function will be called.
