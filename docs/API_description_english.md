 API description
=====
### Subscribed Topics:

* **/joint_goal (sensor_msgs/msg/JointState)** 
make the robot move to a position in joint space after planning a trajectory.  
example: function_pub_joints() in elfin_basic_api/scripts/test.py

* **/cart_goal (geometry_msgs/msg/PoseStamped)**  
make the robot move to a position in cartesian coordination system after planning a trajectory.  
example: function_pub_cart() in elfin_basic_api/scripts/test.py

* **/cart_path_goal (geometry_msgs/msg/PoseArray)**  
make the robot move through the assigned positions in cartesian coordination system after planning a trajectory composed of lines.  
example: function_pub_cart_path() in elfin_basic_api/scripts/test.py

* **/elfin_arm_controller/joint_trajectory (trajectory_msgs/msg/JointTrajectory)**  
This topic contain a trajectory. When you publish this topic, the robot will move along the trajectory.
example: function_pub_trajectory() in elfin_basic_api/scripts/test.py

* **/elfin_teleop_joint_cmd_no_limit (std_msgs/msg/Int64)**  
This is a topic for developer, customers are not supposed to use it. A particular joint will move a little distance after subscribing this topic for once.

	Meaning of the data in the topic:

	| data | joint       | direction |
	| ------- | ------------| -------------- |
	| 1 | elfin_joint1| ccw |
	| -1 | elfin_joint1 | cw |
	| 2 | elfin_joint2 | ccw |
	| -2 | elfin_joint2 | cw |
	| 3 | elfin_joint3| ccw |
	| -3 | elfin_joint3 | cw |
	| 4 | elfin_joint4 | ccw |
	| -4 | elfin_joint4 | cw |
	| 5 | elfin_joint5| ccw |
	| -5 | elfin_joint5 | cw |
	| 6 | elfin_joint6 | ccw |
	| -6 | elfin_joint6 | cw |

------
### Published Topics:

* **/elfin_arm_controller/state (control_msgs/msg/JointTrajectoryControllerState)**  
The current status of the joints.

* **/enable_state (std_msgs/msg/Bool)**  
The servo status of the robot.  
true: enabled / false: disabled

* **/fault_state (std_msgs/msg/Bool)**  
The fault status of the robot.  
true: warning / false: no fault

* **elfin_basic_api/parameter_updates (dynamic_reconfigure/Config)**  
The value of the dynamic parameters of elfin_basic_api, e.g. velocity scaling.

* **/reference_link_name (std_msgs/msg/String)**  
The reference link in the calculations of the elfin_basic_api node

* **/end_link_name (std_msgs/msg/String)**  
The end link in the calculations of the elfin_basic_api node

------
### Services:

* **/get_reference_link (std_srvs/srv/SetBool)**  
You can get the reference link name of *elfin_basic_api* from the response of this service.

* **/get_end_link (std_srvs/srv/SetBool)** 
You can get the end link name of *elfin_basic_api* from the response of this service.

* **/stop_teleop (std_srvs/srv/SetBool)**  
Make the robot stop moving.

* **/get_txpdo (std_srvs/srv/SetBool)**  
You can get the content of TxPDOs from the response of this service.

* **/get_rxpdo (std_srvs/srv/SetBool)**  
You can get the content of RxPDOs from the response of this service.

* **/get_current_position (std_srvs/srv/SetBool)** 
You can get the count values of the current joint positions from the response of this service.

* **elfin_basic_api/set_parameters (dynamic_reconfigure/Reconfigure)**  
Set the dynamic parameters of elfin_basic_api, e.g. velocity scaling  
example: set_parameters() in elfin_robot_bringup/script/set_velocity_scaling.py

* **/recognize_position (std_srvs/SetBool)**  
Recognize the position of joints.

* **/write_do (elfin_robot_msgs/srv/ElfinIODWrite)**
Write a value into DO  

* **/read_di (elfin_robot_msgs/srv/ElfinIODRead)**  
Read the value from DI  

* **/get_io_txpdo (std_srvs/srv/SetBool)**  
You can get the content of TxPDOs from the response of this service.

* **/get_io_rxpdo (std_srvs/srv/SetBool)**  
You can get the content of RxPDOs from the response of this service.

* **elfin_module_open_brake_slaveX(std_srvs/srv/SetBool)**  
When the module is not enabled, you can open the brake of the corresponding module using this service.  

* **elfin_module_close_brake_slaveX(std_srvs/srv/SetBool)**  
When the module is not enabled, you can close the brake of the corresponding module using this service.  
