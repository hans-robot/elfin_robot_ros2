 API 简介
=====
### Subscribed Topics:

* **/joint_goal (sensor_msgs/msg/JointState)**  
令机械臂规划到达指定臂型的路径并执行此路径  
example: function_pub_joints() in elfin_basic_api/scripts/test.py

* **/cart_goal (geometry_msgs/msg/PoseStamped)**  
令机械臂规划到达指定空间位置的路径并执行此路径  
example: function_pub_cart() in elfin_basic_api/scripts/test.py

* **/cart_path_goal (geometry_msgs/msg/PoseArray)**  
令机械臂规划一条路径，以直线方式依次经过各个指定的点，并执行此路径  
example: function_pub_cart_path() in elfin_basic_api/scripts/test.py

* **/elfin_arm_controller/joint_trajectory (trajectory_msgs/msg/JointTrajectory)**  
本消息内容为一条轨迹，可令机械臂沿着这条轨迹运动
example: function_pub_trajectory() in elfin_basic_api/scripts/test.py

* **/elfin_teleop_joint_cmd_no_limit (std_msgs/msg/Int64)**  
本指令为调试机器时使用的指令，不建议客户使用。发送本指令后，机械臂的特定关节会向一个方向移动一点距离，连续发送就会连续运动。  
消息内容含义如下：

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
反映机械臂各个关节的状态

* **/enable_state (std_msgs/msg/Bool)**  
反映此时Elfin机械臂上电机的使能状态。  
true: 使能  / false: 未使能

* **/fault_state (std_msgs/msg/Bool)**  
反映此时Elfin机械臂上电机的报错状态。  
true: 有报错  / false: 无报错

* **/reference_link_name (std_msgs/msg/String)**  
elfin_basic_api节点计算时所使用的参考基坐标系的名字

* **/end_link_name (std_msgs/msg/String)**  
elfin_basic_api节点计算时所使用的末端坐标系的名字

------
### Services:

* **/get_reference_link (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含elfin_basic_api规划时的参考基坐标系名

* **/get_end_link (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含elfin_basic_api规划时的末端坐标系名

* **/stop_teleop (std_srvs/srv/SetBool)**  
呼叫本服务会令机械臂停止运动

* **/get_txpdo (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含机械臂上从站的txpdo信息

* **/get_rxpdo (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含机械臂上从站的rxpdo信息

* **/get_current_position (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含机械臂各轴的位置的编码器值  

* **/recognize_position (std_srvs/SetBool)**  
呼叫本服务可使机械臂进行姿态识别

* **/write_do (elfin_robot_msgs/srv/ElfinIODWrite)**  
向DO中写入内容  

* **/read_di (elfin_robot_msgs/srv/ElfinIODRead)**  
从DI中读取内容  

* **/get_io_txpdo (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含IO从站的txpdo信息

* **/get_io_rxpdo (std_srvs/srv/SetBool)**  
呼叫本服务得到的反馈信息中会包含IO从站的rxpdo信息

* **elfin_module_open_brake_slaveX(std_srvs/srv/SetBool)**  
在模组未使能情况下，呼叫本服务可以打开相应模组的抱闸  

* **elfin_module_close_brake_slaveX(std_srvs/srv/SetBool)**  
在模组未使能情况下，呼叫本服务可以关闭相应模组的抱闸  
