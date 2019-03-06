# Lab 6
## Example usage

#Open-Loop Control
roslaunch worlds glennan_world.launch
rosrun mobot_pub_des_state open_loop_controller
rosrun mobot_pub_des_state mobot_pub_des_state
rosrun mobot_pub_des_state pub_des_state_path_client

#Linear Steering Control
roslaunch worlds glennan_world.launch
rosrun lin_steering lin_steering_wrt_odom
rosrun mobot_pub_des_state mobot_pub_des_state
rosrun mobot_pub_des_state pub_des_state_path_client
