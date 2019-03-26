# Lab 7
## Example usage

#Running with AMCL/Odom
roslaunch worlds glennan_world.launch
rosrun lin_steering lin_steering_wrt_amcl_and_odom
rosrun amcl amcl
rosrun map_server map_server glennan_2nd_flr_model_map.yaml (**cd where the .yaml file is**)
rosrun mobot_pub_des_state mobot_pub_des_state
rosrun mobot_pub_des_state pub_des_state_path_client
