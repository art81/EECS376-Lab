#Lab 9 Package for Perception based Manipulation of Block
This package contains a library of useful utilities, baxter_trajectory_streamer.cpp, for controlling Baxter arms.
Also, two action servers: left_arm_as and rt_arm_as, which accept trajectory goal messages, using the action
message defined in this package.  The interpolators accept trajectories and interpolate linearly between successive
joint-space points.

An example action client, pre_pose (from traj_action_client_pre_pose.cpp), uses the baxter_trajectory_streamer library
and sends trajectory goals to the left and right arm action servers.  The example trajectories start from the
current arm poses and go to hard-coded goal poses.

## Example usage
baxter_master (in every terminal)
roslaunch Lab9 start_baxter.launch
rosrun Lab9 rt_arm_as
rosrun Lab9 lab9_Manip
rosrun object_finder object_finder_as
