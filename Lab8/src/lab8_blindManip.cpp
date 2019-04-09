// traj_action_client_pre_pose: 
// uses right and left arm trajectory action servers to send robot to a hard-coded pre pose

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <baxter_trajectory_streamer/trajAction.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count=0;

void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

void sendTraJ(std::vector<Eigen::VectorXd> des_path_right, Baxter_traj_streamer baxter_traj_streamer, actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> &right_arm_action_client) {
    //convert from vector of 7dof poses to trajectory message  
    trajectory_msgs::JointTrajectory des_trajectory_right;
    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
    
    // goal objects compatible with the arm servers
    baxter_trajectory_streamer::trajGoal goal_right;
    goal_right.trajectory = des_trajectory_right; 

    ROS_INFO("sending goals to right arm: ");
    g_done_count = 0;
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); 
    while (g_done_count < 1) {
       ROS_INFO("waiting to finish pre-pose..");
       ros::Duration(1.0).sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle 

    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
    // attempt to connect to the servers:
    ROS_INFO("waiting for right-arm server: ");
    bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on right-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;         

    Eigen::VectorXd q_pre_pose_right;
    Eigen::VectorXd q_vec_right_arm;
    std::vector<Eigen::VectorXd> des_path_right;

    ros::Publisher gripper_publisher_right = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1, true); 
    baxter_core_msgs::EndEffectorCommand gripper_cmd_open, gripper_cmd_close;

    gripper_cmd_open.id = 65538;
    gripper_cmd_open.command ="go";
    //gripper_cmd_open.args = "{'position': 100.0}'"; //oops
    gripper_cmd_open.args = "{\"position\": 100.0}";
    gripper_cmd_open.sender = "gripper_publisher";
    gripper_cmd_open.sequence = 2;
    
    gripper_cmd_close.id = 65538;
    gripper_cmd_close.command ="go";
    //gripper_cmd_close.args = "{'position': 0.0}'"; //oops
    gripper_cmd_close.args = "{\"position\": 0.0}";
    gripper_cmd_close.sender = "gripper_publisher"; 
    gripper_cmd_close.sequence = 3;

    //hard-Coded joint angles for picking up the block!!

    Eigen::VectorXd approach_pose, grip_pose, drop_pose;
    approach_pose.resize(7);
    grip_pose.resize(7);
    drop_pose.resize(7);

    approach_pose <<  1.21874773597,-0.506213660002,-0.260776733941,1.36064095885,0.333257326168,0.662679700366,0.118116520667; 
    grip_pose     <<  1.16620889399,-0.040266995682,-0.194815560061,0.790000105761,0.318684508683,0.823747683094,0.125019434213; 
    drop_pose     <<  0.0786165153791,-1.34721862696,1.36600989161,1.80933033931,0.360101989956,1.72764586236,-2.31362652333; 

    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();  //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }
    
    gripper_publisher_right.publish(gripper_cmd_open); //Open the gripper to start
    ros::Duration(1.0).sleep();

    while(ros::ok()) {
        q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd(); //Current pose of Right Arm

        des_path_right.push_back(q_vec_right_arm); //start from current pose
        des_path_right.push_back(drop_pose);
        des_path_right.push_back(approach_pose); 
        des_path_right.push_back(grip_pose); 
        sendTraJ(des_path_right, baxter_traj_streamer, right_arm_action_client);
        des_path_right.clear();

        //Enable the Gripper
        gripper_publisher_right.publish(gripper_cmd_close);
        ros::Duration(1.0).sleep();

        des_path_right.push_back(approach_pose); 
        des_path_right.push_back(drop_pose); 
        sendTraJ(des_path_right, baxter_traj_streamer, right_arm_action_client);
        des_path_right.clear();

        ros::Duration(1.0).sleep();

        des_path_right.push_back(drop_pose);
        des_path_right.push_back(approach_pose); 
        des_path_right.push_back(grip_pose);
        sendTraJ(des_path_right, baxter_traj_streamer, right_arm_action_client);
        des_path_right.clear();

        //Disable the Gripper
        gripper_publisher_right.publish(gripper_cmd_open);
        ros::Duration(1.0).sleep();

        des_path_right.push_back(grip_pose); 
        des_path_right.push_back(approach_pose); 
        sendTraJ(des_path_right, baxter_traj_streamer, right_arm_action_client);
    }
    
    ros::spinOnce();
    return 0;
}

