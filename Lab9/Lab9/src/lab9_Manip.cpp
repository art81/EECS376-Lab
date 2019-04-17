// traj_action_client_pre_pose: 
// uses right and left arm trajectory action servers to send robot to a hard-coded pre pose

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Lab9/baxter_trajectory_streamer.h>
#include <Lab9/trajAction.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_fk_ik/baxter_kinematics.h>  //Baxter kinematics
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <object_manipulation_properties/object_ID_codes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count=0;
void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const Lab9::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}

geometry_msgs::PoseStamped g_perceived_object_pose;
int g_found_object_code;
bool objectFound_ = false;
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state, const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
        g_perceived_object_pose= result->object_pose;
        ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 							   g_perceived_object_pose.pose.position.y,
                 							   g_perceived_object_pose.pose.position.z);

        ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 											g_perceived_object_pose.pose.orientation.y,
                 											g_perceived_object_pose.pose.orientation.z,
                 											g_perceived_object_pose.pose.orientation.w);
        objectFound_ = true;
    } else {
        ROS_WARN("object not found!");
        objectFound_ = false;
    }
}

void sendTraJ(std::vector<Eigen::VectorXd> des_path_right, Baxter_traj_streamer baxter_traj_streamer, actionlib::SimpleActionClient<Lab9::trajAction> &right_arm_action_client) {
    //convert from vector of 7dof poses to trajectory message  
    trajectory_msgs::JointTrajectory des_trajectory_right;
    baxter_traj_streamer.stuff_trajectory_right_arm(des_path_right, des_trajectory_right); 
    
    // goal objects compatible with the arm servers
    Lab9::trajGoal goal_right;
    goal_right.trajectory = des_trajectory_right; 

    ROS_INFO("sending goals to right arm: ");
    g_done_count = 0;
    right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb); 
    while (g_done_count < 1) {
       ROS_INFO("waiting to finish pre-pose..");
       ros::Duration(1.0).sleep();
    }
}

Vectorq7x1 pickCorrectJoints(std::vector<Vectorq7x1> &solutions, Eigen::VectorXd currJoints) {
    std::vector<double> scores;

    double c1 = 50;
    double c2 = 20;
    double c3 = 10;
    double c4 = 10;
    double c5 = 5;
    double c6 = 2;
    double c7 = 2;

    for(int i = 0;i < solutions.size();i++) {
        scores.push_back(0.0);
    }

    for(int i = 0;i < solutions.size(); i++) {
        scores[i] = (c1*abs(solutions[i][0] - currJoints(0))) +
                    (c2*abs(solutions[i][1] - currJoints(1))) +
                    (c3*abs(solutions[i][2] - currJoints(2))) +
                    (c4*abs(solutions[i][3] - currJoints(3))) +
                    (c5*abs(solutions[i][4] - currJoints(4))) +
                    (c6*abs(solutions[i][5] - currJoints(5))) +
                    (c7*abs(solutions[i][6] - currJoints(6)));
        ROS_INFO("Score at index %d: %f", i, scores[i]);
    }

    double min = 10000;
    int index = 0;

    for(int i = 0;i < scores.size();i++) {
        if(scores[i] < min) {
            min = scores[i];
            index = i;
        }
    }

    ROS_INFO("Returned index: %d", index);
    return solutions[index];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle 

    actionlib::SimpleActionClient<Lab9::trajAction> right_arm_action_client("rightArmTrajActionServer", true);
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

    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();  //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }       
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("object_finder_action_service", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 

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

    //Inverse Kinematics Stuff
    Baxter_IK_solver baxter_ik_solver;
    Eigen::Vector3d n_des,t_des,b_des;

    Eigen::Affine3d a_tool_des; // expressed in DH frame  
    
    //Sending goal to object finder to find the gearbox bottom
    object_finder::objectFinderGoal object_finder_goal;
    object_finder_goal.object_id = ObjectIdCodes::GEARBOX_BOTTOM;
    object_finder_goal.known_surface_ht = false;
    
    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
    object_finder_ac.waitForResult();
    
    drop_pose <<  0.0786165153791,-1.34721862696,1.36600989161,1.80933033931,0.360101989956,1.72764586236,-2.31362652333; 
    
    if(objectFound_) {
    	//Now g_perceived_object_pose is populated!
    	
    	Eigen::Vector3d p_des;
    	p_des[0] = g_perceived_object_pose.pose.position.x; //X-Val
    	p_des[1] = g_perceived_object_pose.pose.position.y; //Y-Val
    	p_des[2] = g_perceived_object_pose.pose.position.z; //Z-Val
    	a_tool_des.translation() = p_des;
    	
    	b_des<<0,0,-1; //tool flange pointing down
		n_des<<0,0,1; //x-axis pointing forward...arbitrary
		t_des = b_des.cross(n_des); //consistent right-hand frame

		Eigen::Matrix3d R_des;
		R_des.col(0) = n_des;
		R_des.col(1) = t_des;
		R_des.col(2) = b_des;
		a_tool_des.linear() = R_des;
		
		std::vector<Vectorq7x1> q_solns;
		int nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);

		q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd();

		if(nsolns > 0) {
		    grip_pose = pickCorrectJoints(q_solns, q_vec_right_arm);
		} else {
		    ROS_WARN("Nah dude, no solutions");
		    return 0;
		}

		a_tool_des.translation()[2] += 0.3;
		nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);

		if(nsolns > 0) {
		    approach_pose = pickCorrectJoints(q_solns, q_vec_right_arm);
		} else {
		    ROS_WARN("Nah dude, no solutions");
		    return 0;
		}
    	
    	gripper_publisher_right.publish(gripper_cmd_open); //Open the gripper to start
    	ros::Duration(1.0).sleep();
    	
    	q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd(); //Current pose of Right Arm

        des_path_right.push_back(q_vec_right_arm); //start from current pose
        des_path_right.push_back(drop_pose);
        des_path_right.push_back(approach_pose); 
        des_path_right.push_back(grip_pose); 
        //sendTraJ(des_path_right, baxter_traj_streamer, right_arm_action_client);
        des_path_right.clear();
    	
    } else {
    	//Not Found!
    }
    
	/*
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
    */
    
    ros::spinOnce();
    return 0;
}

