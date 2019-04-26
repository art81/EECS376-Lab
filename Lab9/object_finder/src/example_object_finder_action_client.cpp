// example_object_finder_action_client: 
// wsn, April, 2016
// illustrates use of object_finder action server called "objectFinderActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <part_codes/part_codes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;


std::vector <geometry_msgs::PoseStamped> g_perceived_object_poses;
ros::Publisher *g_pose_publisher;

int g_found_object_code;
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    geometry_msgs::PoseStamped perceived_object_pose;
    g_perceived_object_poses.clear();
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        //ROS_INFO("found objects!");
        int n_objects = result->object_poses.size();
        for (int i_object=0;i_object<n_objects;i_object++) {
         //g_perceived_object_pose= result->object_pose; //MAKE MORE GENERAL FOR  POSES
            ROS_INFO("object %d: ",i_object);
            perceived_object_pose = result->object_poses[i_object];
            ROS_INFO("   pose x,y,z = %f, %f, %f",perceived_object_pose.pose.position.x,
                 perceived_object_pose.pose.position.y,
                 perceived_object_pose.pose.position.z);

            ROS_INFO("   quaternion x,y,z, w = %f, %f, %f, %f",perceived_object_pose.pose.orientation.x,
                 perceived_object_pose.pose.orientation.y,
                 perceived_object_pose.pose.orientation.z,
                 perceived_object_pose.pose.orientation.w);
            g_perceived_object_poses.push_back(perceived_object_pose);
        }
         //g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_finder_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("object_finder_action_service", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    object_finder::objectFinderGoal object_finder_goal;
    //object_finder::objectFinderResult object_finder_result;
    int part_index;
    cout<<"enter index for desired part:"<<endl<<"GEARBOX_TOP = 1"<<endl<<"GEARBOX_BOTTOM = 2"<<endl<<"BOLT = 3"<<endl<<"SMALL_GEAR = 4"<<endl<<"LARGE_GEAR = 5"<<endl<<"TOTE=6"<<endl;
    cout<<"enter  0 for fake part (hardcoded pose: "<<endl;
    cout<<"enter part index: ";
    cin>>part_index;
    switch(part_index) {
        case 0: object_finder_goal.object_id = part_codes::part_codes::FAKE_PART;
            break;
        case 1: object_finder_goal.object_id = part_codes::part_codes::GEARBOX_TOP;
        break;
        case 2: object_finder_goal.object_id = part_codes::part_codes::GEARBOX_BOTTOM;
        break;
        case 3: object_finder_goal.object_id = part_codes::part_codes::BOLT;
        break;
        case 4: object_finder_goal.object_id = part_codes::part_codes::SMALL_GEAR;
        break;
        case 5: object_finder_goal.object_id = part_codes::part_codes::LARGE_GEAR;
        break;
        case 6: object_finder_goal.object_id = part_codes::part_codes::TOTE;
        break;
        default:
            ROS_WARN("unknown part code; quitting");
            return 1;
    }
       

     ROS_INFO("sending goal to find object: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        bool finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }       
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object(s)!");
        return 0;
    }    
    else {
        ROS_WARN("object not found!:");
        return 1;
    }

}

