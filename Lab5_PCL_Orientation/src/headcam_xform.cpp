//utility to convert tf to affine for head cam pose

#include<ros/ros.h>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_xform_utils"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    XformUtils xformUtils; //instantiate an object of XformUtils
    geometry_msgs::Pose object_pose, gripper_pose;
    geometry_msgs::PoseStamped gripper_pose_stamped;

    //define a pose with origin (x,y,z) = (0.5, -0.35, -0.155).
    object_pose.position.x = 0.244;
    object_pose.position.y = 0.020;
    object_pose.position.z = 0.627;
    //and orientation (x,y,z,w) = (0, 0, 0.842, 0.54)
    object_pose.orientation.x = 0.679;
    object_pose.orientation.y = -0.679;
    object_pose.orientation.z = 0.198;
    object_pose.orientation.w = -0.198;
    ROS_INFO("headcam pose w/rt torso_lift_link: ");
    xformUtils.printPose(object_pose);
    Eigen::Affine3d object_affine;
    //use XformUtils to convert from pose to affine:
    object_affine = xformUtils.transformPoseToEigenAffine3d(object_pose);
    cout << "xform_affine origin: " << object_affine.translation().transpose() << endl;
    cout << "xform_affine R matrix: " << endl;
    cout << object_affine.linear() << endl;

  

}
