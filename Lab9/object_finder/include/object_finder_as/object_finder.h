//some ROS headers:
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h> 

#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl_ros/transforms.h>

//point-cloud library headers; likely don't need all these
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/crop_box.h>

//headers for using OpenCV functions
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/utility.hpp>
//#include "opencv2/highgui.hpp"

#include <iostream>
#include <stdio.h> 

//CWRU stuff:
#include <pcl_utils/pcl_utils.h>
#include <object_finder/objectFinderAction.h>
#include <xform_utils/xform_utils.h>
#include <part_codes/part_codes.h>

using namespace std;
using namespace cv;

Eigen::Affine3f g_affine_headcam_wrt_base;
XformUtils xformUtils;
//magic numbers for filtering pointcloud points:
//specify the min and max values, x,y, znd z, for points to retain...
// as expressed in the robot's torso frame (torso_lift_link)
//orig wsn numbers:
/*
const float MIN_X = 0.4; //include points starting 0.4m in front of robot
const float MAX_X = 0.8; //include points out to 0.9m in front of robot
const float MIN_Y = -0.3; //include points starting -0.5m to left of robot
const float MAX_Y = 0.3; //include points up to 0.5m to right of robot 
const float MIN_Z = -0.05; //2cm above the table top
const float MAX_Z = 0.1; //consider points up to this height w/rt torso frame

const float TABLE_TOP_MIN = -0.1;
const float TABLE_TOP_MAX = 0.2; //0.05;
*/

const float MIN_X = 0.25; //include points starting 0.4m in front of robot
const float MAX_X = 1.0; //include points out to 0.9m in front of robot
const float MIN_Y = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ = 0.0;//0.02; //box filter from this height above the table top
const float MAX_DZ = 0.2; //consider points up to this height above table top

const float TABLE_TOP_MIN = -0.5;
const float TABLE_TOP_MAX = 0.0;

//choose, e.g., resolution of 5mm, so 100x200 image for 0.5m x 1.0m pointcloud
// adjustable--> image  size
// try 400 pix/m...works fine, i.e. 2.5mm resolution
//const float PIXELS_PER_METER = 450.0; //200.0;
//Chris's value:
const float PIXELS_PER_METER = 400.0; //200.0;

const float TABLE_GRASP_CLEARANCE = 0.01; //add this much to table  top height for gripper clearance

const float MIN_BLOB_PIXELS = 600; //must have  at least this many pixels to  preserve as a blob; gearbox top/bottom has about 900 pts
const float MIN_BLOB_AVG_HEIGHT = 5.0; //avg z-height must be  at least this many mm to preserve as blob



const int Nu = (int) ((MAX_X - MIN_X) * PIXELS_PER_METER);
const int Nv = (int) ((MAX_Y - MIN_Y) * PIXELS_PER_METER);

Mat_<uchar> g_bw_img(Nu, Nv);
Mat_<int> g_labelImage(Nu, Nv);
Mat g_dst(g_bw_img.size(), CV_8UC3);

//SHOULD move these to class member data
vector<float> g_x_centroids, g_y_centroids;
vector<float> g_x_centroids_wrt_robot, g_y_centroids_wrt_robot;
vector<float> g_avg_z_heights;
vector<float> g_npts_blobs;
vector<float> g_orientations;
//Global Variable matrix to hold quaternions
vector<geometry_msgs::Quaternion> g_vec_of_quat; 


Eigen::Affine3f affine_cam_wrt_torso_;

class ObjectFinder {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;

    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    //PclUtils pclUtils_;
    tf::TransformListener* tfListener_;
    void initializeSubscribers();
    void initializePublishers();


    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    //bool find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose);
    //bool find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose);
    //float find_table_height();

    double surface_height_;
    bool found_surface_height_;
    bool got_headcam_image_;


    vector<int> indices_;
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing

    Eigen::Affine3f affine_cam_wrt_torso_;
    
    Eigen::Vector3f major_axis_,centroid_,plane_normal_;
    vector<int> viable_labels_;


public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
    void headcamCB(const sensor_msgs::PointCloud2ConstPtr& cloud);

    XformUtils xformUtils_;
    ros::Subscriber pointcloud_subscriber_; //use this to subscribe to a pointcloud topic
    void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
            Eigen::Vector4f box_pt_max, vector<int> &indices);


    void blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights,
        vector<float> &npts_blobs,
        vector<int> &viable_labels);

    Eigen::Affine3f compute_affine_cam_wrt_torso_lift_link(void);
    float find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz);
    void convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices);
    sensor_msgs::PointCloud2 ros_cloud_, downsampled_cloud_, ros_box_filtered_cloud_, ros_crop_filtered_cloud_, ros_pass_filtered_cloud_; //here are ROS-compatible messages
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCam_clr_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_filtered_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_filtered_cloud_ptr_; //(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Publisher pubCloud_; // = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubDnSamp_; // = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt_; // = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);  
    ros::Publisher pubCropFilt_;
    ros::Publisher pubPassFilt_;



    bool find_gearbox_top(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
            vector<float> avg_z_heights, vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses);
    bool find_gearbox_bottom(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
            vector<float> avg_z_heights, vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses);
    bool find_totes(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
            vector<float> avg_z_heights, vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses); 
    
    void find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion);

    double findOrientation(int labelNum);
    
};
