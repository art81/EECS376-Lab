// action server to respond to perception requests
// Wyatt Newman, 2/2019
#include<object_finder/object_finder.h>
/*
#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl_utils/pcl_utils.h>
#include <pcl/filters/crop_box.h>
#include <object_finder/objectFinderAction.h>
#include <xform_utils/xform_utils.h>
#include <part_codes/part_codes.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "find_tote_fncs.cpp"



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

//Chris's numbers:
/*
const float MIN_X = 0.25; //include points starting 0.4m in front of robot
const float MAX_X = 0.8; //include points out to 0.9m in front of robot
const float MIN_Y = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y = 0.7; //include points up to 0.5m to right of robot
const float MIN_Z = -0.05; //2cm above the table top
const float MAX_Z = 0.14; //consider points up to this height w/rt torso frame

const float TABLE_TOP_MIN = -0.1;
const float TABLE_TOP_MAX = 0.1;

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
Mat dst(g_bw_img.size(), CV_8UC3);

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
            vector<float> &avg_z_heights, vector<float> &npts_blobs);
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
    bool find_totes(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
            vector<float> avg_z_heights, vector<float> npts_blobs, float table_height, vector<geometry_msgs::PoseStamped> &object_poses); 
    
    void find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion);
    
};
*/

ObjectFinder::ObjectFinder() :
object_finder_as_(nh_, "object_finder_action_service", boost::bind(&ObjectFinder::executeCB, this, _1), false), pclCam_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
box_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>),
transformed_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>), crop_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>),
pass_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>) //(new pcl::PointCloud<pcl::PointXYZRGB>);
{
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation


    object_finder_as_.start(); //start the server running
    tfListener_ = new tf::TransformListener; //create a transform listener
    found_surface_height_ = false;
    got_headcam_image_ = false;
    initializeSubscribers();
    initializePublishers();
    affine_cam_wrt_torso_ = compute_affine_cam_wrt_torso_lift_link();

    ROS_INFO("waiting for image data");
    while (!got_headcam_image_) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        got_headcam_image_ = true;
    }
}

void ObjectFinder::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    pointcloud_subscriber_ = nh_.subscribe("/head_camera/depth_registered/points", 1, &ObjectFinder::headcamCB, this);
    // add more subscribers here, as needed
}

void ObjectFinder::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("/headcam_pointcloud", 1, true);
    pubDnSamp_ = nh_.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1, true);
    pubBoxFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1, true);
    pubCropFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("crop_filtered_pcd", 1, true);
    pubPassFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("pass_filtered_pcd", 1, true);


}

void ObjectFinder::headcamCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_headcam_image_) { // once only, to keep the data stable
        ROS_INFO("got new image");
        pcl::fromROSMsg(*cloud, *pclCam_clr_ptr_);
        ROS_INFO("image has  %d * %d points", pclCam_clr_ptr_->width, pclCam_clr_ptr_->height);
        got_headcam_image_ = true;
    }
}

void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
        Eigen::Vector4f box_pt_max, vector<int> &indices) {
    /**/
    //int npts = input_cloud_ptr->points.size();
    //Eigen::Vector3f pt;
    indices.clear();
    cout << "box min: " << box_pt_min.transpose() << endl;
    cout << "box max: " << box_pt_max.transpose() << endl;
    /*
    for (int i = 0; i < npts; ++i) {
        pt = input_cloud_ptr->points[i].getVector3fMap();

        //check if in the box:
        if ((pt[0] > box_pt_min[0])&&(pt[0] < box_pt_max[0])&&(pt[1] > box_pt_min[1])&&(pt[1] < box_pt_max[1])&&(pt[2] > box_pt_min[2])&&(pt[2] < box_pt_max[2])) {
            //passed box-crop test; include this point
            //ROS_INFO("point passes test");
            //cout<<"pt passed test: "<<pt.transpose()<<endl;
            indices.push_back(i);
        }
    }
     */
    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud(input_cloud_ptr);
    cropFilter.setMin(box_pt_min);
    cropFilter.setMax(box_pt_max);

    cropFilter.filter(indices);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *crop_filtered_cloud_ptr_);
    //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*crop_filtered_cloud_ptr_, ros_crop_filtered_cloud_); //convert to ros message for publication and display        


    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;
}


//This fnc was used to confirm the table height relative to torso frame;
// It is no longer needed, since the table height, once found, can be hard coded
// i.e., 7cm below the torso-frame origin

float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz) {
    vector<int> indices;
    /*
     Eigen::Vector4f minPoint; 
      minPoint[0]=MIN_X;  // define minimum point x 
      minPoint[1]=MIN_Y;  // define minimum point y 
      minPoint[2]=TABLE_TOP_MIN;  // define minimum point z 
     Eigen::Vector4f maxPoint; 
      maxPoint[0]=MAX_X;  // define max point x 
      maxPoint[1]=MAX_Y;  // define max point y 
      maxPoint[2]=TABLE_TOP_MAX;  // define max point z 
    pcl::CropBox<pcl::PointXYZRGB> cropFilter; 
        cropFilter.setInputCloud (input_cloud_ptr); 
               cropFilter.setMin(minPoint); 
               cropFilter.setMax(maxPoint); 
               //cropFilter.setTranslation(boxTranslatation); 
               //cropFilter.setRotation(boxRotation); 
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>);           
    //crop_filtered_cloud_ptr_ = new PointCloud<pcl::PointXYZRGB>;
        cropFilter.filter (indices); 
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *crop_filtered_cloud_ptr_);
        //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*crop_filtered_cloud_ptr_, ros_crop_filtered_cloud_); //convert to ros message for publication and display    
     */
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (float z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();

        if (npts > 0) ROS_INFO("z=%f; npts = %d", z, npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f", npts_max, z_table);

    pass.setFilterLimits(z_table - 0.5 * dz, z_table + 0.5 * dz);
    //pass.filter (*pass_filtered_cloud_ptr_);
    pass.filter(indices);

    //OOPS: want to select these from transformed point cloud?  (same frame as other displays)
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *pass_filtered_cloud_ptr_);
    //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*pass_filtered_cloud_ptr_, ros_pass_filtered_cloud_); //convert to ros message for publication and display

    return z_table;
}

//hard-coded xform for Fetch w/ assumed torso lift and head angle:
//assuming head is tilted down 1.0rad, this is the head-camera frame
// with respect to the torso_lift_link frame, expressed as an Eigen::Affine

//should be  transform from  head_camera_rgb_optical_frame to torso_lift_link
//example Fetch rosbags have: rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame

/* Translation: [0.238, 0.019, 0.656]
- Rotation: in Quaternion [-0.660, 0.673, -0.242, 0.230]
            in RPY (radian) [-2.461, -0.009, -1.593]
            in RPY (degree) [-140.999, -0.509, -91.274]
 */

Eigen::Affine3f ObjectFinder::compute_affine_cam_wrt_torso_lift_link(void) {
    //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_torso;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam << 0.238, 0.019, 0.656; //0.244, 0.02, 0.627;
    affine_cam_wrt_torso.translation() = O_cam;
    Eigen::Vector3f nvec, tvec, bvec;
    //magic numbers, as determined by rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame
    // when running robot in Gazebo with headcam tilted down 1.0rad
    //from Fetch bag: 
    //Eigen::Quaternionf q;
    cout << "enter tilt angle (0.897=head_tilt_joint for fetch data): ";
    float angle;
    cin>>angle;
    Eigen::Quaternionf q(Eigen::AngleAxisf{angle, Eigen::Vector3f
        {0, 1, 0}});
    //    outputAsMatrix(Eigen::Quaterniond{Eigen::AngleAxisd{angle, Eigen::Vector3d{0, 1, 0}}});
    /*
    q.x() = -0.660; 
    q.y() = 0.673; 
    q.z() =-0.242; 
    q.w() = 0.230;  
     * */
    //R_cam = q.normalized().toRotationMatrix();

    nvec << 0, -1, 0;
    tvec << -sin(angle), 0, -cos(angle);
    bvec << cos(angle), 0, -sin(angle);
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;

    affine_cam_wrt_torso.linear() = R_cam;
    return affine_cam_wrt_torso;
}

//given a binary image in bw_img, find and label connected regions  (blobs)
// labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot
void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) {
    //ROS_INFO("starting identification of plane from data: ");
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size
    
    // first compute the centroid of the data:
    //Eigen::Vector3f centroid; // make this member var, centroid_
    centroid_ = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    //centroid = compute_centroid(points_mat);
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid_ += points_mat.col(ipt); //add all the column vectors together
    }
    centroid_ /= npts; //divide by the number of points to get the centroid    
    cout<<"centroid: "<<centroid_.transpose()<<endl;


    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; //we'll extract the eigenvalues to here
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    //cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    //cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    //cout << "The matrix of eigenvectors, V, is:" << endl;
    //cout<< es3d.eigenvectors() << endl << endl;
    //cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    //however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals = es3f.eigenvalues().real(); // grab just the real parts
    //cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

    double min_lambda = evals[0]; //initialize the hunt for min eval
    double max_lambda = evals[0]; // and for max eval
    //Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    //Eigen::Vector3f evec0, evec1, evec2; //, major_axis; 
    //evec0 = es3f.eigenvectors().col(0).real();
    //evec1 = es3f.eigenvectors().col(1).real();
    //evec2 = es3f.eigenvectors().col(2).real();  
    
    
    //((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec = 
    // = evec'*CoVar*evec = evec'*lambda*evec = lambda
    // min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
    // max lambda is associated with direction of major axis
    
    //sort the evals:
    
    //complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal_ = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions
    
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal = 0;
    int i_major_axis=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            plane_normal_ = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
        }        
    }

	float x_component = major_axis_(0);
	float y_component = -major_axis_(1);
	orientation = atan2(y_component,x_component) - M_PI/2;

	quaternion = xformUtils.convertPlanarPsi2Quaternion(orientation);
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    //cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    //cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
    //cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    //cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;  
    
    //what is the correct sign of the normal?  If the data is with respect to the camera frame,
    // then the camera optical axis is z axis, and thus any points reflected must be from a surface
    // with negative z component of surface normal
    if (plane_normal_(2)>0) plane_normal_ = -plane_normal_; // negate, if necessary
    
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    
    //ROS_INFO("major_axis: %f, %f, %f",major_axis_(0),major_axis_(1),major_axis_(2));
    //ROS_INFO("plane normal: %f, %f, %f",plane_normal(0),plane_normal(1),plane_normal(2));
}



//given a binary image in g_bw_img, find and label connected regions  (blobs)
// g_labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot
//OPERATES ON GLOBAL VARS g_bw_img and g_labelImage

void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights,
        vector<float> &npts_blobs) {

    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    //openCV function to do all the  hard work
    int nLabels = connectedComponents(g_bw_img, g_labelImage, 8); //4 vs 8 connected regions


    ROS_INFO("found %d blobs", nLabels);
    vector<float> temp_x_centroids, temp_y_centroids, temp_avg_z_heights, temp_npts_blobs;
    //g_x_centroids.resize(nLabels);
    //g_y_centroids.resize(nLabels);
    temp_y_centroids.resize(nLabels);
    temp_x_centroids.resize(nLabels);
    temp_avg_z_heights.resize(nLabels);
    temp_npts_blobs.resize(nLabels);
    for (int label = 0; label < nLabels; ++label) {
        temp_y_centroids[label] = 0.0;
        temp_x_centroids[label] = 0.0;
        temp_avg_z_heights[label] = 0.0;
        temp_npts_blobs[label] = 0.0;
    }

    //compute centroids
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            int label = g_labelImage.at<int>(r, c);
            temp_y_centroids[label] += c; //robot y-direction corresponds to columns--will negate later
            temp_x_centroids[label] += r; //robot x-direction corresponds to rows--will negate later
            temp_npts_blobs[label] += 1.0;
            double zval = (float) g_bw_img(r, c);
            temp_avg_z_heights[label] += zval; //check the  order of this
        }
    }
    //cout<<"checkpoint 1"<<endl;
    for (int label = 0; label < nLabels; ++label) {
        ROS_INFO("label %d has %f points", label, temp_npts_blobs[label]);
        temp_y_centroids[label] /= temp_npts_blobs[label];
        temp_x_centroids[label] /= temp_npts_blobs[label];
        temp_avg_z_heights[label] /= temp_npts_blobs[label];
        //ROS_INFO("label %d has centroid %f, %f:",label,temp_x_centroids[label],temp_y_centroids[label]);
    }
    cout << "filtering by height and area..." << endl;
    //filter to  keep only blobs that are high enough and large enough
    for (int label = 0; label < nLabels; ++label) {
        ROS_INFO("label %d has %d points and  avg height %f:", label, (int) temp_npts_blobs[label], temp_avg_z_heights[label]);
        if (temp_avg_z_heights[label] > MIN_BLOB_AVG_HEIGHT) {
            if (temp_npts_blobs[label] > MIN_BLOB_PIXELS) {
                //ROS_INFO("label %d has %f points:",label,temp_npts_blobs[label]);
                x_centroids_wrt_robot.push_back(temp_x_centroids[label]);
                y_centroids_wrt_robot.push_back(temp_y_centroids[label]);
                avg_z_heights.push_back(temp_avg_z_heights[label]);
                npts_blobs.push_back(temp_npts_blobs[label]);
                //ROS_INFO("label %d has %f points, avg height %f and centroid %f, %f:",label,temp_npts_blobs[label],temp_avg_z_heights[label],
                //        temp_x_centroids[label],temp_y_centroids[label]);                    
            }
        }
    }


    //colorize the regions and display them:
    //also compute centroids;
    nLabels = npts_blobs.size(); //new number of labels, after filtering
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0); //make the background black
    //assign random color to each region label
    for (int label = 1; label < nLabels; ++label) {
        colors[label] = Vec3b((rand()&255), (rand()&255), (rand()&255));
    }

    //convert to robot coords:
    //convert to dpixel_x, dpixel_y w/rt image centroid, scale by pixels/m, and add offset from PCL cropping, x,y
    for (int label = 0; label < nLabels; ++label) {
        x_centroids_wrt_robot[label] = ((dst.rows / 2) - x_centroids_wrt_robot[label]) / PIXELS_PER_METER + (MIN_X + MAX_X) / 2.0;
        y_centroids_wrt_robot[label] = ((dst.cols / 2) - y_centroids_wrt_robot[label]) / PIXELS_PER_METER + (MIN_Y + MAX_Y) / 2.0;
        ROS_INFO("label %d has %d points, avg height %f, and centroid w/rt robot: %f, %f:", label, (int) npts_blobs[label], avg_z_heights[label], x_centroids_wrt_robot[label], y_centroids_wrt_robot[label]);
    }
   //xxx
        g_orientations.clear();
        g_vec_of_quat.clear();
        Eigen::Vector3f e_pt;
	for(int label = 1; label < nLabels; label++){
		int npts_blob = npts_blobs[label];
		Eigen::MatrixXf blob(3, npts_blob);
		int col_num = 0;
		for (int r = 0; r < dst.rows; ++r) {
			for (int c = 0; c < dst.cols; ++c){
				int label_num = g_labelImage.at<int>(r,c);
				if(label_num == label){
					e_pt<<c,r,0.0;
					blob.col(col_num) = e_pt;
					col_num++;
				}
			}
		}
		float angle;
		geometry_msgs::Quaternion quat;
		find_orientation(blob, angle, quat);
		//add pi/2 to factor in rotated camera frame wrt robot
		g_orientations.push_back(angle);
		g_vec_of_quat.push_back(quat);
        }
    

}


//operates on global OpenCV matrices g_labelImage and g_bw_img
//provide a transformed cloud pointer and indices of interest (filtered points above table)

void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices) {
    //for each pointcloud point, compute which pixel it maps to and find its z value
    //convert point cloud to top-down 2D projection for OpenCV processing
    float x, y, z;
    int index, u, v;
    Eigen::Vector3f cloud_pt;
    g_bw_img = 0; //initialize image to all black
    g_labelImage = 0; // ditto for result that  will get populated  with region labels
    int npts_cloud = indices.size();
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        index = indices[ipt];
        cloud_pt = transformed_cloud_ptr->points[index].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - MIN_Y) * PIXELS_PER_METER); //second component is from y of PCloud
            u = round((x - MIN_X) * PIXELS_PER_METER); //first component is from x of PCloud
            //flip/invert these so image makes sense visually
            u = Nu - u; //robot x-coord w/rt torso is in direction of rows from bottom to  top of image, so negate; 
            v = Nv - v; //robot y-coord w/rt torso points left, so negate to convert to image  column number
            //make sure the computed indices fit within the matrix size:
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                //set scaling such that 200 corresponds to 10cm height:
                int grayval = z * 1000;
                //cout<<"u,v,zval, grayval = "<<u<<", "<<v<<", "<<z<<", "<<grayval<<endl;

                if (grayval > 255) grayval = 255;

                g_bw_img(u, v) = (unsigned char) grayval; //assign scaled height as gray level; indices u,v are robot -x, -y, = row,col
            }
        }

    }
}


// if asked to find an object:
//  *take a snapshot
//  *find table height
//  *transform points to table frame
//  *box filter these points above table
//  *convert to 2D and find blobs
//  *call corresponding find-object function to recognize specific parts
//  *for blobs of interest, convert coords back to robot torso-lift frame
//  *fill result message with vector of poses of object of interest

void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    vector<geometry_msgs::PoseStamped> object_poses;
    /*
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height = goal->surface_ht;
    }
     * */
    bool found_object = false;
    //get a fresh snapshot:
    got_headcam_image_ = false;

    while (!got_headcam_image_) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
        ROS_INFO("waiting for snapshot...");
    }

    //if here, have a new cloud in *pclCam_clr_ptr_; 
    pcl::toROSMsg(*pclCam_clr_ptr_, ros_cloud_); //convert from PCL cloud to ROS message this way

    //transform this cloud to base-frame coords:
    ROS_INFO("transforming point cloud");
    //transform the head-camera data into the torso frame
    // result will be in "transformed_cloud_ptr_"
    pcl::transformPointCloud(*pclCam_clr_ptr_, *transformed_cloud_ptr_, affine_cam_wrt_torso_);

    //find table height from this snapshot:
    double table_height = find_table_height(transformed_cloud_ptr_, TABLE_TOP_MIN, TABLE_TOP_MAX, 0.01);

    //box-filter the points:
    //specify opposite corners of a box to box-filter the transformed points
    //magic numbers are at top of this program
    Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X, MIN_Y, table_height + 0.01, 0; //1cm above table top
    box_pt_max << MAX_X, MAX_Y, MAX_Z, 0;

    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    int npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]
    //
    blob_finder(g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs);

    result_.object_id = goal->object_id; //by default, set the "found" object_id to the "requested" object_id
    //note--finder might change this ID, if warranted

    switch (object_id) {
            //coordinator::ManipTaskResult::FAILED_PERCEPTION:
        case part_codes::part_codes::GEARBOX_TOP:
            //specialized functions to find objects of interest:

            found_object = find_gearbox_top(g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs, table_height, object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found gearbox_top objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }
            break;
            
        case part_codes::part_codes::TOTE:
            found_object = find_totes(g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs, table_height, object_poses); 
            if (found_object) {
                ROS_INFO("found tote objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }            
            break;

        default:
            ROS_WARN("this object ID is not implemented");
            result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED;
            object_finder_as_.setAborted(result_);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder_node"); // name this node 

    ROS_INFO("instantiating the object finder action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    //tf::TransformListener tfListener;
    //ROS_INFO("listening for kinect-to-base transform:");
    //tf::StampedTransform stf_kinect_wrt_base;
    /*
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and base_link...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), stf_kinect_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("kinect to base_link tf is good");
    object_finder_as.xformUtils_.printStampedTf(stf_kinect_wrt_base);
    tf::Transform tf_kinect_wrt_base = object_finder_as.xformUtils_.get_tf_from_stamped_tf(stf_kinect_wrt_base);
    g_affine_kinect_wrt_base = object_finder_as.xformUtils_.transformTFToAffine3f(tf_kinect_wrt_base);
    cout << "affine rotation: " << endl;
    cout << g_affine_kinect_wrt_base.linear() << endl;
    cout << "affine offset: " << g_affine_kinect_wrt_base.translation().transpose() << endl;
     */
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        object_finder_as.pubCloud_.publish(object_finder_as.ros_cloud_); //publish original point-cloud, so it is viewable in rviz        
        object_finder_as.pubBoxFilt_.publish(object_finder_as.ros_box_filtered_cloud_); //ditto for filtered point cloud   
        object_finder_as.pubCropFilt_.publish(object_finder_as.ros_crop_filtered_cloud_); //ditto for filtered point cloud   
        object_finder_as.pubPassFilt_.publish(object_finder_as.ros_pass_filtered_cloud_); //ditto for filtered point cloud   

        ros::Duration(0.1).sleep();
    }

    return 0;
}

bool ObjectFinder::find_gearbox_top(vector<float> x_centroids_wrt_robot, vector<float> y_centroids_wrt_robot,
        vector<float> avg_z_heights, vector<float> npts_blobs, float table_height,
        vector<geometry_msgs::PoseStamped> &object_poses) {
    geometry_msgs::PoseStamped object_pose;
    object_poses.clear();
    int n_objects = x_centroids_wrt_robot.size();
    if (n_objects < 2) return false; //background is object 0
    object_pose.header.frame_id = "torso_lift_link";
    for (int i_object = 1; i_object < n_objects; i_object++) {
        object_pose.pose.position.x = x_centroids_wrt_robot[i_object];
        object_pose.pose.position.y = y_centroids_wrt_robot[i_object];
        object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE;
        //FIX ME!  do not yet have value orientation info
        object_pose.pose.orientation.x = 0;
        object_pose.pose.orientation.y = 0;
        object_pose.pose.orientation.z = 0;
        object_pose.pose.orientation.w = 1;
        object_poses.push_back(object_pose);
    }
    return true;
}
