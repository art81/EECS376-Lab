//magic numbers for filtering pointcloud points:
//specify the min and max values, x,y, znd z, for points to retain...
// as expressed in the robot's torso frame (torso_lift_link)
const float MIN_X =  0.4;  //include points starting 0.4m in front of robot
const float MAX_X =  1.0;  //include points out to 1.0m in front of robot
const float MIN_Y = -0.5;  //include points starting -0.5m to left of robot
const float MAX_Y =  0.5;  //include points up to 0.5m to right of robot

const float TABLE_TOP_MIN = -0.5;
const float TABLE_TOP_MAX = 0.0;

//choose, e.g., resolution of 5mm, so 100x200 image for 0.5m x 1.0m pointcloud
// adjustable--> image  size
// try 400 pix/m...works fine, i.e. 2.5mm resolution
const float PIXELS_PER_METER = 400.0; //200.0;

const int Nu = (int) ((MAX_X - MIN_X) * PIXELS_PER_METER);
const int Nv = (int) ((MAX_Y - MIN_Y) * PIXELS_PER_METER);

Mat_<uchar> bw_img(Nu, Nv);
Mat_<int>labelImage(Nu, Nv);
Mat dst(bw_img.size(), CV_8UC3);

//given a binary image in bw_img, find and label connected regions  (blobs)
// labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot
void blob_color(void) {
    //openCV function to do all the  hard work
    int nLabels = connectedComponents(bw_img, labelImage, 8); //4 vs 8 connected regions
    ROS_INFO("found %d blobs",nLabels);
    g_x_centroids.resize(nLabels);
    g_y_centroids.resize(nLabels);
    g_x_centroids_wrt_robot.resize(nLabels);
    g_y_centroids_wrt_robot.resize(nLabels);
    g_avg_z_heights.resize(nLabels);
    g_npts_blobs.resize(nLabels);
    for (int label = 0; label < nLabels; ++label) {
        g_x_centroids[label]=0.0;
        g_y_centroids[label]=0.0;
        g_x_centroids_wrt_robot[label]=0.0;
        g_y_centroids_wrt_robot[label]=0.0;
        g_avg_z_heights[label]=0.0;
        g_npts_blobs[label]=0.0;
    }
    //colorize the regions and display them:
    //also compute centroids;
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0); //make the background black
    //assign random color to each region label
    for (int label = 1; label < nLabels; ++label) {
        colors[label] = Vec3b((rand()&255), (rand()&255), (rand()&255));
    }

    //for human consumption: assign colors to regions and display result
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            int label = labelImage.at<int>(r, c);
            Vec3b &pixel = dst.at<Vec3b>(r, c);
            pixel = colors[label];
            //while  we're combing through pixels, compute centroids as well:
            g_y_centroids[label]+=c;
            g_x_centroids[label]+=r;
            g_npts_blobs[label]+=1.0;
        }
    }
    ROS_INFO("image size: %d cols, %d rows",dst.cols,dst.rows);
           
    for (int label = 0; label < nLabels; ++label) {
        g_x_centroids[label]/=g_npts_blobs[label];
        g_y_centroids[label]/=g_npts_blobs[label];

        ROS_INFO("label %d has %f points w/ centroid %f, %f:",label,g_npts_blobs[label],g_x_centroids[label],g_y_centroids[label]);
    }
    //convert to robot coords:
    //convert to dpixel_x, dpixel_y w/rt image centroid, scale by pixels/m, and add offset from PCL cropping, x,y
    for (int label = 1; label < nLabels; ++label) {    
        g_x_centroids_wrt_robot[label] = ((dst.rows/2) - g_x_centroids[label])/PIXELS_PER_METER + (MIN_X+MAX_X)/2.0;
        g_y_centroids_wrt_robot[label] = (g_y_centroids[label]- (dst.cols/2))/PIXELS_PER_METER + (MIN_Y+MAX_Y)/2.0;
        ROS_INFO("label %d has centroid w/rt robot: %f, %f:",label,g_x_centroids_wrt_robot[label],g_y_centroids_wrt_robot[label]);
                
    }
	
	g_blobs_orientations.push_back(-1); //Invalid orientation for background blob
	for(int i = 1;i < nLabels; i++) {
		g_blobs_orientations.push_back(findOrientation(i));
		ROS_INFO("label %d has orientation: %f:",i,g_blobs_orientations[i]);

		geometry_msgs::Quaternion currQuat = COPYconvertPlanarPsi2Quaternion(g_blobs_orientations[i]);
		ROS_INFO("and has corresponding quaternion: %f, %f, %f, %f", currQuat.x, currQuat.y, currQuat.z, currQuat.w);
	}

    //display the result in an openCV window
    //imshow("Connected Components", dst); //do this  from "main" instead
}

bool ObjectFinder::find_gearbox_bottom(float surface_height, geometry_msgs::PoseStamped &object_pose) {
	Eigen::Vector3f plane_normal;
	double plane_dist;
    Eigen::Vector3f major_axis;
    Eigen::Vector3f centroid;
    
    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    get_transformed_cloud(transformedCloud);
    
    blob_color(transformedCloud);
    
    bool found_object = pclUtils_.find_plane_fit(MIN_X, MAX_X, MIN_Y, MAX_Y, surface_height + 0.05, surface_height + 0.3, 0.001,
            									 plane_normal, plane_dist, major_axis, centroid);
            									 
    if (plane_normal(2) < 0) plane_normal(2) *= -1.0; //in world frame, normal must point UP
    Eigen::Matrix3f R;
    Eigen::Vector3f y_vec;
    R.col(0) = major_axis;
    R.col(2) = plane_normal;
    R.col(1) = plane_normal.cross(major_axis);
    Eigen::Quaternionf quat(R);
    object_pose.header.frame_id = "torso";
    object_pose.pose.position.x = centroid(0);
    object_pose.pose.position.y = centroid(1);
    //the TOY_BLOCK model has its origin in the middle of the block, not the top surface
    //so lower the block model origin by half the block height from upper surface
    object_pose.pose.position.z = surface_height + 0.005;
    //create R from normal and major axis, then convert R to quaternion

    object_pose.pose.orientation.x = quat.x();
    object_pose.pose.orientation.y = quat.y();
    object_pose.pose.orientation.z = quat.z();
    object_pose.pose.orientation.w = quat.w();
    
    return found_object;
}
