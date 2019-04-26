#include<object_finder_as/object_finder.h>

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

        object_finder_as.ros_crop_filtered_cloud_.header.frame_id = "torso";

        object_finder_as.pubCloud_.publish(object_finder_as.ros_cloud_); //publish original point-cloud, so it is viewable in rviz        
        object_finder_as.pubBoxFilt_.publish(object_finder_as.ros_box_filtered_cloud_); //ditto for filtered point cloud   
        object_finder_as.pubCropFilt_.publish(object_finder_as.ros_crop_filtered_cloud_); //ditto for filtered point cloud   
        object_finder_as.pubPassFilt_.publish(object_finder_as.ros_pass_filtered_cloud_); //ditto for filtered point cloud   

        ros::Duration(0.1).sleep();
    }

    return 0;
}
