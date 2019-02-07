#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <nav_msgs/Odometry.h>

using namespace std;

nav_msgs::Odometry g_odom; //odom is not actually used in this code, but could be
                           //callback extracts the robot heading w/rt world

//some "magic" (tunable) numbers
//define a radius within which we want to be near to a wall on the left
const double WALL_FOLLOW_RADIUS = 0.75;

const double RADIUS_LEFT_TURN = 0.2; //0.1;

//some timing constants
const double SPEED = 0.3; //0.3; // m/s speed command
const double YAW_RATE = 0.3; //0.1; // rad/sec yaw rate command
const double SAMPLE_DT = 0.01; //specify a sample period of 10ms

//crude tangent approx/look-ahead: pi/2 -0.2 rad from front, i.e.
//a ping slightly ahead of the robot's immediate left
//BETTER would be to compute a tangent of the wall to the left
const double DANG_TANGENT_APPROX = 0.2;

// these values to be set within the laser callback
//parameters of the lidar:
double g_angle_min = 0.0;
double g_angle_max = 0.0;
double g_angle_increment = 0.0;
double g_range_min = 0.0;
double g_range_max = 0.0;

//some useful constants computed once at first LIDAR callback:
int g_index_min_dist_ping; //ping index corresponding to closest ray
int g_index_90deg_right; //ping index at -pi/2, i.e. robot's right
int g_index_90deg_left;  //ping index at +pi/2, i.e. robot;s left
int g_index_tangent_left; //ping index slightly ahead of ping to left
int g_index_front = -1; // init NOT real; callback will have to fix this
//above is a crude way to indicate callback inits are necessary;
// would be cleaner to define a new variable for this: g_first_call = true;

//global variables, computed each callback of LIDAR message;
float g_radius_left; //radius of ping to robot's left; want to keep this about WALL_FOLLOW_RADIUS
float g_radius_min; //radius of closest ping from -pi/2 to +pi/2 (exclude pings behind robot)
float g_radius_tan_test; //radius of ping used to look ahead from robots left
float g_clearance_tan_test; //cos(theta)*radius of look-ahead ping; i.e. clearance
//ping index corresponding to straight ahead

sensor_msgs::LaserScan g_laser_scan;

double g_phi = -100; //robot heading--absolute,  w/rt world (per odom); gets updated by odom

//Used to make returning two averages easy
struct Averages
{
        double xAvg;
        double yAvg;
};

Averages g_AvgLeftUp;

void odomCallback(const nav_msgs::Odometry& odom_msg) {
        g_odom = odom_msg;
        //here is a means to convert quaternion to a scalar heading angle
        double quat_z = odom_msg.pose.pose.orientation.z;
        double quat_w = odom_msg.pose.pose.orientation.w;
        g_phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
        ROS_INFO("g_phi = %f", g_phi);
}

bool start = false;

//callback function to interpret lidar pings;
// look for blockage ahead, clearance to left, and clearance slightly ahead on left
// use computed values for wall following
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
        g_laser_scan = laser_scan;
        //on very first call, set up some values that will be re-used in subsequent calls
        if (g_index_front < 0) {
                //for first message received, set up the desired index of LIDAR range to eval
                g_angle_min = laser_scan.angle_min;
                g_angle_max = laser_scan.angle_max;
                g_angle_increment = laser_scan.angle_increment;
                g_range_min = laser_scan.range_min;
                g_range_max = laser_scan.range_max;
                // what is the index of the ping that is straight ahead?
                g_index_front = (int) ((0.0 - g_angle_min) / g_angle_increment);
                g_index_90deg_left = (int) ((2*g_angle_max) / g_angle_increment) - 10;
                g_index_90deg_right = 0;
                //arbitrarily choose to also look at ping xx rad CW from 90deg left
                // use this for tangent approx
                g_index_tangent_left = g_index_90deg_left - (int) (DANG_TANGENT_APPROX / g_angle_increment);
                int num_pings = laser_scan.ranges.size();
                ROS_INFO("LIDAR setup: ");
                ROS_INFO("there are %d pings in the laserscan", num_pings);
                ROS_INFO(" g_index_front = %d", g_index_front);
                ROS_INFO(" g_index_90deg_left = %d", g_index_90deg_left);
                ROS_INFO(" g_index_90deg_right = %d", g_index_90deg_right);
                ROS_INFO(" g_index_tangent_left = %d", g_index_tangent_left);
                ros::Duration(2.0).sleep();
        }
        g_radius_left = laser_scan.ranges[g_index_90deg_left];
		g_radius_tan_test = laser_scan.ranges[g_index_tangent_left];

		if(isnan(g_radius_left) && start) {
				g_radius_left = g_range_max;
		}
		if(isnan(g_radius_tan_test) && start) {
				g_radius_tan_test = g_range_max;
		}
	
        g_clearance_tan_test = g_radius_tan_test * cos(DANG_TANGENT_APPROX);
        //search for min ping dist:
        g_radius_min = g_radius_left;
        float r_test;
        g_index_min_dist_ping = g_index_90deg_left;
        for (int i = g_index_90deg_left; i >= g_index_90deg_right; i--) {
                r_test = laser_scan.ranges[i];
                //ROS_INFO("i, r_test = %d,  %f",i,r_test);
                if (r_test <= g_radius_min) {
                        g_radius_min = r_test;
                        g_index_min_dist_ping = i;
                }
        }
        ROS_INFO("min ping dist = %f at index %d", g_radius_min, g_index_min_dist_ping);
}

//Gets the X value in reference to the robot given angle and ping distance.
double getXFromPing(double dist, double angle) {
        return cos(angle)*dist;
}

//Gets the Y value in reference to the robot given angle and ping distance.
double getYFromPing(double dist, double angle) {
        return sin(angle)*dist;
}

//This function will average numInAvg laserscan distances in front of startingIdx.
Averages averagePings(int numInAvg, int startingIdx) {
        double sumX = 0.0;
        double sumY = 0.0;

        //Going backwards because we want to average pings going clockwise
        for (int i = startingIdx; i > (startingIdx - numInAvg); i-- ) {
				if(isnan(g_laser_scan.ranges[i])) {
					  numInAvg++;
					  continue;
				}
                sumX += getXFromPing(g_laser_scan.ranges[i], M_PI - (i-g_index_90deg_right)*g_angle_increment);
                sumY += getYFromPing(g_laser_scan.ranges[i], M_PI - (i-g_index_90deg_right)*g_angle_increment);
        }
        Averages result = {(sumX / numInAvg),(sumY / numInAvg)};
        return result;
}

//Returns the angle that the wall is facing assuming that
//Straight up and down is 0 and counter clockwise in positive
double determineXDiffOfWallOnLeft() {
        double wallAngle = 0.0;

        int pingLeftIdx = g_index_90deg_left;
        int pingLeftUpIdx = g_index_90deg_left - (int)((M_PI/8)/g_angle_increment); //Index relating to pi/3 degrees ahead of direct left

        Averages AvgLeft;

        AvgLeft     = averagePings(10, pingLeftIdx);
        g_AvgLeftUp = averagePings(10, pingLeftUpIdx);

        double distLeft   = sqrt(pow(AvgLeft.xAvg,2) + pow(AvgLeft.yAvg,2)); //Distance from bot to left wall
        double distLeftUp = sqrt(pow(g_AvgLeftUp.xAvg,2) + pow(g_AvgLeftUp.yAvg,2)); //Distance from bot to further up on left wall

        //If the distLeftUp is off of the wall then dont turn the robot at all
        if (abs(distLeftUp - distLeft) > WALL_FOLLOW_RADIUS || abs(distLeftUp) > 3*WALL_FOLLOW_RADIUS) {
                ros::spinOnce(); //update g_phi
                ROS_INFO("INVALID");
                return 0.0; //So that the robot doesn't turn at all since we got no real info about the wall
        } else {
                ROS_INFO("VALID");
                return AvgLeft.xAvg - g_AvgLeftUp.xAvg;
        }
}

//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
        ros::init(argc, argv, "commander");
        ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
        ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        ros::Subscriber lidar_subscriber = n.subscribe("/scan", 1, laserCallback);
        ros::Subscriber odom_subscriber = n.subscribe("/odom", 1, odomCallback);

        geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR
        // start with all zeros in the command message; should be the case by default, but just to be safe..
        twist_cmd.linear.x = 0.0;
        twist_cmd.linear.y = 0.0;
        twist_cmd.linear.z = 0.0;
        twist_cmd.angular.x = 0.0;
        twist_cmd.angular.y = 0.0;
        twist_cmd.angular.z = 0.0;
        double phi_cmd = 0.0;

        ros::Rate loop_timer(1 / SAMPLE_DT); //create a ros object from the ros “Rate” class; set 100Hz rate
        double timer = 0.0;
        //start sending some zero-velocity commands, just to warm up communications with STDR
        for (int i = 0; i < 10; i++) {
                twist_commander.publish(twist_cmd);
                ros::spinOnce();
                loop_timer.sleep();
        }
        while (g_index_front < 0) {
                ros::spinOnce();
                loop_timer.sleep();
        }
        //make sure have feedback from odom
        while (g_phi<-50) {
                ros::spinOnce();
                loop_timer.sleep();
        }
        phi_cmd = g_phi;
        //WALL-FOLLOWING ALGORITHM:
        //init: start moving forward until some lidar ping is less than WALL_FOLLOW_RADIUS
        twist_cmd.linear.x = SPEED; //command to move forward
        twist_cmd.angular.z = 0.0;
        ROS_INFO("moving forward until first ping at test radius");
        while (g_radius_min > WALL_FOLLOW_RADIUS) {
                // ROS_WARN("lost left wall...");
                ROS_WARN("min ping dist = %f at index %d", g_radius_min, g_index_min_dist_ping);
                twist_commander.publish(twist_cmd);
                ros::spinOnce();
                loop_timer.sleep();
        }

        //OK...ready to start
        ROS_INFO("starting wall following algorithm");
        while (ros::ok()) {
				start = true;

                ros::spinOnce();
                ROS_INFO("testing if mindist ping is other than left wall");
                int pingIndexTolerance = 20;
                if ((g_radius_min < WALL_FOLLOW_RADIUS) && (g_index_min_dist_ping < g_index_tangent_left) && (abs(g_index_min_dist_ping - g_index_tangent_left) > pingIndexTolerance)) {
                        //if here, then there is a ping closer than left wall; rotate to make this min-dist ping on left.
                        //Only happens if the min dist ping is more than pingIndexTolerance pings away from directly left.
                        twist_cmd.linear.x = 0; //halt
                        twist_commander.publish(twist_cmd);
                        ROS_WARN("barrier ahead; need to spin");
                        ROS_WARN("min ping dist = %f at index %d", g_radius_min, g_index_min_dist_ping);

                        double dtheta = ((double) (g_index_min_dist_ping - g_index_90deg_left - pingIndexTolerance)) * g_angle_increment;
                        ROS_INFO("rotate dtheta = %f", dtheta);
                        twist_cmd.angular.z = -YAW_RATE;
                        twist_commander.publish(twist_cmd);
                        double dt = fabs(dtheta / YAW_RATE);
                        ROS_INFO("spin for dt = %f", dt);
                        ros::Duration(dt).sleep();
                        g_radius_min = -1.0; //make sure we have a fresh update from lidar callback; crude trick
                        while (g_radius_min < 0) { //val will change after a callback
                                ros::spinOnce();
                                loop_timer.sleep();
                        }
                        ROS_WARN("done spinning; min ping dist = %f at index %d", g_radius_min, g_index_min_dist_ping);

                        twist_cmd.angular.z = 0.0;
                        twist_cmd.linear.x = 0;
                        ROS_WARN("stopping");
                        twist_commander.publish(twist_cmd);
                }

                //should have clearance on left, as well as slightly forward from there, on left
                ROS_INFO("tangent test clearance = %f", g_clearance_tan_test);
                twist_cmd.linear.x = SPEED;
                ROS_INFO("clearance to left, and clearance ahead left: %f, %f", g_radius_left, g_clearance_tan_test);
                twist_commander.publish(twist_cmd);
                while ((g_clearance_tan_test < WALL_FOLLOW_RADIUS) && (g_index_min_dist_ping >= g_index_tangent_left)) {
                        ROS_WARN("following left wall...");
                        twist_cmd.linear.x = SPEED;
                        ROS_INFO("clearance to left, and clearance ahead left: %f, %f", g_radius_left, g_clearance_tan_test);
                        twist_commander.publish(twist_cmd);
                        ros::spinOnce();
                        loop_timer.sleep();
                }

                if ((g_radius_min < WALL_FOLLOW_RADIUS) && (g_index_min_dist_ping < (g_index_tangent_left - pingIndexTolerance))) {
                        ROS_WARN("blocked ahead");
                        ROS_WARN("min ping dist = %f at index %d", g_radius_min, g_index_min_dist_ping);
                } else if (g_clearance_tan_test > WALL_FOLLOW_RADIUS) {
                        //if the test ping (fwd from left side) is no longer within follower radius, go hunting for it
                        // by making a circular-arc left turn
                        ROS_WARN("lost fwd test ping...turning left");
                        twist_cmd.angular.z = YAW_RATE;
                        twist_cmd.linear.x = RADIUS_LEFT_TURN*YAW_RATE;
                        twist_commander.publish(twist_cmd);

                        while (g_radius_min > WALL_FOLLOW_RADIUS) {
                                ROS_WARN("trying to reconnect to wall w/ circular trajectory");
                                loop_timer.sleep();
								twist_commander.publish(twist_cmd);
								ros::spinOnce();
                        }
                        ROS_INFO("reconnected to wall on left");
                        ROS_INFO("clearance to left, and clearance ahead left: %f, %f", g_radius_left, g_clearance_tan_test);

                        twist_cmd.angular.z = 0.0;
                        twist_cmd.linear.x = 0;
                        ROS_WARN("stopping");
                        twist_commander.publish(twist_cmd);

                        double xDiff = determineXDiffOfWallOnLeft();
                        //This code will turn the robot so that it is facing parallel to the wall on its left
                        double angSpeed = YAW_RATE; //0.1;
                        int count = 0;
                        ROS_INFO("Turning to become parallel with the wall");
                        bool linedUp = false;
                        while(abs(xDiff) >= 0.01) { //while xDiff on wall is greater than 0.05m spin.
                                //Spin in place until you are within good error margins

                                //if xDiff is poisitive, better to spin clockwise
                                if(xDiff > 0) {
                                        twist_cmd.angular.z = -1*angSpeed;
                                } else {
                                        twist_cmd.angular.z = angSpeed;
                                }
                                twist_cmd.linear.x = 0;
                                twist_commander.publish(twist_cmd);
                                ros::spinOnce();
                                loop_timer.sleep();

                                xDiff = determineXDiffOfWallOnLeft();
                                ROS_INFO("XDIFF: %f", xDiff);
                                count++;

                                if(count >= 200) {
                                        break; //break if the loop is stuck
                                }
                        }

                        //Commanding the robot to now move forward
                        twist_cmd.linear.x = SPEED; //command to move forward
                        twist_cmd.angular.z = 0.0;
                        //twist_commander.publish(twist_cmd);

                        //If you are "lined up" with the wall, move forward for a second
                        if(xDiff != 0.0) {
                                twist_commander.publish(twist_cmd);
                                double frontDist = g_AvgLeftUp.yAvg - WALL_FOLLOW_RADIUS;

                                //If infinite range then set to the max range of the lidar
                                if(isnan(frontDist)) {
                                    frontDist = g_laser_scan.range_max - WALL_FOLLOW_RADIUS;
                                }

                                double travelT = frontDist / SPEED;
                                ros::Rate loop_timer2(1.0/travelT);
                                loop_timer2.sleep();
                        }
                }

        } //loop forever

}
