// Makes the turtle bot move in a 3by3 foot square
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

geometry_msgs::Twist currT;
double sample_dt = 0.001;

void moveForward(double speed, double time, ros::Publisher twist_commander, ros::Rate loop_timer) {
        double timer = 0.0;

        currT.linear.x = speed; //command to move forward
        currT.angular.z = 0.0;
        while(timer < time) {
                twist_commander.publish(currT);
                timer+=sample_dt;
                loop_timer.sleep();
        }
}

void turn(double angle, double angularSpeed, ros::Publisher twist_commander, ros::Rate loop_timer) {
        double totalTime = angle/0.5; //How much time you must spin at 0.5rad/s to get desired angle

        currT.linear.x = 0.0; //stop moving forward
        currT.angular.z = angularSpeed; //and start spinning in place
        double timer = 0.0; //reset the timer
        while(timer < totalTime) {
                twist_commander.publish(currT);
                timer+=sample_dt;
                loop_timer.sleep();
        }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lab1");
    ros::NodeHandle nh;

    //publish a twist velocity publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

	currT.linear.x  = 0.0;
	currT.linear.y  = 0.0;
	currT.linear.z  = 0.0;
	currT.angular.x = 0.0;
	currT.angular.y = 0.0;
	currT.angular.z = 0.0;

	ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0; i<10; i++) {
        vel_pub.publish(currT);
        loop_timer.sleep();
	}
	
	double forSpeed = 0.2;
	double forTime  = 6.0;
	double angle    = 1.6;
	double angSpeed = 0.5; 

	moveForward(forSpeed, forTime, vel_pub, loop_timer); //Move forward forSpeed m/s for 5 seconds

	turn(angle, angSpeed, vel_pub, loop_timer); 	     //Turn 90 degrees counter clockwise

	moveForward(forSpeed, forTime, vel_pub, loop_timer);

	turn(angle, angSpeed, vel_pub, loop_timer);

	moveForward(forSpeed, forTime, vel_pub, loop_timer);

	turn(angle, angSpeed, vel_pub, loop_timer);

	moveForward(forSpeed, forTime, vel_pub, loop_timer);

	turn(angle, angSpeed, vel_pub, loop_timer);

	moveForward(0.0, forTime, vel_pub, loop_timer); //Stop

    return 0;
}

