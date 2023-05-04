#include "ros/ros.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"

using namespace std;

class ODOMHEN {
	public:
		ODOMHEN();
		void cb(geometry_msgs::Twist::ConstPtr msg);
		void odom();
	private:
		float _x;
		float _y;
		float _theta;
		float _headingVel;
		float _turningVel;
		bool _topicActive;
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Publisher _topic_pub;
		ros::Rate _rate;
};

ODOMHEN::ODOMHEN(): _rate(100) {
	_x = 0;
	_y = 0;
	_theta = 0;
	_headingVel = 0;
	_turningVel = 0;
	_topicActive = false;
	_topic_sub = _nh.subscribe("/kvaraspace/cmd_vel", 1, &ODOMHEN::cb, this);
	_topic_pub = _nh.advertise<nav_msgs::Odometry>("/odom", 1);
	boost::thread(&ODOMHEN::odom, this);
}

//Callback function: the input of the function is the data to read
void cb(geometry_msgs::Twist::ConstPtr msg) {
	if(!_topicActive){_topicActive = true;}
	_turningVel = msg->angular.z;
	_headingVel = msg->linear.x;
	ROS_INFO("I heard: headingVel = %f, turningVel = %f", _headingVel, _turningVel);
}

void ODOMHEN::odom() {
	geometry_msgs::TwistWithCovariance twist;
	geometry_msgs::PoseWithCovariance pose;
	string child_frame_id = "base_link";
	twist.x = 0;
	twist.y = 0;
	twist
	while (ros::ok()) {
		f.linear.x = _headingVel*_rescaleValueHead;
		f.angular.z = _turningVel*_rescaleValueTurn;
		if(_topicActive) {_topic_pub.publish(f);}
		_rate.sleep();
	}

}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "Joy_wrap");

	//Create the ROS_SUB class object
	JOY_WRAP wrap;

	ros::spin();

	return 0;
}
