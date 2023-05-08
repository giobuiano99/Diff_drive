#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "boost/thread.hpp"

using namespace std;

class JOY_WRAP {
	public:
		JOY_WRAP();
		void cb(sensor_msgs::Joy::ConstPtr msg);
		void publish();
	private:
		float _rescaleValueHead;
		float _rescaleValueTurn;
		float _headingVel;
		float _turningVel;
		bool _topicActive;
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Publisher _topic_pub;
		ros::Rate _rate;
};

JOY_WRAP::JOY_WRAP(): _rate(100) {
	_rescaleValueHead = 0.1;
	_rescaleValueTurn = 1.5;
	_headingVel = 0;
	_turningVel = 0;
	_topicActive = false;
	_topic_sub = _nh.subscribe("/joy", 1, &JOY_WRAP::cb, this);
	_topic_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	boost::thread(&JOY_WRAP::publish, this);
}

//Callback function: the input of the function is the data to read
void JOY_WRAP::cb(sensor_msgs::Joy::ConstPtr msg) {
	if(!_topicActive){_topicActive = true;}
	_turningVel = msg->axes[0];
	_headingVel = msg->axes[1];
	ROS_INFO("I heard: headingVel = %f, turningVel = %f\n", _headingVel, _turningVel);
}

void JOY_WRAP::publish() {
	geometry_msgs::Twist f;
	f.linear.y = 0;
	f.linear.z = 0;
	f.angular.x = 0;
	f.angular.y = 0;
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
