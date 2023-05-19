#include "ros/ros.h"
#include "geometry_msgs/PoseStamped"
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#define _freq 30



using namespace std;

class goal_gen {
	public:
		goal_gen();
		void loop();
	private:
		float _disp;
		tf::Transform _trans;
		tf::TransformBroadcaster _trans_br;
		tf::TransformListener _listener;
		ros::NodeHandle _nh;
		ros::Rate _rate;

};

goal_gen::goal_gen(): _rate(_freq) {
	_disp = 0.1;
	boost::thread(&goal_gen::loop, this);
}



void goal_gen::loop() {

	while (ros::ok()) {

		try {
            _listener.waitForTransform("aruco_marker_frame","camera_frame",ros::Time(0),ros::Duration(3.0));
			 _trans.setOrigin( tf::Vector3(0, 0, _disp) );
			 tf::Quaternion q;
			 q.setRPY(0,1.57,-1.57);


			_trans.setRotation(q);
			_trans_br.sendTransform(tf::StampedTransform(_trans, ros::Time::now(), "aruco_marker_frame", "goal_frame"));




        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


		_rate.sleep();
	}

}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "goal_node");

	//Create the ROS_SUB class object
	goal_gen gg_node;

	ros::spin();

	return 0;
}
