#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <actionlib/client/simple_action_client.h>

#define _freq 2
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


using namespace std;

class goal_gen {
	public:
		goal_gen();
		void loop();
	private:
		float _disp;
		float _max_wander;
		tf::Transform _trans;
		tf::StampedTransform _check_trans;
		tf::StampedTransform _prev_trans;
		tf::TransformBroadcaster _trans_br;
		tf::TransformListener _listener;
		tf::TransformListener _listener_goal;
		tf::StampedTransform _transform;
		ros::NodeHandle _nh;
		ros::Publisher _goal_pub;
		ros::Rate _rate;

};

goal_gen::goal_gen(): _rate(_freq) {
	_disp = 0.3;
	_max_wander = 0.05;
	_goal_pub = _nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	srand(time(NULL));
	boost::thread(&goal_gen::loop, this);
}



void goal_gen::loop() {
	//geometry_msgs::PoseStamped nav_msg;
	MoveBaseClient ac ("move_base",true);
	move_base_msgs::MoveBaseGoal goal;
	bool Tf_equal = false;
	bool first_check = true;
	double roll, pitch, yaw;
	tf2::Quaternion quaternion;

	bool sent = false;

	while (!sent && ros::ok()) {
		bool marker_seen = _listener.waitForTransform("camera_frame", "aruco_marker_frame", ros::Time(0), ros::Duration(1.3));
		ROS_ERROR("LOOKING FOR MARKER");
		if (marker_seen) {
			try{
				ROS_ERROR("MARKER SEEN");
				_trans.setOrigin( tf::Vector3(0, 0, _disp) );
				tf::Quaternion q;
				q.setRPY(0,1.57,-1.57);
				_trans.setRotation(q);
				_trans_br.sendTransform(tf::StampedTransform(_trans, ros::Time::now(), "aruco_marker_frame", "goal_frame"));
				_listener_goal.waitForTransform("map", "goal_frame", ros::Time(0), ros::Duration(0.5));
				_listener_goal.lookupTransform("map","goal_frame",ros::Time(0), _transform);

				goal.target_pose.header.frame_id="map";
				goal.target_pose.header.stamp=ros::Time::now();
				

				goal.target_pose.pose.position.x =  _transform.getOrigin().x();
				goal.target_pose.pose.position.y =  _transform.getOrigin().y();

				tf::Matrix3x3 m( _transform.getRotation() ); // quaternion to RPY
				m.getRPY(roll, pitch, yaw);
				quaternion.setRPY(0,0,yaw);  // RPY to quaternion 
				
				//DEBUG
				//quaternion.setRPY(0,0,0.554); 
				goal.target_pose.pose.orientation.x = quaternion.x() ; 
				goal.target_pose.pose.orientation.y = quaternion.y() ; 
				goal.target_pose.pose.orientation.z = quaternion.z() ; 
				goal.target_pose.pose.orientation.w = quaternion.w() ; 

				ROS_INFO("sending goal");
				ac.sendGoal(goal);
				sent = true;
				/*ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					ROS_INFO("Hooray, the base moved 1 meter forward");
				}
				else {
					ROS_INFO("The base failed to move forward 1 meter for some reason");
				}*/
				ROS_ERROR("SENT, DONE!"); //DEBUG
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		} else {
			try{
				ROS_ERROR("WANDERING TRY");
				_listener_goal.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
				_listener_goal.lookupTransform("map","base_link",ros::Time(0), _transform);
				float rand_dist = float(rand())/RAND_MAX*_max_wander + 0.05;
				float rand_yaw = float(rand())/RAND_MAX*6.28;
				ROS_ERROR("%f", rand_dist);
				ROS_ERROR("%f", rand_yaw);

				goal.target_pose.header.frame_id="map";
				goal.target_pose.header.stamp=ros::Time::now();

				goal.target_pose.pose.position.x =  _transform.getOrigin().x() + rand_dist*cos(rand_yaw);
				goal.target_pose.pose.position.y =  _transform.getOrigin().y() + rand_dist*sin(rand_yaw);

				quaternion.setRPY(0,0,rand_yaw);

				goal.target_pose.pose.orientation.x = quaternion.x() ; 
				goal.target_pose.pose.orientation.y = quaternion.y() ; 
				goal.target_pose.pose.orientation.z = quaternion.z() ; 
				goal.target_pose.pose.orientation.w = quaternion.w() ; 

				ac.sendGoal(goal);
				/*ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					ROS_INFO("Hooray, the base moved 1 meter forward");
				}
				else {
					ROS_INFO("The base failed to move forward 1 meter for some reason");
				}*/
				ROS_ERROR("WANDERING"); //DEBUG
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(0.1).sleep();
			}
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
