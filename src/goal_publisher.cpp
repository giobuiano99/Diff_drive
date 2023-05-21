#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <actionlib/client/simple_action_client.h>

#define _freq 1
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


using namespace std;

class goal_gen {
	public:
		goal_gen();
		void loop();
	private:
		float _disp;
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
	_goal_pub = _nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	boost::thread(&goal_gen::loop, this);
}



void goal_gen::loop() {
	geometry_msgs::PoseStamped nav_msg;
	MoveBaseClient ac ("move_base",true);
	move_base_msgs::MoveBaseGoal goal;
	bool Tf_equal = false;
	bool first_check = true;
	double roll, pitch, yaw;
	tf2::Quaternion quaternion;


	while (ros::ok()) {
		
		try{
            //_listener.waitForTransform("aruco_marker_frame","camera_frame",ros::Time(0),ros::Duration(3.0));
			//_listener.lookupTransform("aruco_marker_frame","camera_frame",ros::Time(0), _check_trans);

		   // compare tfs to check if the marker is in the field of view
			if (first_check) {
				first_check = false;
			} else {
				if (_check_trans == _prev_trans) {
					Tf_equal = true;
				} else {
					Tf_equal = false;
				}
			}

			if (!Tf_equal) {
				_trans.setOrigin( tf::Vector3(0, 0, _disp) );
				tf::Quaternion q;
				q.setRPY(0,1.57,-1.57);
				_trans.setRotation(q);
				//_trans_br.sendTransform(tf::StampedTransform(_trans, ros::Time::now(), "aruco_marker_frame", "goal_frame"));
				//_listener_goal.lookupTransform("map","goal_frame",ros::Time(0), _transform);

				goal.target_pose.header.frame_id="map";
				goal.target_pose.header.stamp=ros::Time::now();
				

				//goal.target_pose.pose.position.x =  _transform.getOrigin().x();
				//goal.target_pose.pose.position.y =  _transform.getOrigin().y();
				goal.target_pose.pose.position.x = 1.2;
				goal.target_pose.pose.position.y = 1.2;

				//tf::Matrix3x3 m( _transform.getRotation() ); // quaternion to RPY
				//m.getRPY(roll, pitch, yaw);
				//quaternion.setRPY(0,0,yaw-3.14);  // RPY to quaternion 
				
				//DEBUG
				quaternion.setRPY(0,0,0.554); 
				goal.target_pose.pose.orientation.x = quaternion.x() ; 
				goal.target_pose.pose.orientation.y = quaternion.y() ; 
				goal.target_pose.pose.orientation.z = quaternion.z() ; 
				goal.target_pose.pose.orientation.w = quaternion.w() ; 


				/*nav_msg.header.frame_id = "goal_frame";
				
				nav_msg.pose.position.x = _transform.getOrigin().x();
				nav_msg.pose.position.y = _transform.getOrigin().y();
				nav_msg.pose.position.z = _transform.getOrigin().z();
				/*nav_msg.pose.position.x = _transform.getOrigin().z();
				nav_msg.pose.position.y = _transform.getOrigin().x();
				nav_msg.pose.position.z = _transform.getOrigin().y(); */
				
				/*tf::Matrix3x3 m( _transform.getRotation() ); // quaternion to RPY
				m.getRPY(roll, pitch, yaw);
				quaternion.setRPY(0,0,yaw-3.14);  // RPY to quaternion 


				nav_msg.pose.orientation.x = quaternion.x();
				nav_msg.pose.orientation.y = quaternion.y();
				nav_msg.pose.orientation.z = quaternion.z();
				nav_msg.pose.orientation.w = quaternion.w();
			       /*nav_msg.pose.orientation.x = 0;
				nav_msg.pose.orientation.y = 0;
				nav_msg.pose.orientation.z = 0;
				nav_msg.pose.orientation.w = 1;*/
				
				ROS_INFO("sending goal");
				ac.sendGoal(goal);
				ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Hooray, the base moved 1 meter forward"); 
				else  ROS_INFO("The base failed to move forward 1 meter for some reason");
			}
			


			//nav_msg.header.stamp = ros::Time::now();
			//_goal_pub.publish(nav_msg);

			_prev_trans.setOrigin(_check_trans.getOrigin());
			_prev_trans.setRotation(_check_trans.getRotation());

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
