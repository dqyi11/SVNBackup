#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <tf/transform_listener.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>
#include <ros/console.h>

using namespace std;

string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
	static tf::TransformBroadcaster sender;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
	transform.setRotation(tf::Quaternion(0, 0, msg->theta));
	sender.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "couzin_broadcaster");
	if(argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
	turtle_name = argv[1];
	ROS_ERROR("%s", turtle_name.c_str());
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
	ros::spin();

	return 0;
};
