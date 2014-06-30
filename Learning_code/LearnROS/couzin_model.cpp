#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>
#include "std_msgs/String.h"
#include <ros/console.h>
#include <sstream>
#include <math.h>

using namespace std;

string turtle_name;
int turtleSize = 1;
double rr = 1.0;
double ro = 2.0;
double ra = 12.0;
double o_rr =0;
double angular_vel = (M_PI/180.0) * 40;
double speed = 3;
double sight = 270;

void poseCallback(const turtlesim::PoseConstPtr& msg){
	static tf::TransformBroadcaster sender;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
	transform.setRotation(tf::Quaternion(0, 0, msg->theta));
	sender.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}


double norm(double x, double y){
	return sqrt(pow(x,2.0)+pow(y,2.0));
}

double angle_convert(double an){
	if(an < 0){
		return (2.0 * M_PI) + an;
	}
	return an;
}
double angle_gap(double a1, double a2){
	double angle = a2 - a1;
    	if (angle > M_PI){
        	angle = angle - 2.0 * M_PI;
	} else if (angle < -M_PI){
		angle = angle + 2.0 * M_PI;
	}
   return angle;
}
bool canSee(double lower, double upper, double cal_angle){
	if(lower > upper){
		if((lower <= cal_angle && cal_angle <= (2.0 * M_PI)) || (0 <= cal_angle && cal_angle <= upper) ){
			return true;
		} 
	} else {
		if(lower <= cal_angle && cal_angle <= upper) { return true;}
	}
	return false;
}
 
int main(int argc, char** argv){
	ros::init(argc, argv, "couzin_combine");
	//if(argc < 2){ROS_ERROR("need turtle name as argument"); return -1;};
	turtle_name = argv[1];
	ROS_ERROR("%s", turtle_name.c_str());
	ros::NodeHandle node;

	ros::service::waitForService("spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	srv.request.x = atof(argv[2]);
	srv.request.y = atof(argv[3]);
	srv.request.theta = atof(argv[4]);
	//srand(time(0));
	//srv.request.x =  ((double)rand() / ((double)RAND_MAX + 1.0)) * (maxSpawn - minSpawn + 1.0) + minSpawn;
	//srv.request.y =  ((double)rand() / ((double)RAND_MAX + 1.0)) * (maxSpawn - minSpawn + 1.0) + minSpawn;
	add_turtle.call(srv);
	node.getParam("o_rr", o_rr);
	node.getParam("rr", rr);
	node.getParam("ro", ro);
	node.getParam("ra", ra);
	node.getParam("angular_vel", angular_vel);
	node.getParam("speed", speed);
	node.getParam("size", turtleSize);
	node.getParam("sight", sight);
	sight = ((M_PI/180.0) * sight)/2.0;
	string publisher_param = turtle_name + "/command_velocity";
	ros::Publisher turtle_vel = node.advertise<turtlesim::Velocity>(publisher_param.c_str(), 10);
	turtle_name ="/" + turtle_name;

	tf::TransformListener listener;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
	ros::Rate rate(10.0);
		
	//std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/start");

	while(node.ok()){
		geometry_msgs::Twist transform;
		tf::StampedTransform own_trans;
		tf::StampedTransform neighbor_trans;
		stringstream ss;
		bool all_set = false;
		bool influence = false;
		bool collide = false;
		double d_vx = 0.0;
		double d_vy = 0.0;
		ros::spinOnce();
		try{
			listener.lookupTransform("/world",turtle_name.c_str(), ros::Time(0), own_trans);
		} catch (tf::TransformException ex){
			rate.sleep();
			ROS_ERROR("%s",ex.what());
			continue;
		}
		double current_angle = tf::getYaw(own_trans.getRotation());
		//ROS_ERROR("%s ---->> %.2f", turtle_name.c_str(), tf::getYaw(own_trans.getRotation()));
		int count =0;
		for(int i=2; i <= turtleSize; i++){
			ss.str("");
			ss << "/turtle" << i;
			string name = ss.str();
	 		if(name == turtle_name) continue;

			try{
				listener.waitForTransform("/world", name.c_str(), ros::Time(0), ros::Duration(0.1) );
				listener.lookupTransform("/world", name.c_str(), ros::Time(0), neighbor_trans);
				/* check neighbors velocity(direction and speed), and Angle(z-coordinate) */
				//listener.lookupTwist(name.c_str(), "/world", name.c_str(), tf::Vector3(0,0,0), "/world", ros::Time(0.0), ros::Duration(0.1), transform);
			} catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				break;
				continue;
			}
			count++;
			double vx = neighbor_trans.getOrigin().x() - own_trans.getOrigin().x();
			double vy = neighbor_trans.getOrigin().y() - own_trans.getOrigin().y();
			double distance = norm(vx,vy);
			vx /= distance;
			vy /= distance;
			double angle = angle_gap(current_angle, atan2(vy,vx)); 

			if(distance <= ra && fabs(angle) <= sight){
				influence = true;
				if(distance <= rr){
					d_vx -= vx;
					d_vy -= vy;
					collide = true;
				}
				if(collide) continue;
				if( distance > rr && distance <= ro){
					vx = cos(tf::getYaw(neighbor_trans.getRotation()));
					vy = sin(tf::getYaw(neighbor_trans.getRotation()));
					//ROS_ERROR("------------[%.2f,%.2f,%.2f]",vx,vy,tf::getYaw(neighbor_trans.getRotation()));
					d_vx += vx / norm(vx,vy);
					d_vy += vy / norm(vx,vy);
				} else if ( distance > ro && distance <= ra){
					d_vx += vx; d_vy +=vy;
				} 
			}
			all_set = true;
		}
		
		turtlesim::Velocity vel_msg;
		//ROS_ERROR("collide = %d", collide);
		double desired =0.0;
		if(count == turtleSize-2){
			if(influence){
				desired = angle_gap(current_angle,atan2(d_vy,d_vx));

				//ROS_ERROR("influence::: ------------[%.2f,%.2f,%.2f]",d_vx,d_vy,desired);
				if( fabs(desired) < angular_vel){
					if(desired < 0){ desired = -angular_vel;} else {desired = angular_vel;}
				} 
			}
			vel_msg.angular = desired;
			if(all_set){
				vel_msg.linear = speed;
			} else {vel_msg.linear = 0;}
			turtle_vel.publish(vel_msg);
		}
		rate.sleep();
	}
	return 0;
};

