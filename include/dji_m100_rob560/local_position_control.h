#ifndef LOCAL_POSITION_CONTROL_H
#define LOCAL_POSITION_CONTROL_H

// ROS
#include <ros/ros.h>

// Standard messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>

//DJI OSDK services
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneTaskControl.h>

// Initialize drone (setup, obtain control, take off)
bool is_M100();
bool set_local_position();
bool obtain_control(bool b);
bool takeoff_land(int task);
bool monitored_takeoff();

// Messages callbacks
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
//void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

// Control function
void local_position_ctrl();

// ***************************************
class Target {
	public:
		Target(int num);
		void set(float x, float y, float z, float yaw);
		void setState(int state);
		void nextState();
		float getX();
		float getY();
		float getZ();
		float getYaw();
		int getState();

	private:
		float _offset_x, _offset_y, _offset_z, _yaw;
		int _state, _last_state, _num_targets;

};

#endif //LOCAL_POSITION_CONTROL_H