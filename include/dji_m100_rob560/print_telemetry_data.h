#ifndef PRINT_TELEMETRY_DATA_H
#define PRINT_TELEMETRY_DATA_H

// ROS
#include <ros/ros.h>

// ROS standard messages
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

// Callback functions
void rc_callback(const sensor_msgs::Joy::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void battery_callback(const sensor_msgs::BatteryState::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void height_callback(const std_msgs::Float32::ConstPtr& msg);
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
void angular_rate_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void acceleration_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void trigger_callback(const sensor_msgs::TimeReference::ConstPtr& msg);

#endif // PRINT_TELEMTRY_DATA_H