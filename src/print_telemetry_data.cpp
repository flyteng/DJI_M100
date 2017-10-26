#include "dji_m100_rob560/print_telemetry_data.h"
#include "dji_sdk/dji_sdk.h"

// Global variables for subscribed topics
sensor_msgs::Joy                  rc_state;
geometry_msgs::Quaternion         attitude_state;
sensor_msgs::BatteryState         battery_state;
sensor_msgs::Imu                  imu_state;
uint8_t                           flight_status_state;
uint8_t                           gps_health_state;
sensor_msgs::NavSatFix            gps_position_state;
std_msgs::Float32                 height_state;
geometry_msgs::Vector3Stamped     velocity_state;
geometry_msgs::PointStamped       local_position_state;
uint8_t                           display_mode_state;
geometry_msgs::Vector3Stamped     angular_rate_state;
geometry_msgs::Vector3Stamped     acceleration_state;
sensor_msgs::TimeReference        trigger_state;

// Main function
int main(int argc, char** argv) {
  ros::init(argc, argv, "print_telemetry_data_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber rc              = nh.subscribe("dji_sdk/rc",                        10, &rc_callback);
  ros::Subscriber attitude        = nh.subscribe("dji_sdk/attitude",                  10, &attitude_callback);
  ros::Subscriber battery         = nh.subscribe("dji_sdk/battery_state",             10, &battery_callback);
  ros::Subscriber imu             = nh.subscribe("dji_sdk/imu",                       10, &imu_callback);
  ros::Subscriber flight_status   = nh.subscribe("dji_sdk/flight_status",             10, &flight_status_callback);
  ros::Subscriber gps_health      = nh.subscribe("dji_sdk/gps_health",                10, &gps_health_callback);
  ros::Subscriber gps_position    = nh.subscribe("dji_sdk/gps_position",              10, &gps_position_callback);
  ros::Subscriber height          = nh.subscribe("dji_sdk/height_above_takeoff",      10, &height_callback);
  ros::Subscriber velocity        = nh.subscribe("dji_sdk/velocity",                  10, &velocity_callback);
  ros::Subscriber local_position  = nh.subscribe("dji_sdk/local_position",            10, &local_position_callback);
  ros::Subscriber display_mode    = nh.subscribe("dji_sdk/display_mode",              10, &display_mode_callback);
  ros::Subscriber angular_rate    = nh.subscribe("dji_sdk/angular_velocity_fused",    10, &angular_rate_callback);
  ros::Subscriber acceleration    = nh.subscribe("dji_sdk/acceleration_ground_fused", 10, &acceleration_callback);
  ros::Subscriber trigger         = nh.subscribe("dji_sdk/trigger_time",              10, &trigger_callback);
  
  print_data();
  ros::spin();
  return 0;
}

// Print data from FC in terminal
void print_data() {
  ROS_INFO("Roll: %.3f \t Pitch: %.3f \t Yaw: %.3f \t Throttle: %.3f \t Mode: %.3f \t Gear: %.3f",
            rc_state.axes[0], rc_state.axes[1], rc_state.axes[2], rc_state.axes[3], rc_state.axes[4], rc_state.axes[5]);
  ROS_INFO("q = [%.3f, %.3f, %.3f, %.3f]",
            attitude_state.w, attitude_state.x, attitude_state.y, attitude_state.z);
  ROS_INFO("Battery: %.3f",
            battery_state.percentage);
  ROS_INFO("IMU lin. acc. = [%.3f, %.3f, %.3f]",
            imu_state.linear_acceleration.x, imu_state.linear_acceleration.y, imu_state.linear_acceleration.z);
  ROS_INFO("IMU ang. vel. = [%.3f, %.3f, %.3f]",
            imu_state.angular_velocity.x, imu_state.angular_velocity.y, imu_state.angular_velocity.z);
  ROS_INFO("Flight status: %u",
            flight_status_state);
  ROS_INFO("GPS health: %u",
            gps_health_state);
  ROS_INFO("Latitude: %.3f \t Longitude: %.3f \t Altitude: %.3f",
            gps_position_state.latitude, gps_position_state.longitude, gps_position_state.altitude);
  ROS_INFO("Height: %.3f",
            height_state.data);
  ROS_INFO("Velocity = [%.3f, %.3f, %.3f]",
            velocity_state.vector.x, velocity_state.vector.y, velocity_state.vector.z);
  ROS_INFO("Local position = [%.3f, %.3f, %.3f]",
            local_position_state.point.x, local_position_state.point.y, local_position_state.point.z);
  ROS_INFO("Display mode: %u",
            display_mode_state);
  ROS_INFO("Ang. rate = [%.3f, %.3f, %.3f]",
            angular_rate_state.vector.x, angular_rate_state.vector.y, angular_rate_state.vector.z);
  ROS_INFO("Acceleration = [%.3f, %.3f, %.3f]",
            acceleration_state.vector.x, acceleration_state.vector.y, acceleration_state.vector.z);
  //ROS_INFO("Trigger: ",
  //          trigger_state.time_ref);
}

// Callback functions
void rc_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  rc_state.axes = msg->axes;
  
  /*  
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().roll     / 10000.0));
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().pitch    / 10000.0));
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().yaw      / 10000.0));
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().throttle / 10000.0));
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().mode));
      rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().gear));
  */
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  attitude_state = msg->quaternion;

  /*  
      Following REP 103 to use FLU for
      body frame. The quaternion is the rotation
      from body_FLU to ground_ENU

      q.quaternion.w = q_FLU2ENU.getW();
      q.quaternion.x = q_FLU2ENU.getX();
      q.quaternion.y = q_FLU2ENU.getY();
      q.quaternion.z = q_FLU2ENU.getZ();
  */
}

void battery_callback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  battery_state = *msg;

  /*  
      msg_battery_state.percentage = vehicle->broadcast->getBatteryInfo().percentage;
      msg_battery_state.present = (vehicle->broadcast->getBatteryInfo().voltage!=0);
  */
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_state = *msg;

  /*
    imu.linear_acceleration.x =  vehicle->broadcast->getAcceleration().x * gravity_const;
    imu.linear_acceleration.y = -vehicle->broadcast->getAcceleration().y * gravity_const;
    imu.linear_acceleration.z = -vehicle->broadcast->getAcceleration().z * gravity_const;

    imu.angular_velocity.x    =  vehicle->broadcast->getAngularRate().x;
    imu.angular_velocity.y    = -vehicle->broadcast->getAngularRate().y;
    imu.angular_velocity.z    = -vehicle->broadcast->getAngularRate().z;

    imu.orientation.w = q_FLU2ENU.getW();
    imu.orientation.x = q_FLU2ENU.getX();
    imu.orientation.y = q_FLU2ENU.getY();
    imu.orientation.z = q_FLU2ENU.getZ();
  */
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
  flight_status_state = msg->data;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  gps_health_state = msg->data;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  gps_position_state = *msg;

  /*
    gps_pos.latitude        = global_pos.latitude * 180 / C_PI;
    gps_pos.longitude       = global_pos.longitude * 180 / C_PI;
    gps_pos.altitude        = global_pos.altitude;
  */
}

void height_callback(const std_msgs::Float32::ConstPtr& msg) {
  height_state = *msg;

  /*
    msg.data = height
  */
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  velocity_state = *msg;

  /*
    velocity.vector.x = vehicle->broadcast->getVelocity().y;
    velocity.vector.y = vehicle->broadcast->getVelocity().x;
    velocity.vector.z = vehicle->broadcast->getVelocity().z;
  */
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  local_position_state = *msg;

  /*
    Following REP 103 to use ENU for short-range Cartesian representations.
    Local position is published in ENU Frame

    gpsConvertENU(local_pos.point.x, local_pos.point.y, gps_pos.longitude, gps_pos.latitude,
                  this->local_pos_ref_longitude, this->local_pos_ref_latitude);
    local_pos.point.z = gps_pos.altitude - this->local_pos_ref_altitude;
  */
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg) {
  display_mode_state = msg->data;
}

void angular_rate_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  angular_rate_state = *msg;

  /*
    Following REP 103 to use FLU for body frame

    angular_rate.vector.x        =  w_FC.x;
    angular_rate.vector.y        = -w_FC.y; //y,z sign are flipped from RD to LU
    angular_rate.vector.z        = -w_FC.z;
  */
}

void acceleration_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  acceleration_state = *msg;

  /*
    Following REP 103 to use ENU for short-range Cartesian representations.
    This is in ground frame.

    acceleration.vector.x        = a_FC.y;  //x, y are swapped from NE to EN
    acceleration.vector.y        = a_FC.x;
    acceleration.vector.z        = a_FC.z;  //z sign is already U
  */
}

void trigger_callback(const sensor_msgs::TimeReference::ConstPtr& msg) {
  trigger_state = *msg;

  /*
    trigTime.time_ref     = now_time;
    trigTime.source       = "FC";
  */
}