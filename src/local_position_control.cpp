#include "dji_m100_rob560/local_position_control.h"
#include "dji_sdk/dji_sdk.h"

// Global variables
ros::ServiceClient          query_version_service;
ros::ServiceClient          set_loc_pos_ref_service;
ros::ServiceClient          sdk_ctrl_authority_service;
ros::ServiceClient          drone_task_service;
ros::Publisher              ctrl_signal_pub;
uint8_t                     flight_status               = 255;
uint8_t                     display_mode                = 255;
uint8_t                     current_gps_health          = 0;
int                         num_targets                 = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix      current_gps_position;
int                         target_state                = 0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_position_control_node");
  ros::NodeHandle nh;

  // Services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_loc_pos_ref_service    = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flight_status_sub = nh.subscribe("dji_sdk/flight_status",   10, &flight_status_callback);
  ros::Subscriber display_mode_sub  = nh.subscribe("dji_sdk/display_mode",    10, &display_mode_callback);
  ros::Subscriber gps_pos_sub       = nh.subscribe("dji_sdk/gps_position",    10, &gps_position_callback);
  ros::Subscriber gps_health_sub    = nh.subscribe("dji_sdk/gps_health",      10, &gps_health_callback);
  ros::Subscriber loc_pos_sub       = nh.subscribe("dji_sdk/local_position",  10, &local_position_callback);

  // Publish the control signal
  ctrl_signal_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  bool takeoff_result;
  if(is_M100() && set_local_position()) {
    takeoff_result = obtain_control(true) ? monitored_takeoff() : false;
  }
  else {
    ROS_ERROR("Only the M100 is supported!");
  }

  if(takeoff_result) {
    //! Enter total number of Targets
    num_targets = 2;
    //! Start Mission by setting Target state to 1
    target_state = 1;
  }

  ros::spin();
  return 0;
}

bool is_M100() {
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  return (query.response.version == DJISDK::DroneFirmwareVersion::M100_31);
}

bool set_local_position() {
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_loc_pos_ref_service.call(localPosReferenceSetter);

  return localPosReferenceSetter.response.result;
}

bool obtain_control(bool b) {
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = b ? 1 : 0;
  sdk_ctrl_authority_service.call(authority);

  return authority.response.result;
}

bool takeoff_land(int task) {
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = task;
  drone_task_service.call(droneTaskControl);

  return droneTaskControl.response.result;
}

bool monitored_takeoff() {
  ros::Time start_time = ros::Time::now();
  float home_altitude = current_gps_position.altitude;

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) return false;
  ROS_INFO("M100 taking off!");

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || (current_gps_position.altitude - home_altitude < 1.0)) {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else {
    // start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

// Callbacks
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg) {
  display_mode = msg->data;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  local_position = *msg;
  double xCmd, yCmd, zCmd;
  sensor_msgs::Joy controlPosYaw;

  // Down sampled to 50Hz loop
  if (elapsed_time > ros::Duration(0.02)) {
    start_time = ros::Time::now();
    if (target_state == 1) {
      //! First arbitrary target
      if (current_gps_health > 3) {
        setTarget(10, 15, 25, 2);
        local_position_ctrl(xCmd, yCmd, zCmd);
      }
      else
      {
        ROS_INFO("Cannot execute Local Position Control");
        ROS_INFO("Not enough GPS Satellites");
        //! Set Target set state to 0 in order to stop Local position control mission
        target_state = 0;
      }
    }

    if (target_state == 2) {
      //! Second arbitrary target
      if (current_gps_health > 3) {
        setTarget(-10, 5, 5, 2);
        local_position_ctrl(xCmd, yCmd, zCmd);
      }
      else
      {
        ROS_INFO("Cannot execute Local Position Control");
        ROS_INFO("Not enough GPS Satellites");
        //! Set Target set state to 0 in order to stop Local position control mission
        target_state = 0;
      }
    }
  }
}

// Control function
/*!
 * This function calculates the difference between target and current local position
 * and sends the commands to the Position and Yaw control topic.
 *
 */
void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd) {
  xCmd = target_offset_x - local_position.point.x;
  yCmd = target_offset_y - local_position.point.y;
  zCmd = target_offset_z;

  sensor_msgs::Joy controlPosYaw;
  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(target_yaw);
  ctrl_signal_pub.publish(controlPosYaw);

  // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
  // This error threshold will have to change depending on aircraft/payload/wind conditions.
  if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) &&
      (local_position.point.z > (target_offset_z - 0.1)) && (local_position.point.z < (target_offset_z + 0.1))) {
    if(target_state <= num_targets) {
      ROS_INFO("%d of %d target(s) complete", target_state, num_targets);
      target_state++;
    }
    else
    {
      target_state = 0;
    }
  }
}