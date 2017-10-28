#include "dji_m100_rob560/local_position_control.h"
#include "dji_sdk/dji_sdk.h"

// Global variables
ros::ServiceClient          query_version_service;
ros::ServiceClient          set_loc_pos_ref_service;
ros::ServiceClient          sdk_ctrl_authority_service;
ros::ServiceClient          drone_task_service;
ros::Publisher              ctrl_signal_pub;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix      current_gps_position;
uint8_t                     flight_status               = 255;
//uint8_t                     display_mode                = 255;
uint8_t                     current_gps_health          = 0;
int                         num_targets                 = 0;
Target 						*target 					= new Target(2);

int main(int argc, char** argv) {
	ros::init(argc, argv, "local_position_control_node");
	ros::NodeHandle nh;

	// Services
	query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>	("dji_sdk/query_drone_version");
	set_loc_pos_ref_service    = nh.serviceClient<dji_sdk::SetLocalPosRef>		("dji_sdk/set_local_pos_ref");
	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>	("dji_sdk/sdk_control_authority");
	drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>	("dji_sdk/drone_task_control");

	// Subscribe to messages from dji_sdk_node
	ros::Subscriber flight_status_sub = nh.subscribe("dji_sdk/flight_status",   10, &flight_status_callback);
	//ros::Subscriber display_mode_sub  = nh.subscribe("dji_sdk/display_mode",    10, &display_mode_callback);
	ros::Subscriber gps_pos_sub       = nh.subscribe("dji_sdk/gps_position",    10, &gps_position_callback);
	ros::Subscriber gps_health_sub    = nh.subscribe("dji_sdk/gps_health",      10, &gps_health_callback);
	ros::Subscriber loc_pos_sub       = nh.subscribe("dji_sdk/local_position",  10, &local_position_callback);

	// Publish the control signal
	ctrl_signal_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

	bool ready = false;
	if(is_M100() && set_local_position()) {
		ready = obtain_control(true) ? monitored_takeoff() : false;
	}
	else {
		ROS_ERROR("Only the M100 is supported!");
	}

	if(ready) target->setState(1);

	ros::spin();
	return 0;
}

// Services
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

	// If M100 is not in the air after 10 seconds, fail.
	while (ros::Time::now() - start_time < ros::Duration(10)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || (current_gps_position.altitude - home_altitude < 1.0)) {
		ROS_ERROR("Take-off failed.");
		return false;
	}
	else {
		// start_time = ros::Time::now();
		ROS_INFO("Successful take-off!");
		ros::spinOnce();
	}
	return true;
}

// Callbacks
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
	flight_status = msg->data;
}

/*void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg) {
	display_mode = msg->data;
}*/

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
	current_gps_health = msg->data;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	local_position = *msg;

	static ros::Time start_time = ros::Time::now();
	ros::Duration elapsed_time = ros::Time::now() - start_time;

	// Down sampled to 50Hz loop
	if (elapsed_time >= ros::Duration(0.02)) {
		if (current_gps_health > 3) {
			start_time = ros::Time::now();
			switch(target->getState()) {
				case 0:	break;
				case 1:
					target->set(10, 15, 25, 2);
					target->setState(99);
					break;
				case 2:
					target->set(-10, 5, 5, 2);
					target->setState(99);
					break;
				case 99:
					local_position_ctrl();
					break;
			}
		}
		else {
			ROS_WARN("Cannot execute local position control. Not enough GPS satellites");
			target->setState(0);
		}
	}
}

// Control function
void local_position_ctrl() {
	static double xCmd, yCmd, zCmd;
	xCmd = target->getX() - local_position.point.x;
	yCmd = target->getY() - local_position.point.y;
	zCmd = target->getZ();

	sensor_msgs::Joy controlPosYaw;
	controlPosYaw.axes.push_back(xCmd);
	controlPosYaw.axes.push_back(yCmd);
	controlPosYaw.axes.push_back(zCmd);
	controlPosYaw.axes.push_back(target->getYaw());
	ctrl_signal_pub.publish(controlPosYaw);

	// 0.1 m is the minimum error to reach target in x, y, and z directions.
	// This error threshold will have to change depending on aircraft/payload/wind conditions.
	if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) && (local_position.point.z > (zCmd - 0.1)) && (local_position.point.z < (zCmd + 0.1))) {
		target->nextState();
	}
}

Target::Target(int num) {
	_offset_x = _offset_y = _offset_z = _yaw = 0.0;
	_state = _last_state = 0;
	_num_targets = num;
}

void Target::set(float x, float y, float z, float yaw) {
	_offset_x = x;
	_offset_y = y;
	_offset_z = z;
	_yaw = yaw;
}

void Target::setState(int state) {
	_last_state = _state;
	_state = state;
}

void Target::nextState() {
	if(_last_state < _num_targets) {
		ROS_INFO("%d of %d targets complete.", _last_state, _num_targets);
		_state = _last_state + 1;
	}
	else {
		ROS_INFO("All %d targets completed.", _num_targets);
		_state = 0;
	}
}

float Target::getX() {
	return _offset_x;
}

float Target::getY() {
	return _offset_y;
}

float Target::getZ() {
	return _offset_z;
}

float Target::getYaw() {
	return _yaw;
}

int Target::getState() {
	return _state;
}