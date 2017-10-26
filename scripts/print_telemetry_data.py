import dji_sdk
import rospy
from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import Imu, NavSatFix, Joy, TimeReference, BatteryState
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped

# Callback functions
def rc_callback(data):
  rospy.loginfo('Roll: %.3f \t Pitch: %.3f \t Yaw: %.3f \t Throttle: %.3f \t Mode: %.3f \t Gear: %.3f', data.axes[0], data.axes[1], data.axes[2], data.axes[3], data.axes[4], data.axes[5])

def attitude_callback(data):
  rospy.loginfo('q = [%.3f, %.3f, %.3f, %.3f]', data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z)

def battery_callback(data):
  rospy.loginfo('Battery: %.3f', data.percentage)

def imu_callback(data):
  rospy.loginfo('IMU lin. acc. = [%.3f, %.3f, %.3f]', data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
  rospy.loginfo('IMU ang. vel. = [%.3f, %.3f, %.3f]', data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

def flight_status_callback(data):
  rospy.loginfo('Flight status: %u', data.data)

def gps_health_callback(data):
  rospy.loginfo('GPS health: %u', data.data)

def gps_position_callback(data):
  rospy.loginfo('Latitude: %.3f \t Longitude: %.3f \t Altitude: %.3f', data.latitude, data.longitude, data.altitude)

def height_callback(data):
  rospy.loginfo('Height: %.3f', data.data)

def velocity_callback(data):
  rospy.loginfo('Velocity = [%.3f, %.3f, %.3f]', data.vector.x, data.vector.y, data.vector.z)

def local_position_callback(data):
  rospy.loginfo('Local position = [%.3f, %.3f, %.3f]', data.point.x, data.point.y, data.point.z)

def display_mode_callback(data):
  rospy.loginfo('Display mode: %u', data.data)

def angular_rate_callback(data):
  rospy.loginfo('Ang. rate = [%.3f, %.3f, %.3f]', data.vector.x, data.vector.y, data.vector.z)

def acceleration_callback(data):
  rospy.loginfo('Acceleration = [%.3f, %.3f, %.3f]', data.vector.x, data.vector.y, data.vector.z)

def trigger_callback(data):
  #rospy.loginfo('Trigger: ', data.time_ref)

def main():
  rospy.init_node('print_telemetry_data_py', anonymous=True)

  # Subscribe to messages from dji_sdk_node
  rospy.Subscriber('dji_sdk/rc',                        Joy,                rc_callback);
  rospy.Subscriber('dji_sdk/attitude',                  QuaternionStamped,  attitude_callback);
  rospy.Subscriber('dji_sdk/battery_state',             BatteryState,       battery_callback);
  rospy.Subscriber('dji_sdk/imu',                       Imu,                imu_callback);
  rospy.Subscriber('dji_sdk/flight_status',             UInt8,              flight_status_callback);
  rospy.Subscriber('dji_sdk/gps_health',                UInt8,              gps_health_callback);
  rospy.Subscriber('dji_sdk/gps_position',              NavSatFix,          gps_position_callback);
  rospy.Subscriber('dji_sdk/height_above_takeoff',      Float32,            height_callback);
  rospy.Subscriber('dji_sdk/velocity',                  Vector3Stamped,     velocity_callback);
  rospy.Subscriber('dji_sdk/local_position',            PointStamped,       local_position_callback);
  rospy.Subscriber('dji_sdk/display_mode',              UInt8,              display_mode_callback);
  rospy.Subscriber('dji_sdk/angular_velocity_fused',    Vector3Stamped,     angular_rate_callback);
  rospy.Subscriber('dji_sdk/acceleration_ground_fused', Vector3Stamped,     acceleration_callback);
  rospy.Subscriber('dji_sdk/trigger_time',              TimeReference,      trigger_callback);

  rospy.spin()

if __name__ == '__main__':
  main()