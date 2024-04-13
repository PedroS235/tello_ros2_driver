from rclpy.time import Time

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import BatteryState, Imu
from tello_msgs.msg import FlightStats
from tellopy._internal.tello import LogData, FlightData


def generate_odom_msg(
    time: Time, odom_frame_id: str, data: LogData
) -> Odometry:
    msg = Odometry()

    msg.header.stamp = time.to_msg()
    msg.header.frame_id = odom_frame_id

    # Note that this pose is very inaccurate.
    msg.pose.pose.position.x = data.mvo.pos_x
    msg.pose.pose.position.y = -data.mvo.pos_y
    msg.pose.pose.position.z = data.mvo.pos_z

    msg.pose.pose.orientation.x = data.imu.q1
    msg.pose.pose.orientation.y = -data.imu.q2
    msg.pose.pose.orientation.z = -data.imu.q3
    msg.pose.pose.orientation.w = data.imu.q0

    msg.twist.twist.linear.x = data.mvo.vel_x
    msg.twist.twist.linear.y = data.mvo.vel_x
    msg.twist.twist.linear.z = data.mvo.vel_x

    return msg


def generate_imu_msg(time: Time, imu_frame_id: str, data: LogData) -> Imu:
    msg = Imu()

    msg.header.frame_id = imu_frame_id
    msg.header.stamp = time.to_msg()

    msg.orientation.x = data.imu.q1
    msg.orientation.y = -data.imu.q2
    msg.orientation.z = -data.imu.q3
    msg.orientation.w = data.imu.q0

    msg.angular_velocity.x = data.imu.gyro_x
    msg.angular_velocity.y = data.imu.gyro_y
    msg.angular_velocity.z = data.imu.gyro_z

    msg.linear_acceleration.x = data.imu.acc_x
    msg.linear_acceleration.y = data.imu.acc_y
    msg.linear_acceleration.z = data.imu.acc_z

    return msg


def create_tf_between_odom_drone(
    time: Time, drone_frame_id: str, odom_frame_id: str, data: LogData
) -> TransformStamped:
    tf_odom_drone = TransformStamped()

    tf_odom_drone.header.stamp = time.to_msg()
    tf_odom_drone.header.frame_id = odom_frame_id
    tf_odom_drone._child_frame_id = drone_frame_id

    # Note that this pose is very inaccurate.
    tf_odom_drone.transform.translation.x = data.mvo.pos_x
    tf_odom_drone.transform.translation.y = -data.mvo.pos_y
    tf_odom_drone.transform.translation.z = data.mvo.pos_z

    tf_odom_drone.transform.rotation.x = data.imu.q1
    tf_odom_drone.transform.rotation.y = -data.imu.q2
    tf_odom_drone.transform.rotation.z = -data.imu.q3
    tf_odom_drone.transform.rotation.w = data.imu.q0

    return tf_odom_drone


def generate_battery_state_msg(time: Time, data: FlightData) -> BatteryState:
    msg = BatteryState()
    # NOTE: A tello battery is a 1S HV battery, so 4.35V is the max voltage
    max_v = 4.35
    min_v = 3.30  # Assumed value

    msg.percentage = float(data.battery_percentage)
    msg.voltage = min_v + (max_v - min_v) * data.battery_percentage / 100.0
    msg.cell_voltage = [msg.voltage]
    msg.header.stamp = time.to_msg()
    msg.design_capacity = 1.100  # 1.1 Ah

    # TODO: Possibly add the battery state. Not clear what the values are for the moment

    return msg


def generate_flight_data_msg(data: FlightData) -> FlightStats:
    msg = FlightStats()

    # - Battery data
    msg.battery_low = data.battery_low
    msg.battery_lower = data.battery_lower
    msg.battery_percentage = data.battery_percentage
    msg.drone_battery_left = data.drone_battery_left
    # flight_data.drone_fly_time_left = data.drone_fly_time_left

    # =========================================================================

    # - States
    msg.battery_state = data.battery_state
    msg.camera_state = data.camera_state
    msg.electrical_machinery_state = data.electrical_machinery_state
    msg.down_visual_state = data.down_visual_state
    msg.gravity_state = data.gravity_state
    msg.imu_calibration_state = data.imu_calibration_state
    msg.imu_state = data.imu_state
    msg.power_state = data.power_state
    msg.pressure_state = data.pressure_state
    msg.wind_state = data.wind_state

    # =========================================================================

    # - Stats
    msg.drone_hover = data.drone_hover
    msg.em_open = data.em_open
    msg.em_sky = data.em_sky
    msg.em_ground = data.em_ground
    msg.factory_mode = data.factory_mode
    msg.fly_mode = data.fly_mode
    # flight_data.fly_time = data.fly_time
    msg.front_in = data.front_in
    msg.front_lsc = data.front_lsc
    msg.front_out = data.front_out

    # =========================================================================

    # - Sensors
    msg.fly_speed = data.fly_speed
    msg.east_speed = data.east_speed
    msg.ground_speed = data.ground_speed
    msg.height = data.height
    msg.light_strength = data.light_strength
    msg.north_speed = data.north_speed
    msg.temperature_high = data.temperature_height

    # =========================================================================

    # - Other
    msg.outage_recording = data.outage_recording
    msg.smart_video_exit_mode = data.smart_video_exit_mode
    # flight_data.throw_fly_timer = data.throw_fly_timer

    # =========================================================================

    # - WiFi
    msg.wifi_disturb = data.wifi_disturb
    msg.wifi_strength = data.wifi_strength

    return msg
