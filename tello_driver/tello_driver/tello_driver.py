import time
from rclpy.node import Node
import threading
import cv2
import numpy as np
import av
from cv_bridge import CvBridge
from tellopy import Tello
from tello_driver import connect_to_wifi_device as ctwd

# ROS messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int32
from tello_msgs.msg import FlightData, FlipControl


class TelloRosWrapper(Node):
    # Set inverval that will be self.get_logger().infoing the drone's battery
    _battery_percentage_timer_interval = 5  # seconds
    _frame_skip = 300

    tello_ssid = None
    tello_pw = None
    current_battery_percentage = 0

    # Publishers
    _camera_image_publisher = None
    _flight_data_publisher = None

    # Subscribers
    _velocity_command_subscriber = None
    _land_subscriber = None
    _takeoff_subscriber = None
    _flip_control_subscriber = None
    _throw_and_go_subscriber = None
    _palm_land_subscriber = None
    _set_att_limit_subscriber = None

    # Timers
    _current_battery_percentage_timer = None

    # Topics
    image_topic_name = "/camera/image_raw"
    flight_data_topic_name = "/flight_data"
    velocity_command_topic_name = "/cmd_vel"
    land_topic_name = "/land"
    takeoff_topic_name = "/takeoff"
    flip_control_topic_name = "/flip"
    throw_and_go_topic_name = "/throw_and_go"
    palm_land_topic_name = "/palm_land"
    set_att_limit_topic_name = "/set_att_limit"

    # Flags
    _auto_connect_to_wifi = False

    signal_shutdown = False

    def __init__(self, node_name):
        super().__init__(node_name)
        # ROS OpenCv bridge
        self._cv_bridge = CvBridge()

        self.tello = Tello()
        self.tello.set_loglevel(self.tello.LOG_WARN)

    def begin(self):
        self.get_logger().info("Initiating the Node...")

        self.read_parameters()

        self._connect_to_tello_network()

        self._init_publisher()
        self._init_subscribers()
        self._init_timers()

        self._start_camera_image_thread()
        self.get_logger().info("Node is ready!")

    def _init_publisher(self):
        self.get_logger().info("Initializing the publishers.")
        self._camera_image_publisher = self.create_publisher(
            Image, self.image_topic_name, 10
        )
        self._flight_data_publisher = self.create_publisher(
            FlightData, self.flight_data_topic_name, 10
        )

    def _init_subscribers(self):
        self.get_logger().info("Initializing the subscribers.")
        self.tello.subscribe(
            self.tello.EVENT_FLIGHT_DATA, self._flight_data_callback
        )

        self._velocity_command_subscriber = self.create_subscription(
            Twist,
            self.velocity_command_topic_name,
            self.command_velocity_callback,
            1,
        )

        self._land_subscriber = self.create_subscription(
            Empty, self.land_topic_name, self._land_callback, 1
        )
        self._takeoff_subscriber = self.create_subscription(
            Empty, self.takeoff_topic_name, self._takeoff_callback, 1
        )

        self._flip_control_subscriber = self.create_subscription(
            FlipControl,
            self.flip_control_topic_name,
            self._flip_control_callback,
            1,
        )
        # self._throw_and_go_subscriber = self.subscriptions(
        #     Empty, self.throw_and_go_topic_name, self._throw_and_go_callback, 1
        # )
        # self._palm_land_subscriber = self.subscriptions(
        #     Empty, self.palm_land_topic_name, self._palm_land_callback, 1
        # )
        # self._set_att_limit_subscriber = self.subscriptions(
        #     Int32,
        #     self.set_att_limit_topic_name,
        #     self._set_att_limit_callback,
        #     1,
        # )

    def _init_timers(self):
        self.get_logger().info("Initializing the timers.")
        self._current_battery_percentage_timer = self.create_timer(
            self._battery_percentage_timer_interval,
            self._current_battery_percentage_callback,
        )

    def read_parameters(self):
        self.get_logger().info("Reading parameters...")

        self.declare_parameter("image_topic_name", "/camera/image_raw")
        self.declare_parameter("flight_data_topic_name", "flight_data")
        self.declare_parameter("velocity_command_topic_name", "/cmd_vel")
        self.declare_parameter("land_topic_name", "/land")
        self.declare_parameter("takeoff_topic_name", "/takeoff")
        self.declare_parameter("flip_control_topic_name", "/flip")
        self.declare_parameter("auto_wifi_connection", False)
        self.declare_parameter("tello_ssid", "tello01")
        self.declare_parameter("tello_pw", "")

        self.image_topic_name = (
            self.get_parameter("image_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.flight_data_topic_name = (
            self.get_parameter("flight_data_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.velocity_command_topic_name = (
            self.get_parameter("velocity_command_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.land_topic_name = (
            self.get_parameter("land_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.takeoff_topic_name = (
            self.get_parameter("takeoff_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.flip_control_topic_name = (
            self.get_parameter("flip_control_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._auto_connect_to_wifi = (
            self.get_parameter("auto_wifi_connection")
            .get_parameter_value()
            .bool_value
        )

        self.tello_ssid = (
            self.get_parameter("tello_ssid").get_parameter_value().string_value
        )
        self.tello_pw = (
            self.get_parameter("tello_pw").get_parameter_value().string_value
        )

        self.get_logger().info("Finished reading parameters!")

    def _start_camera_image_thread(self):
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(
            target=self._camera_image_callback
        )
        self._video_thread.start()

    # -------------
    # - Callbacks -
    # -------------
    def command_velocity_callback(self, msg: Twist):
        self.tello.set_pitch(msg.linear.x)  # linear X
        self.tello.set_roll(-msg.linear.y)  # linear Y
        self.tello.set_throttle(msg.linear.z)  # linear Z
        self.tello.set_yaw(-msg.angular.z)  # angular Z

    def _land_callback(self, msg: Empty):
        msg  # remove linter error
        self.tello.land()

    def _takeoff_callback(self, msg: Empty):
        msg  # remove linter error
        self.tello.takeoff()

    def _palm_land_callback(self, msg: Empty):
        msg  # remove linter error
        self.tello.palm_land()

    def _set_att_limit_callback(self, msg: Int32):
        pass

    def _throw_and_go_callback(self, msg: Empty):
        pass

    def _flip_control_callback(self, msg: FlipControl):
        if msg.flip_forward:
            self.tello.flip_forward()
        elif msg.flip_backward:
            self.tello.flip_back()
        elif msg.flip_right:
            self.tello.flip_right()
        elif msg.flip_left:
            self.tello.flip_left()
        elif msg.flip_forward_left:
            self.tello.flip_forwardleft()
        elif msg.flip_forward_right:
            self.tello.flip_forwardright()
        elif msg.flip_back_left:
            self.tello.flip_backleft()
        elif msg.flip_back_right:
            self.tello.flip_backward_right()

    def _flight_data_callback(self, event, sender, data):
        flight_data = FlightData()

        # - Battery data
        flight_data.battery_low = data.battery_low
        flight_data.battery_lower = data.battery_lower
        flight_data.battery_percentage = data.battery_percentage
        flight_data.drone_battery_left = data.drone_battery_left
        # flight_data.drone_fly_time_left = data.drone_fly_time_left

        # =========================================================================

        # - States
        flight_data.battery_state = data.battery_state
        flight_data.camera_state = data.camera_state
        flight_data.electrical_machinery_state = (
            data.electrical_machinery_state
        )
        flight_data.down_visual_state = data.down_visual_state
        flight_data.gravity_state = data.gravity_state
        flight_data.imu_calibration_state = data.imu_calibration_state
        flight_data.imu_state = data.imu_state
        flight_data.power_state = data.power_state
        flight_data.pressure_state = data.pressure_state
        flight_data.wind_state = data.wind_state

        # =========================================================================

        # - Stats
        flight_data.drone_hover = data.drone_hover
        flight_data.em_open = data.em_open
        flight_data.em_sky = data.em_sky
        flight_data.em_ground = data.em_ground
        flight_data.factory_mode = data.factory_mode
        flight_data.fly_mode = data.fly_mode
        # flight_data.fly_time = data.fly_time
        flight_data.front_in = data.front_in
        flight_data.front_lsc = data.front_lsc
        flight_data.front_out = data.front_out

        # =========================================================================

        # - Sensors
        flight_data.fly_speed = data.fly_speed
        flight_data.east_speed = data.east_speed
        flight_data.ground_speed = data.ground_speed
        flight_data.height = data.height
        flight_data.light_strength = data.light_strength
        flight_data.north_speed = data.north_speed
        flight_data.temperature_high = data.temperature_height

        # =========================================================================

        # - Other
        flight_data.outage_recording = data.outage_recording
        flight_data.smart_video_exit_mode = data.smart_video_exit_mode
        # flight_data.throw_fly_timer = data.throw_fly_timer

        # =========================================================================

        # - WiFi
        flight_data.wifi_disturb = data.wifi_disturb
        flight_data.wifi_strength = data.wifi_strength

        self.current_battery_percentage = flight_data.battery_percentage

        # - Publish Flight data
        self._flight_data_publisher.publish(flight_data)

    def _current_battery_percentage_callback(self):
        self.get_logger().info(
            f"Battery percentage: {self.current_battery_percentage}%"
        )

    def _camera_image_callback(self):
        video_stream = self.tello.get_video_stream()
        container = av.open(video_stream)

        self.get_logger().info("video stream is starting")

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1
                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            # VIDEO RESOLUTION
            # original 960x720

            # Reduced image size to have less delay
            image = cv2.resize(
                image, (960, 720), interpolation=cv2.INTER_LINEAR
            )

            # convert OpenCV image => ROS Image message
            image = self._cv_bridge.cv2_to_imgmsg(image, "rgb8")

            self._camera_image_publisher.publish(image)

            # check for normal shutdown
            if self._stop_request.isSet():
                return

    def _connect_to_tello_network(self):
        self.get_logger().info("Connecting to drone")
        if self._auto_connect_to_wifi:
            if not ctwd.connect_device(
                self.tello_ssid, self.tello_pw, verbose=False
            ):
                self.get_logger().error("Connection to drone unsuccessful!")
                self.signal_shutdown = True

        self.tello.connect()
        from tellopy._internal import error

        try:
            self.tello.wait_for_connection(5)
        except error.TelloError:
            self.get_logger().error("Connection to drone unsuccessful!")
            self.signal_shutdown = True
            return

        self.get_logger().info(" to drone successfull")

    def shutdown_rountine(self):
        self.get_logger().info("Shutdown routine started")
        self.get_logger().info("Landing...")
        self.tello.land()

        self.get_logger().info("Stoping publishing the camera image.")
        self._stop_request.set()
        self._video_thread.join(timeout=2)

        time.sleep(2)

        self.get_logger().info("Stoping the communication between the drone")
        self.tello.quit()
        self.get_logger().info("Shutdown routine completed!")
