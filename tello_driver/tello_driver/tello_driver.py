import time
from rclpy.node import Node
import threading
import cv2
import numpy as np
import av
from cv_bridge import CvBridge
from tellopy import Tello
from tellopy._internal.tello import LogData
from tello_driver import connect_to_wifi_device as ctwd

# ROS messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros.transform_broadcaster import TransformBroadcaster
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int32
from tello_msgs.msg import FlightStats, FlipControl

from tello_driver.serializers import (
    create_tf_between_odom_drone,
    generate_flight_data_msg,
    generate_imu_msg,
    generate_odom_msg,
)


class TelloRosWrapper(Node):
    """A ROS Node that wraps the tellopy library to control the Tello drone."""

    _battery_percentage_timer_interval = 5  # seconds

    _current_battery_percentage = 0

    # Publishers
    _camera_image_publisher = None
    _flight_data_publisher = None
    _odometry_publisher = None
    _imu_publisher = None
    _tfb = None

    # Subscribers
    _velocity_command_subscriber = None
    _land_subscriber = None
    _takeoff_subscriber = None
    _flip_control_subscriber = None
    _throw_and_go_subscriber = None
    _palm_land_subscriber = None
    _set_alt_limit_subscriber = None
    _toggle_fast_mode_subscriber = None
    _camera_exposure_subscriber = None

    # Timers
    _current_battery_percentage_timer = None
    _reset_cmd_vel_timer = None

    # Topics
    _image_topic_name = "camera/image_raw"
    _flight_data_topic_name = "flight_data"
    _velocity_command_topic_name = "cmd_vel"
    _land_topic_name = "land"
    _takeoff_topic_name = "takeoff"
    _flip_control_topic_name = "flip"
    _throw_and_go_topic_name = "throw_and_go"
    _palm_land_topic_name = "palm_land"
    _set_alt_limit_topic_name = "set_att_limit"
    _odom_topic_name = "odom"
    _imu_topic_name = "imu"
    _toggle_fast_mode_topic_name = "toggle_fast_mode"
    _camera_exposure_topic_name = "camera/exposure"

    # Frame Ids
    _odom_frame_id = "odom"
    _imu_frame_id = "imu"
    _drone_frame_id = "tello"

    # Flags
    _auto_connect_to_wifi = False

    # Wifi Setup
    _tello_ssid = "tello-XXXXXX"
    _tello_pw = ""

    # Settings
    _alt_limit = 30
    _fast_mode = False
    _video_mode = (
        "4:3"  # Valid options: "4:3" (wider view) or "16:9" (better quality)
    )
    _camera_exposure = 0  # Valid values: 0, 1, 2

    _last_cmd_vel_time = 0

    def __init__(self, node_name) -> None:
        """Initialize the TelloRosWrapper class.

        Args:
            node_name (str): The name of the node.
        """
        super().__init__(node_name)
        self._cv_bridge = CvBridge()

        self.tello = Tello()
        self.tello.set_loglevel(self.tello.LOG_INFO)
        self.begin()

    def begin(self) -> None:
        """Start all the necessary components of the node."""
        self.get_logger().info("Initiating the Node...")

        self.read_parameters()
        self._setup_initial_settings()

        self._connect_to_tello_network()

        self._init_publisher()
        self._init_subscribers()
        self._init_timers()

        self._start_camera_image_thread()
        self.get_logger().info("Tello driver node is ready!")

    def _setup_initial_settings(self) -> None:
        zoom = False
        if self._video_mode == "16:9":
            zoom = True
        elif self._video_mode != "4:3":
            self.get_logger().warn(
                "Invalid video mode! Only 4:3 and 16:9 are valid"
            )

        self.tello.set_video_mode(zoom)
        self.tello.set_exposure(self._camera_exposure)
        self.tello.set_alt_limit(self._alt_limit)
        self.tello.fast_mode = self._fast_mode

    def _init_publisher(self) -> None:
        """Initialize all the required publishers."""
        self.get_logger().info("Initializing the publishers.")
        self._camera_image_publisher = self.create_publisher(
            Image, self._image_topic_name, 10
        )
        self._flight_data_publisher = self.create_publisher(
            FlightStats, self._flight_data_topic_name, 1
        )
        self._odometry_publisher = self.create_publisher(
            Odometry, self._odom_topic_name, 1
        )
        self._imu_publisher = self.create_publisher(
            Imu, self._imu_topic_name, 1
        )
        self._tfb = TransformBroadcaster(self)

    def _init_subscribers(self) -> None:
        """Initialize all the required subscribers."""
        self.get_logger().info("Initializing the subscribers.")
        self.tello.subscribe(
            self.tello.EVENT_FLIGHT_DATA, self._flight_data_callback
        )

        self.tello.subscribe(
            self.tello.EVENT_LOG_DATA, self._log_data_callback
        )

        self._velocity_command_subscriber = self.create_subscription(
            Twist,
            self._velocity_command_topic_name,
            self.command_velocity_callback,
            1,
        )

        self._land_subscriber = self.create_subscription(
            Empty, self._land_topic_name, self._land_callback, 1
        )
        self._takeoff_subscriber = self.create_subscription(
            Empty, self._takeoff_topic_name, self._takeoff_callback, 1
        )

        self._flip_control_subscriber = self.create_subscription(
            FlipControl,
            self._flip_control_topic_name,
            self._flip_control_callback,
            1,
        )
        self._throw_and_go_subscriber = self.create_subscription(
            Empty,
            self._throw_and_go_topic_name,
            self._throw_and_go_callback,
            1,
        )
        self._palm_land_subscriber = self.create_subscription(
            Empty, self._palm_land_topic_name, self._palm_land_callback, 1
        )
        self._set_alt_limit_subscriber = self.create_subscription(
            Int32,
            self._set_alt_limit_topic_name,
            self._set_alt_limit_callback,
            1,
        )

        self._toggle_fast_mode_subscriber = self.create_subscription(
            Empty,
            self._toggle_fast_mode_topic_name,
            self._toggle_fast_mode_callback,
            1,
        )

        self.create_subscription(
            Int32,
            self._camera_exposure_topic_name,
            self._camera_exposure_callback,
            1,
        )

    def _init_timers(self) -> None:
        """Initialize all the required timers."""
        self.get_logger().info("Initializing the timers.")
        self._current_battery_percentage_timer = self.create_timer(
            self._battery_percentage_timer_interval,
            self._current_battery_percentage_callback,
        )

    def _start_camera_image_thread(self) -> None:
        """Start the thread that receives the camera image from the drone."""
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(
            target=self._camera_image_callback
        )
        self._video_thread.start()

    # -------------
    # - Callbacks -
    # -------------
    def command_velocity_callback(self, msg: Twist) -> None:
        """Callback for the velocity command subscriber.

        Args:
            msg (Twist): The message containing the velocity command.
        """

        self.tello.set_pitch(msg.linear.x)  # linear X
        self.tello.set_roll(-msg.linear.y)  # linear Y
        self.tello.set_throttle(msg.linear.z)  # linear Z
        self.tello.set_yaw(-msg.angular.z)  # angular Z

        self._last_cmd_vel_time = self.get_clock().now().nanoseconds

    def _reset_cmd_vel_callback(self) -> None:
        """Callback for the reset cmd_vel timer.
        If no cmd_vel is received withing 0.09 seconds, the drone will
        stop moving.
        """
        if (
            self.get_clock().now().nanoseconds - self._last_cmd_vel_time
            > 90000
        ):
            self.tello.set_pitch(0)
            self.tello.set_roll(0)
            self.tello.set_throttle(0)
            self.tello.set_yaw(0)

    def _toggle_fast_mode_callback(self, msg: Empty) -> None:
        """Callback for the toggle fast mode subscriber.

        Args:
            msg (Empty): The message containing the toggle fast mode command.
        """
        msg
        self._fast_mode = not self._fast_mode
        self.tello.fast_mode = self._fast_mode

    def _camera_exposure_callback(self, msg: Int32) -> None:
        """Callback for the camera exposure subscriber.

        Args:
            msg (Int32): The message containing the camera exposure command.
        """
        if msg.data in [0, 1, 2]:
            self.tello.set_exposure(msg.data)
        else:
            self.get_logger().warn(
                "Invalid exposure value! Only 0, 1, 2 are valid"
            )

    def _land_callback(self, msg: Empty) -> None:
        """Callback for the land subscriber.

        Args:
            msg (Empty): The message containing the land command.
        """
        msg  # remove linter error
        self.tello.land()

    def _takeoff_callback(self, msg: Empty) -> None:
        """Callback for the takeoff subscriber.

        Args:
            msg (Empty): The message containing the takeoff command.
        """
        msg  # remove linter error
        self.tello.takeoff()
        self.tello.set_alt_limit(self._alt_limit)
        print("Altitude limit:", self.tello.get_alt_limit())
        print("Atitude limit:", self.tello.get_att_limit())

    def _palm_land_callback(self, msg: Empty) -> None:
        """Callback for the palm land subscriber.

        Args:
            msg (Empty): The message containing the palm land command.
        """
        msg  # remove linter error
        self.tello.palm_land()

    def _set_alt_limit_callback(self, msg: Int32) -> None:
        """Callback for the set attitude limit subscriber.

        Args:
            msg (Int32): The message containing the attitude limit.
        """
        self.tello.set_att_limit(msg.data)
        self._alt_limit = msg.data

    def _throw_and_go_callback(self, msg: Empty) -> None:
        """Callback for the throw and go subscriber.

        Args:
            msg (Empty): The message containing the throw and go command.
        """
        msg  # remove linter error
        self.tello.throw_and_go()

    def _flip_control_callback(self, msg: FlipControl) -> None:
        """Callback for the flip control subscriber.

        Args:
            msg (FlipControl): The message containing the flip command.
        """
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

    def _log_data_callback(self, event, sender, data: LogData) -> None:
        """Callback for the log data subscriber from tellopy."""
        # calling event and sender to ignore linter error
        event
        sender

        now = self.get_clock().now()

        odom_msg = generate_odom_msg(now, self._odom_frame_id, data)
        imu_msg = generate_imu_msg(now, self._imu_frame_id, data)
        tf_odom_drone = create_tf_between_odom_drone(
            now, self._odom_frame_id, self._drone_frame_id, data
        )

        self._tfb.sendTransform(tf_odom_drone)
        self._odometry_publisher.publish(odom_msg)
        self._imu_publisher.publish(imu_msg)

    def _flight_data_callback(self, event, sender, data) -> None:
        """Callback for the flight data subscriber from tellopy."""
        # calling event and sender to ignore linter error
        event
        sender

        flight_data_msg = generate_flight_data_msg(data)

        self._current_battery_percentage = flight_data_msg.battery_percentage

        # - Publish Flight data
        self._flight_data_publisher.publish(flight_data_msg)

    def _current_battery_percentage_callback(self):
        """Callback for the current battery percentage timer."""
        self.get_logger().info(
            f"Battery percentage: {self._current_battery_percentage}%"
        )

    def _camera_image_callback(self) -> None:
        """Callback for the camera image subscriber."""
        video_stream = self.tello.get_video_stream()
        container = av.open(video_stream)

        self.get_logger().info("video stream is starting")

        for frame in container.decode(video=0):
            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

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

    def _connect_to_tello_network(self) -> None:
        """Connect to the Tello drone network automatically."""
        self.get_logger().info("Connecting to drone")
        if self._auto_connect_to_wifi:
            if not ctwd.connect_device(
                self._tello_ssid, self._tello_pw, verbose=False
            ):
                self.get_logger().error("Connection to drone unsuccessful!")

        self.tello.connect()
        from tellopy._internal import error

        try:
            self.tello.wait_for_connection(5)
        except error.TelloError:
            self.get_logger().error("Connection to drone unsuccessful!")
            return

        self.get_logger().info(" to drone successfull")

    def read_parameters(self) -> None:
        """Read all the required parameters."""
        self.get_logger().info("Reading parameters...")

        self.declare_parameter("image_topic_name", self._image_topic_name)
        self.declare_parameter(
            "flight_data_topic_name", self._flight_data_topic_name
        )
        self.declare_parameter(
            "velocity_command_topic_name", self._velocity_command_topic_name
        )
        self.declare_parameter("land_topic_name", self._land_topic_name)
        self.declare_parameter("takeoff_topic_name", self._takeoff_topic_name)
        self.declare_parameter(
            "flip_control_topic_name", self._flip_control_topic_name
        )
        self.declare_parameter(
            "auto_wifi_connection", self._auto_connect_to_wifi
        )
        self.declare_parameter("tello_ssid", self._tello_ssid)
        self.declare_parameter("tello_pw", self._tello_pw)
        self.declare_parameter("odom_topic_name", self._odom_topic_name)
        self.declare_parameter("imu_topic_name", self._imu_topic_name)
        self.declare_parameter("odom_frame_id", self._odom_frame_id)
        self.declare_parameter("imu_frame_id", self._imu_frame_id)
        self.declare_parameter("drone_frame_id", self._drone_frame_id)
        self.declare_parameter("fast_mode", self._fast_mode)
        self.declare_parameter("video_mode", self._video_mode)
        self.declare_parameter("camera_exposure", self._camera_exposure)
        self.declare_parameter("alt_limit", self._alt_limit)

        self._image_topic_name = (
            self.get_parameter("image_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._flight_data_topic_name = (
            self.get_parameter("flight_data_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._velocity_command_topic_name = (
            self.get_parameter("velocity_command_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._land_topic_name = (
            self.get_parameter("land_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._takeoff_topic_name = (
            self.get_parameter("takeoff_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._flip_control_topic_name = (
            self.get_parameter("flip_control_topic_name")
            .get_parameter_value()
            .string_value
        )
        self._auto_connect_to_wifi = (
            self.get_parameter("auto_wifi_connection")
            .get_parameter_value()
            .bool_value
        )

        self._tello_ssid = (
            self.get_parameter("tello_ssid").get_parameter_value().string_value
        )
        self._tello_pw = (
            self.get_parameter("tello_pw").get_parameter_value().string_value
        )
        self._odom_frame_id = (
            self.get_parameter("odom_frame_id")
            .get_parameter_value()
            .string_value
        )
        self._imu_frame_id = (
            self.get_parameter("imu_frame_id")
            .get_parameter_value()
            .string_value
        )
        self._drone_frame_id = (
            self.get_parameter("drone_frame_id")
            .get_parameter_value()
            .string_value
        )
        self._imu_topic_name = (
            self.get_parameter("imu_topic_name")
            .get_parameter_value()
            .string_value
        )

        self._fast_mode = (
            self.get_parameter("fast_mode").get_parameter_value().bool_value
        )
        self._video_mode = (
            self.get_parameter("video_mode").get_parameter_value().string_value
        )
        self._camera_exposure = (
            self.get_parameter("camera_exposure")
            .get_parameter_value()
            .integer_value
        )
        self._alt_limit = (
            self.get_parameter("alt_limit").get_parameter_value().integer_value
        )

        self.get_logger().info("Finished reading parameters!")

    def shutdown_rountine(self) -> None:
        """Shutdown routine for the node. Makes sure the drone lands first."""
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
