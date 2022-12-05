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
from std_msgs.msg import Header
from tello_msgs.msg import FlightData, FlipControl


class TelloRosWrapper(Node):
    # Set inverval that will be printing the drone's battery
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

    # Timers
    _current_battery_percentage_timer = None

    # Topics
    image_topic_name = "/tello/camera/image_raw"
    flight_data_topic_name = "/tello/flight_data"
    velocity_command_topic_name = "/tello/cmd_vel"
    land_topic_name = "/tello/land"
    takeoff_topic_name = "/tello/takeoff"
    flip_control_topic_name = "/tello/flip"

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
        print("[INFO] - Initiating the Node...")
        self._connect_to_tello_network()

        self._init_publisher()
        self._init_subscribers()
        self._init_timers()

        self._start_camera_image_thread()
        print("[INFO] - Node is ready!")

    def _init_publisher(self):
        print("[INFO] - Initializing the publishers.")
        self._camera_image_publisher = self.create_publisher(
            Image, self.image_topic_name, 1
        )
        self._flight_data_publisher = self.create_publisher(
            FlightData, self.flight_data_topic_name, 10
        )

    def _init_subscribers(self):
        print("[INFO] - Initializing the subscribers.")
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._flight_data_callback)

        self._velocity_command_subscriber = self.create_subscription(
            Twist, self.velocity_command_topic_name, self.command_velocity_callback, 1
        )

        self._land_subscriber = self.create_subscription(
            Header, self.land_topic_name, self._land_callback, 1
        )
        self._takeoff_subscriber = self.create_subscription(
            Header, self.takeoff_topic_name, self._takeoff_callback, 1
        )

        self._flip_control_subscriber = self.create_subscription(
            FlipControl, self.flip_control_topic_name, self._flip_control_callback, 1
        )

    def _init_timers(self):
        print("[INFO] - Initializing the timers.")
        self._current_battery_percentage_timer = self.create_timer(
            self._battery_percentage_timer_interval,
            self._current_battery_percentage_callback,
        )

    def _start_camera_image_thread(self):
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._camera_image_callback)
        self._video_thread.start()

    # -------------
    # - Callbacks -
    # -------------
    def command_velocity_callback(self, msg: Twist):
        self.tello.set_pitch(msg.linear.x)  # linear X
        self.tello.set_roll(-msg.linear.y)  # linear Y
        self.tello.set_throttle(msg.linear.z)  # linear Z
        self.tello.set_yaw(msg.angular.z)  # angular Z

    def _land_callback(self, msg: Header):
        msg  # remove linter error
        self.tello.land()

    def _takeoff_callback(self, msg: Header):
        msg  # remove linter error
        self.tello.takeoff()

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
        flight_data.electrical_machinery_state = data.electrical_machinery_state
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
        flight_data.temperature_height = data.temperature_height

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
        print(f"[INFO] - Battery percentage: {self.current_battery_percentage}%")

    def _flip_control_callback(self, msg: FlipControl):
        if msg.flip_forward:
            self.tello.flip_forward()
        elif msg.flip_backward:
            self.tello.flip_back()
        elif msg.flip_right:
            self.tello.flip_right()
        elif msg.flip_left:
            self.tello.flip_left()

    def _camera_image_callback(self):
        video_stream = self.tello.get_video_stream()
        container = av.open(video_stream)

        print("[INFO] - video stream is starting")

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1
                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            # VIDEO RESOLUTION
            # original 960x720

            # Reduced image size to have less delay
            image = cv2.resize(image, (480, 360), interpolation=cv2.INTER_LINEAR)

            # convert OpenCV image => ROS Image message
            image = self._cv_bridge.cv2_to_imgmsg(image, "rgb8")

            self._camera_image_publisher.publish(image)

            # check for normal shutdown
            if self._stop_request.isSet():
                return

    def _connect_to_tello_network(self):
        print("[INFO] - Connecting to drone")
        if self._auto_connect_to_wifi:
            if not ctwd.connect_device(self.tello_ssid, self.tello_pw, verbose=False):
                print("[ERROR] - Connection to drone unsuccessful!")
                self.signal_shutdown = True

        self.tello.connect()
        from tellopy._internal import error

        try:
            self.tello.wait_for_connection(5)
        except error.TelloError:
            print("[ERROR] - Connection to drone unsuccessful!")
            self.signal_shutdown = True
            return

        print("[INFO] - Connection to drone successfull")

    def shutdown_rountine(self):
        print("[INFO] - Shutdown routine started")
        print("[INFO] - Landing...")
        self.tello.land()

        print("[INFO] - Stoping publishing the camera image.")
        self._stop_request.set()
        self._video_thread.join(timeout=2)

        time.sleep(2)

        print("[INFO] - Stoping the communication between the drone")
        self.tello.quit()
        print("[INFO] - Shutdown routine completed!")
