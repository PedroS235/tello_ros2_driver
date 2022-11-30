import rclpy
from rclpy.node import Node
from tellopy import Tello
import threading
import cv2
import numpy as np
import av
from cv_bridge import CvBridge
from tello_ros_wrapper import connect_to_wifi_device as ctwd
import time

# ROS messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header


# TODO
# [] - Add a land callback
# [] - Add a takeoff callback
# [] - Add a flip _flight_data_callback
# [] - Publish flight data once we have the ros msg

class TelloRosWrapper(Node):
    # Constants
    CURRENT_BATTERY_PERCENTAGE_TIMER_INTERVAL = 10  # (s)
    _frame_skip = 300

    tello_ssid = None
    tello_pw = None
    current_battery_percentage = -1

    # Publishers
    _camera_image_publisher = None

    # Subscribers
    _velocity_command_subscriber = None
    _land_subscriber = None
    _takeoff_subscriber = None

    # Timers
    _current_battery_percentage_timer = None

    # Topics
    image_topic_name = "/tello/camera/image_raw"
    flight_data_topic_name = "/tello/flight_data"
    velocity_command_topic_name = "/tello/cmd_vel_stamped"
    land_topic_name = "/tello/land"
    takeoff_topic_name = "/tello/takeoff"

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
            Image, self.image_topic_name, 1)

    def _init_subscribers(self):
        print("[INFO] - Initializing the subscribers.")
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA,
                             self._flight_data_callback)

        self._velocity_command_subscriber = self.create_subscription(
            TwistStamped, self.velocity_command_topic_name,
            self.command_velocity_callback, 1)

        self._land_subscriber = self.create_subscription(
            Header, self.land_topic_name, self._land_callback, 1)
        self._takeoff_subscriber = self.create_subscription(
            Header, self.takeoff_topic_name, self._takeoff_callback, 1)

    def _init_timers(self):
        print("[INFO] - Initializing the timers.")
        self._current_battery_percentage_timer = self.create_timer(
            self.CURRENT_BATTERY_PERCENTAGE_TIMER_INTERVAL,
            self._current_battery_percentage_callback)

    def _start_camera_image_thread(self):
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(
            target=self._camera_image_callback)
        self._video_thread.start()

    # -------------
    # - Callbacks -
    # -------------
    def command_velocity_callback(self, msg: TwistStamped):
        self.tello.set_pitch(msg.twist.linear.x)  # linear X
        self.tello.set_roll(msg.twist.linear.y)  # linear Y
        self.tello.set_throttle(msg.twist.linear.z)  # linear Z
        self.tello.set_yaw(msg.twist.angular.z)  # angular Z

    def _land_callback(self, msg: Header):
        msg  # remove linter error
        self.tello.land()

    def _takeoff_callback(self, msg: Header):
        msg  # remove linter error
        self.tello.takeoff()

    def _flight_data_callback(self, event, sender, data):
        pass

    def _current_battery_percentage_callback(self):
        print(
            f"[info] [Tello_driver] - Drone's battery percentage is {self.current_battery_percentage}%"
        )

    def _camera_image_callback(self):
        # get video stream, open with PyAV
        video_stream = self.tello.get_video_stream()
        time.sleep(5)
        container = av.open(video_stream)

        print("[info] [Tello_driver] - video stream is starting")

        for frame in container.decode(video=0):
            if self._frame_skip > 0:
                self._frame_skip = self._frame_skip - 1
                continue

            # convert PyAV frame => PIL image => OpenCV image
            image = np.array(frame.to_image())

            # VIDEO RESOLUTION
            # original 960x720

            # Reduced image size to have less delay
            image = cv2.resize(image, (480, 360),
                               interpolation=cv2.INTER_LINEAR)

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
