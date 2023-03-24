from rclpy.node import Node
from pynput import keyboard

# ROS messages
from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class Controller(Node):
    # - Topics
    tello_vel_cmd_stamped_topic_name = "/cmd_vel"
    tello_takeoff_topic_name = "/takeoff"
    tello_land_topic_name = "/land"
    tello_flip_control_topic_name = "/flip"

    def __init__(self, node_name):
        super().__init__(node_name)
        self.print_controls()
        self.key_pressed = {
            "th": 0.0,
            "right": 0.0,
            "forward": 0.0,
            "cw": 0.0,
        }
        self.speed = 0.5  # from 0 - 1
        self._keyboard_listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.shift_key_pressed = False
        self.shutdown = False

    def print_controls(self):
        print("---------------------")
        print("- Movement Controls -")
        print("---------------------")
        print("w: pitch forward")
        print("s: pitch backward")
        print("a: roll left")
        print("d: roll right")
        print("up arrow: more altitude")
        print("down arrow: less altitude")
        print("left arrow: yaw ccw")
        print("right arrow: yaw cw")
        print("--------------------")
        print("- General Controls -")
        print("--------------------")
        print("t: takeoff")
        print("l: land")
        print("x: more speed")
        print("z: less speed")
        print("-----------------")
        print("- Flip Controls -")
        print("-----------------")
        print("shift + up arrow: flip forward")
        print("shift + down arrow: flip backward")
        print("shift + left arrow: flip left")
        print("shift + right arrow: flip right")
        print("shift + u: flip forward left")
        print("shift + i: flip forward right")
        print("shift + j: flip backwards left")
        print("shift + k: flip backwards right")
        print("--------------------------------------------")

    def begin(self):
        self.init_pub()
        self.init_timers()
        self._keyboard_listener.start()

    def init_timers(self):
        self.cmd_vel_timer = self.create_timer(0.05, self.cmd_vel_callback)

    def init_pub(self):
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.tello_vel_cmd_stamped_topic_name, 1
        )

        self._takeoff_pub = self.create_publisher(
            Empty, self.tello_takeoff_topic_name, 1
        )

        self._land_pub = self.create_publisher(
            Empty, self.tello_land_topic_name, 1
        )

        self._flip_control_pub = self.create_publisher(
            FlipControl, self.tello_flip_control_topic_name, 1
        )

    def on_press(self, key):
        print(f"pressing the key {key}")
        try:
            if key.char == "w":
                self.key_pressed["forward"] = self.speed
            if key.char == "s":
                self.key_pressed["forward"] = -self.speed
            if key.char == "d":
                self.key_pressed["right"] = -self.speed
            if key.char == "a":
                self.key_pressed["right"] = self.speed
            if key.char == "t":
                self._takeoff_pub.publish(Empty())
            if key.char == "l":
                self._land_pub.publish(Empty())
            if key.char == "z":
                self.speed -= 0.1
                if self.speed < 0.1:
                    self.speed = 0.1
            if key.char == "x":
                self.speed += 0.1
                if self.speed > 1:
                    self.speed = 1
        except AttributeError:
            pass

        try:
            if key == key.shift:
                self.shift_key_pressed = True
            if key == key.up and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = True
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
                msg.flip_right = False
            if key == key.down and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = True
                msg.flip_left = False
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
            if key == key.left and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = True
                msg.flip_right = False
                self._flip_control_pub.publish(msg)
            if key == key.right and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = True
                self._flip_control_pub.publish(msg)

            if key == key.up and not self.shift_key_pressed:
                self.key_pressed["th"] = self.speed
            if key == key.down and not self.shift_key_pressed:
                self.key_pressed["th"] = -self.speed
            if key == key.left and not self.shift_key_pressed:
                self.key_pressed["cw"] = self.speed
            if key == key.right and not self.shift_key_pressed:
                self.key_pressed["cw"] = -self.speed

        except AttributeError:
            if key.char == "u" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = True
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "i" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = True
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "j" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = True
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "k" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = True
                self._flip_control_pub.publish(msg)

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.shutdown = True
            return False
        try:
            if key.char == "w":
                self.key_pressed["forward"] = 0.0
            if key.char == "s":
                self.key_pressed["forward"] = 0.0
            if key.char == "d":
                self.key_pressed["right"] = 0.0
            if key.char == "a":
                self.key_pressed["right"] = 0.0
        except AttributeError:
            pass

        try:
            if key == key.shift:
                self.shift_key_pressed = False

            if key == key.up and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.down and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.left and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key == key.right and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)

            if key == key.up and not self.shift_key_pressed:
                self.key_pressed["th"] = 0.0
            if key == key.down and not self.shift_key_pressed:
                self.key_pressed["th"] = 0.0
            if key == key.left and not self.shift_key_pressed:
                self.key_pressed["cw"] = 0.0
            if key == key.right and not self.shift_key_pressed:
                self.key_pressed["cw"] = 0.0

        except AttributeError:
            if key.char == "u" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "i" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "j" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)
            if key.char == "k" and self.shift_key_pressed:
                msg = FlipControl()
                msg.flip_forward = False
                msg.flip_backward = False
                msg.flip_left = False
                msg.flip_right = False
                msg.flip_forward_left = False
                msg.flip_forward_right = False
                msg.flip_back_left = False
                msg.flip_back_right = False
                self._flip_control_pub.publish(msg)

    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.key_pressed["forward"]
        msg.linear.y = self.key_pressed["right"]
        msg.linear.z = self.key_pressed["th"]

        msg.angular.z = self.key_pressed["cw"]
        self.cmd_vel_pub.publish(msg)
