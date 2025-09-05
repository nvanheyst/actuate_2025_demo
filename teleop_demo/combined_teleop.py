#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from pan_tilt_msgs.msg import PanTiltCmdDeg

import sys
import select
import termios
import tty

# --- Configuration ---
# Set the namespace for the robot base. Leave empty for global.
ROBOT_NAMESPACE = '/j100_0667' 

# Key mappings for robot base movement
MOVE_BINDINGS = {
    'i': (1, 0, 0, 0), 'o': (1, 0, 0, -1), 'j': (0, 0, 0, 1), 'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1), ',': (-1, 0, 0, 0), '.': (-1, 0, 0, 1), 'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0), 'I': (1, 0, 0, 0), 'J': (0, 1, 0, 0), 'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0), '<': (-1, 0, 0, 0), '>': (-1, -1, 0, 0), 'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0), 'b': (0, 0, -1, 0),
}

# Key mappings for adjusting robot speed
SPEED_BINDINGS = {
    'q': (1.1, 1.1), 'z': (0.9, 0.9), 'w': (1.1, 1), 'x': (0.9, 1),
    'e': (1, 1.1), 'c': (1, 0.9),
}

# Arrow keys send escape sequences. e.g., Up is '\x1b[A'
PTU_BINDINGS = {
    '\x1b[A': (0, -1),  # Up Arrow: Tilt Down
    '\x1b[B': (0, 1),   # Down Arrow: Tilt Up
    '\x1b[D': (1, 0),   # Left Arrow: Pan Left
    '\x1b[C': (-1, 0),  # Right Arrow: Pan Right
}

INSTRUCTIONS = """
--------------------------------------------------
  Combined Robot and Pan-Tilt Unit Controller
--------------------------------------------------
Robot Base Controls:      | Pan-Tilt Controls:
  u    i    o             |   [Up Arrow]
  j    k    l             | [Left] [Down] [Right]
  m    ,    .             |
                          | [Space] : Reset PTU
--------------------------------------------------
- Hold <Shift> for strafing (holonomic mode).
- q/z: Increase/decrease all speeds by 10%
- w/x: Increase/decrease linear speed by 10%
- e/c: Increase/decrease angular speed by 10%
- Press <k> or anything else to stop the robot.
- CTRL-C or q to quit
--------------------------------------------------
"""

def get_key(settings):
    # Get a key press, handles escape sequences for arrow keys.
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_status(speed, turn, pan, tilt):
    # Prints the current status of the robot and PTU.
    status_msg = (
        f"Robot Speed: {speed:.2f} | Robot Turn: {turn:.2f} | "
        f"PTU Pan: {pan:.1f}° | PTU Tilt: {tilt:.1f}°"
    )
    sys.stdout.write(f"\r{status_msg}      ")
    sys.stdout.flush()

class CombinedTeleopNode(Node):
    def __init__(self):
        super().__init__('combined_teleop_node')

        # --- Hardcoded settings for direct python3 execution ---
        # PTU Settings
        self.ptu_step = 2.0
        self.max_pan = 55.0
        self.min_pan = -55.0
        self.max_tilt = 55.0
        self.min_tilt = -55.0
        # Robot Settings
        self.speed = 0.5
        self.turn = 1.0
        
        # Robot State
        self.x, self.y, self.z, self.th = 0, 0, 0, 0
        # PTU State
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        # Construct topic names based on the hardcoded namespace
        twist_topic = f"{ROBOT_NAMESPACE}/cmd_vel" if ROBOT_NAMESPACE else "cmd_vel"
        # The C++ driver subscribes to a global topic, so we must use it.
        ptu_topic = "/pan_tilt_cmd_deg"

        # ROS2 Communications
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.twist_pub = self.create_publisher(Twist, twist_topic, 10)
        self.ptu_pub = self.create_publisher(PanTiltCmdDeg, ptu_topic, qos_profile)
        self.timer = self.create_timer(0.1, self.publish_commands) # 10 Hz publish rate

        self.get_logger().info("Combined Teleop Node Initialized.")
        self.get_logger().info(f"Publishing Twist to: {self.twist_pub.topic_name}")
        self.get_logger().info(f"Publishing PTU commands to: {self.ptu_pub.topic_name}")

    def publish_commands(self):
        # Publish Robot Twist Command
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.turn
        self.twist_pub.publish(twist)

        # Publish PTU Command
        ptu_cmd = PanTiltCmdDeg()
        ptu_cmd.speed = 30
        ptu_cmd.yaw = self.pan_angle
        ptu_cmd.pitch = self.tilt_angle
        self.ptu_pub.publish(ptu_cmd)

    def stop_robot(self):
        # Publishes a zero-velocity Twist message.
        twist = Twist()
        self.twist_pub.publish(twist)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = CombinedTeleopNode()
    
    print(INSTRUCTIONS)
    
    try:
        while rclpy.ok():
            key = get_key(settings)

            # Robot Control Logic
            if key in MOVE_BINDINGS:
                node.x = MOVE_BINDINGS[key][0]
                node.y = MOVE_BINDINGS[key][1]
                node.z = MOVE_BINDINGS[key][2]
                node.th = MOVE_BINDINGS[key][3]
            elif key in SPEED_BINDINGS:
                node.speed *= SPEED_BINDINGS[key][0]
                node.turn *= SPEED_BINDINGS[key][1]
            else:
                # Any other key press stops the robot (except PTU keys and quit)
                if key not in PTU_BINDINGS and key != ' ':
                    node.x, node.y, node.z, node.th = 0, 0, 0, 0
                if key == 'q' or key == '\x03': # Quit on 'q' or CTRL-C
                    break
            
            # PTU Control Logic
            if key in PTU_BINDINGS:
                node.pan_angle += PTU_BINDINGS[key][0] * node.ptu_step
                node.tilt_angle += PTU_BINDINGS[key][1] * node.ptu_step

                # Clamp values to stay within limits
                node.pan_angle = max(node.min_pan, min(node.max_pan, node.pan_angle))
                node.tilt_angle = max(node.min_tilt, min(node.max_tilt, node.tilt_angle))
            elif key == ' ':
                node.pan_angle = 0.0
                node.tilt_angle = 0.0
            
            print_status(node.speed, node.turn, node.pan_angle, node.tilt_angle)
            # Process ROS2 events
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        print("\nStopping robot and shutting down.")
        node.stop_robot()
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


