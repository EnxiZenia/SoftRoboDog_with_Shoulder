import math
import time
import signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Joy
from builtin_interfaces.msg import Time as RosTime

from my_parameters.msg import JoystickParameters  # Your existing custom message
from my_parameters.msg import ObstacleProximityLog  # Your logging message


class CmdVelToJoystick(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joystick')

        # State variables
        self.goal_active = False
        self.override_enabled = False
        self.cmd_vel_enabled = True
        self.alpha_b= math.pi/12
        self.last_button6 = 0
        self.last_button7 = 0
        self.distance_zone = 1
        self.turn_counter = 0
        self.last_turn_command = False

        self.start_time = None
        self.transition_timer = None

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.status_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(JoystickParameters, '/joystick_command', 20)
        self.obstacle_log_pub = self.create_publisher(ObstacleProximityLog, '/obstacle_log', 20)

    def joy_callback(self, msg):
        button6 = msg.buttons[6]
        button7 = msg.buttons[7]

        # Map buttons 0–3 to distance_zone (1–4)
        for i in range(4):
            if msg.buttons[i] == 1:
                self.distance_zone = i + 1

        if button6 == 1 and self.last_button6 == 0:
            self.override_enabled = False
            self.get_logger().info("🛑 Emergency stop")

        if button7 == 1 and self.last_button7 == 0:

            self.override_enabled = True
            self.get_logger().info("✅ Joystick override enabled")

        self.last_button6 = button6
        self.last_button7 = button7

    def status_callback(self, msg):
        new_goal_active = any(status.status == 2 for status in msg.status_list)

        if new_goal_active and not self.goal_active:
            self.get_logger().info("🟢 Navigation became active")
            self.turn_counter = 0
            self.distance_zone = 1
            self.start_transition_sequence("start")

        elif not new_goal_active and self.goal_active:
            self.get_logger().info("🔴 Navigation ended")
            self.start_transition_sequence("end")
            self.get_logger().info(f"Number of turns: {self.turn_counter}")


        self.goal_active = new_goal_active

    def start_transition_sequence(self, mode):
        self.cmd_vel_enabled = False
        self.get_logger().info(f"⏳ Starting transition: {mode}")

        # Prepare fixed message to publish
        self.transition_msg = JoystickParameters()
        self.transition_msg.turn_command = False
        self.transition_msg.turn_angle = 0.0
        self.transition_msg.position_direction = 0.0
        self.transition_msg.position_command = False
        self.transition_msg.enabled = True

        self.start_time = time.time()

        if self.transition_timer:
            self.transition_timer.cancel()

        # Start a single timer that handles both publishing + transition timing
        self.transition_timer = self.create_timer(0.05, self.transition_step)

    def transition_step(self):
        elapsed = time.time() - self.start_time

        self.publisher_.publish(self.transition_msg)

        if elapsed >= 2.0:
            self.cmd_vel_enabled = True
            self.transition_timer.cancel()
            self.transition_timer = None
            self.get_logger().info("✅ Transition complete — cmd_vel re-enabled")

    def cmd_callback(self, msg):
        if not self.cmd_vel_enabled:
            return

        out_msg = JoystickParameters()

        # Turn logic
        if msg.angular.z > 0.2:
            out_msg.turn_command = True
            out_msg.turn_angle = math.pi / 2
        elif msg.angular.z < -0.2:
            out_msg.turn_command = True
            out_msg.turn_angle = -math.pi / 2
        else:
            out_msg.turn_command = False
            out_msg.turn_angle = 0.0

        if out_msg.turn_command and not self.last_turn_command:
            self.turn_counter += 1
        self.last_turn_command = out_msg.turn_command

        # Motion logic
        self.alpha_b = 2.895 * msg.linear.x + 0.2531
        # self.alpha_b = msg.linear.x 
        out_msg.velocity = max(min(self.alpha_b , math.pi / 4), math.pi / 12)
        out_msg.position_direction = 0.0
        out_msg.position_command = True
        out_msg.enabled = self.override_enabled and self.goal_active

        self.publisher_.publish(out_msg)

        # Publish log message
        log_msg = ObstacleProximityLog()
        log_msg.distance_zone = self.distance_zone
        log_msg.goal_status = int(self.goal_active)
        log_msg.stamp = self.get_clock().now().to_msg()
        log_msg.turn_count = self.turn_counter
        self.obstacle_log_pub.publish(log_msg)

    def stop_robot(self):
        stop_msg = JoystickParameters()
        stop_msg.turn_command = False
        stop_msg.turn_angle = 0.0
        stop_msg.position_direction = 0.0
        stop_msg.velocity = 0.2618
        stop_msg.position_command = True
        stop_msg.enabled = False

        self.get_logger().info("🛑 Sending safety stop...")
        for _ in range(5):
            self.publisher_.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoystick()

    def shutdown_handler(signum, frame):
        node.get_logger().info(f"🔌 Caught signal {signum}. Sending safety stop...")
        node.stop_robot()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTSTP, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔌 KeyboardInterrupt — sending safety stop...")
        node.stop_robot()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
