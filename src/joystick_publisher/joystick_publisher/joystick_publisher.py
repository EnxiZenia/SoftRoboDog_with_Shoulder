import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from my_parameters.msg import JoystickParameters

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            20
        )

        # Robot parameters
        self.enabled = False
        self.velocity = 0.5
        self.velocity_min = 0.2
        self.velocity_max = 1.0
        self.velocity_step = 0.05
        self.reset = 0
        self.save = 0

        self.publisher_ = self.create_publisher(JoystickParameters,
                                                '/joystick_command',
                                                20)

        # Logitech F710 button mapping:
        # BLUE = X
        # GREEN = A
        # RED = B
        # YELLOW = Y
        self.BLUE = 0     # X = enable
        self.GREEN = 1    # A = decrease
        self.YELLOW = 3   # Y = increas
        self.RED = 2      # B = stop
        self.LB = 4 
        self.RB = 5 

        # Left joystick axes
        self.AXIS_X = 0
        self.AXIS_Y = 1

    def joy_callback(self, msg):
        # --- ENABLE / DISABLE ---
        if msg.buttons[self.BLUE] == 1:
            self.enabled = True

        if msg.buttons[self.RED] == 1:   # Red = stop
            self.enabled = False

        # --- SPEED CONTROL ---
        if msg.buttons[self.YELLOW] == 1:  # Yellow = increase speed
            self.velocity += self.velocity_step

        if msg.buttons[self.GREEN] == 1:     # Green = decrease speed
            self.velocity -= self.velocity_step

        # Clamp velocity
        self.velocity = max(self.velocity_min,
                            min(self.velocity, self.velocity_max))
        

        # --- DIRECTION FROM LEFT JOYSTICK ---
        x = msg.axes[self.AXIS_X]
        y = msg.axes[self.AXIS_Y]

        if abs(x) < 0.2 and abs(y) < 0.2:
            direction = 0.0
            move = False
        else:
            direction = math.atan2(-x, y)
            move = True

        # --- PUBLISH ---
        out = JoystickParameters()
        out.enabled = self.enabled
        out.velocity = float(self.velocity)
        out.position_command = move
        out.position_direction = float(direction)
        out.turn_command = False
        out.turn_angle = 0.0
        out.reset = bool(msg.buttons[self.LB])
        out.save = bool(msg.buttons[self.RB])

        self.publisher_.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
