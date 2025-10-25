import math
import rclpy
from rclpy.node import Node

from my_parameters.msg import JoystickParameters, ShoulderCommand, GaitClock


class ShoulderGait(Node):
    """
    Generates four shoulder joint angles phase-locked to a shared gait clock.
    Subscribes:
      - /joystick_command (JoystickParameters): provides enable flag and a speed proxy.
      - /gait_clock (GaitClock): provides phase/omega/T for strict synchronization.
    Publishes:
      - /shoulder_cmd (ShoulderCommand): 4 angles in radians, order [FL, FR, RL, RR].
    """

    def __init__(self):
        super().__init__('shoulder_gait')

        # Tunable parameters (consider external YAML later)
        self.declare_parameter('phi0', 0.0)                 # neutral offset [rad]
        self.declare_parameter('phi_max_base', 0.35)        # base amplitude [rad]
        self.declare_parameter('v_to_phi_k', 0.0)           # optional amplitude gain vs velocity
        self.declare_parameter('phase_shift_T_ratio', 0.12) # 0.10 ~ 0.15 of one period
        self.declare_parameter('publish_rate', 100.0)       # Hz

        self.phi0 = float(self.get_parameter('phi0').value)
        self.phi_max_base = float(self.get_parameter('phi_max_base').value)
        self.k_phi = float(self.get_parameter('v_to_phi_k').value)
        self.ratio = float(self.get_parameter('phase_shift_T_ratio').value)
        self.rate = float(self.get_parameter('publish_rate').value)

        # Trot: left-right opposite, diagonal in-phase
        self.sign = {'FL': +1.0, 'FR': -1.0, 'RL': +1.0, 'RR': -1.0}
        self.psi  = {'FL': 0.0,  'FR': math.pi, 'RL': math.pi, 'RR': 0.0}

        # State
        self.enabled = False
        self.last_js: JoystickParameters | None = None
        self.clock: GaitClock | None = None

        # I/O
        self.create_subscription(JoystickParameters, '/joystick_command', self.cb_js, 10)
        self.create_subscription(GaitClock, '/gait_clock', self.cb_clock, 10)
        self.pub = self.create_publisher(ShoulderCommand, '/shoulder_cmd', 20)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)

        self.get_logger().info('ShoulderGait started.')

    def cb_js(self, msg: JoystickParameters):
        self.last_js = msg
        self.enabled = bool(msg.enabled)

    def cb_clock(self, msg: GaitClock):
        self.clock = msg

    def on_timer(self):
        # If disabled or missing inputs, hold neutral
        if not self.enabled or self.last_js is None or self.clock is None or not self.clock.enabled:
            self._publish([self.phi0, self.phi0, self.phi0, self.phi0], enabled=False)
            return

        phase = float(self.clock.phase)
        # Optional amplitude scaling using velocity proxy
        v = float(self.last_js.velocity)
        phi_max = max(0.0, self.phi_max_base + self.k_phi * v)

        theta_shift = 2.0 * math.pi * self.ratio

        names = ['FL', 'FR', 'RL', 'RR']
        phi = [
            self.phi0 + self.sign[n] * phi_max * math.sin(phase + self.psi[n] + theta_shift)
            for n in names
        ]
        self._publish(phi, enabled=True)

    def _publish(self, phi_list, enabled: bool):
        msg = ShoulderCommand()
        msg.phi = [float(x) for x in phi_list]
        msg.enabled = bool(enabled)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShoulderGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()