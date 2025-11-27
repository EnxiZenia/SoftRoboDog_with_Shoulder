import math
import numpy as np
import rclpy
from rclpy.node import Node
from my_parameters.msg import JoystickParameters  # Correct import based on your topic info
from my_parameters.msg import MotorParameters
from my_parameters.msg import DebugParameter

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_subscriber')
        
        # Initialize the start time
        self.time_start = self.get_clock().now()


        # Publisher for motor command
        self.publisher_ = self.create_publisher(MotorParameters, "/motor_command", 20)
        # self.debug_publisher_ = self.create_publisher(DebugParameter, "debug_motor_parameters", 10)
        # Subscription to joystick command
        self.subscription = self.create_subscription(
            JoystickParameters,  
            '/joystick_command',
            self.callback_function, 
            20
        )

        # Initial references and states
        self.alpha_r_ref = 0
        # self.alpha_b_ref = 0.6247
        self.alpha_b_ref = 0.0
        self.z_l_ref = 0.0076
        self.period = 0.8
        self.move = False
        self.pre_tight = False

        # Initialize all variables to 0.0
        self.alpha_r_fr = 0.0
        self.alpha_b_fr = 0.0
        self.z_l_fr = 0.0

        self.alpha_r_fl = 0.0
        self.alpha_b_fl = 0.0
        self.z_l_fl = 0.0

        self.alpha_r_rr = 0.0
        self.alpha_b_rr = 0.0
        self.z_l_rr = 0.0

        self.alpha_r_rl = 0.0
        self.alpha_b_rl = 0.0
        self.z_l_rl = 0.0

    def callback_function(self, msg):
        # Update reference values based on joystick input
        self.alpha_r_ref = msg.turn_angle if msg.turn_command else msg.position_direction

        # Call the second function to calculate angles
        if msg.turn_command:
            self.alpha_r_rr = self.alpha_r_ref + math.pi
            self.alpha_r_rl = self.alpha_r_ref + math.pi
        else:
            self.alpha_r_rr = self.alpha_r_ref
            self.alpha_r_rl = self.alpha_r_ref

        self.alpha_r_fr = self.alpha_r_ref
        self.alpha_r_fl = self.alpha_r_ref

        if msg.enabled == True:
            if msg.turn_command == 1 or msg.position_command == 1:
                self.move = True
            else:
                self.move = False
        else:
            self.move = False

        self.pre_tight = msg.enabled
        self.alpha_b_ref = msg.velocity
        self.update_motor_command()

    def calculate_alpha_abc(self, alpha_r, alpha_b, z_l):
        """ Mathematical calculation for alpha_A, alpha_B, and alpha_C """
        # L_disk = 0.02944
        # R_spool = 0.021
        # R_w = L_disk / math.sqrt(3)

        # alpha_A = -(-R_w * alpha_b * math.cos(-alpha_r) + z_l) / R_spool
        # alpha_B = -(-R_w * alpha_b * math.cos(-alpha_r + (2 * math.pi / 3)) + z_l) / R_spool
        # alpha_C = -(-R_w * alpha_b * math.cos(-alpha_r + (4 * math.pi / 3)) + z_l) / R_spool

        L_disk = 0.02944       # Disk side length (m)
        R_spool = 0.021
        R_w = L_disk / np.sqrt(3)
        original_len = 0.15    # Original twisted wire length (m)
        r = 0.00053 / 2

        # ---- A ----
        length_A = -R_w * alpha_b * np.cos(-alpha_r) + z_l
        final_len_A = original_len + length_A

        if length_A < 0:
            alpha_A = np.sqrt(original_len**2 - final_len_A**2) / r
        else:
            alpha_A = 0

        # ---- B ----
        length_B = -R_w * alpha_b * np.cos(-alpha_r + 2*np.pi/3) + z_l
        final_len_B = original_len + length_B

        if length_B < 0:
            alpha_B = np.sqrt(original_len**2 - final_len_B**2) / r
        else:
            alpha_B = 0

        # ---- C ----
        length_C = -R_w * alpha_b * np.cos(-alpha_r + 4*np.pi/3) + z_l
        final_len_C = original_len + length_C

        if length_C < 0:
            alpha_C = np.sqrt(original_len**2 - final_len_C**2) / r
        else:
            alpha_C = 0
        

        return float(alpha_A), float(alpha_B), float(alpha_C), float(alpha_b)

    def update_motor_command(self):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_duration = current_time - self.time_start
        elapsed_time = elapsed_duration.nanoseconds / 1e9  # Convert nanoseconds to seconds
        t_mod = elapsed_time % 0.8

        # self.get_logger().info(f't_mod: {t_mod}')
        # --- Function 1 (Sinusoidal) ---
        # func1_value = self.alpha_b_ref * math.sin(2 * math.pi / self.period * elapsed_time)

        # # --- Function 2 (Negative Sinusoidal) ---
        # func2_value = -self.alpha_b_ref * math.sin(2 * math.pi / self.period * elapsed_time)



        # --- Function 1 (Linear Wave) ---
        if 0 <= t_mod < 0.2:
            func1_value = t_mod / 0.2
        elif 0.2 <= t_mod < 0.6:
            func1_value = -5*t_mod + 2
        elif 0.6 <= t_mod < 0.8:
            func1_value = 5 * t_mod - 4
        else:
            func1_value = 0

        # --- Function 2 (Inverse Linear Wave) ---
        if 0 <= t_mod < 0.2:
            func2_value = -t_mod / 0.2
        elif 0.2 <= t_mod < 0.6:
            func2_value = 5 * t_mod - 2
        elif 0.6 <= t_mod < 0.8:
            func2_value = -5*t_mod + 4
        else:
            func2_value = 0
        

        # --- Function 3 (Custom Wave) ---
        if 0 <= t_mod < 0.2:
            func3_value = 1 - (t_mod / 0.2)
        elif 0.2 <= t_mod < 0.6:
            func3_value = 0
        elif 0.6 <= t_mod < 0.8:
            func3_value = (t_mod - 0.6) / 0.2
        else:
            func3_value = 0  

        # --- Function 4 (New Custom Wave) ---
        if 0 <= t_mod < 0.2:
            func4_value = 0
        elif 0.2 <= t_mod < 0.4:
            func4_value = (t_mod - 0.2) / 0.2
        elif 0.4 <= t_mod < 0.6:
            func4_value = 1 - (t_mod - 0.4) / 0.2
        elif 0.6 <= t_mod < 0.8:
            func4_value = 0
        else:
            func4_value = 0

        # Assign calculated values to variables
        self.alpha_b_fr = func1_value * self.alpha_b_ref
        self.alpha_b_rl = func1_value * self.alpha_b_ref
        self.alpha_b_fl = func2_value * self.alpha_b_ref
        self.alpha_b_rr = func2_value * self.alpha_b_ref
        self.z_l_fl = func3_value * self.z_l_ref
        self.z_l_rr = func3_value * self.z_l_ref
        self.z_l_fr = func4_value * self.z_l_ref
        self.z_l_rl = func4_value * self.z_l_ref


        # --- Calculate and Publish for all wheels ---
        motor_command = MotorParameters()
        motor_command.alpha_a_fr, motor_command.alpha_b_fr, motor_command.alpha_c_fr, motor_command.step_fr = self.calculate_alpha_abc(self.alpha_r_fr, self.alpha_b_fr, self.z_l_fr)
        motor_command.alpha_a_fl, motor_command.alpha_b_fl, motor_command.alpha_c_fl, motor_command.step_fl = self.calculate_alpha_abc(self.alpha_r_fl, self.alpha_b_fl, self.z_l_fl)
        motor_command.alpha_a_rr, motor_command.alpha_b_rr, motor_command.alpha_c_rr, motor_command.step_rr = self.calculate_alpha_abc(self.alpha_r_rr, self.alpha_b_rr, self.z_l_rr)
        motor_command.alpha_a_rl, motor_command.alpha_b_rl, motor_command.alpha_c_rl, motor_command.step_rl = self.calculate_alpha_abc(self.alpha_r_rl, self.alpha_b_rl, self.z_l_rl)
        motor_command.move = self.move
        motor_command.pre_tight = self.pre_tight
        self.publisher_.publish(motor_command)


        # debug_msg = DebugParameter()
        # debug_msg.alpha_r_fr = self.alpha_r_fr
        # debug_msg.alpha_b_fr = self.alpha_b_fr
        # debug_msg.z_l_fr = self.z_l_fr
        # debug_msg.alpha_r_fl = self.alpha_r_fl
        # debug_msg.alpha_b_fl = self.alpha_b_fl
        # debug_msg.z_l_fl = self.z_l_fl
        # debug_msg.alpha_r_rr = self.alpha_r_rr
        # debug_msg.alpha_b_rr = self.alpha_b_rr
        # debug_msg.z_l_rr = self.z_l_rr
        # debug_msg.alpha_r_rl = self.alpha_r_rl
        # debug_msg.alpha_b_rl = self.alpha_b_rl
        # debug_msg.z_l_rl = self.z_l_rl

        # self.debug_publisher_.publish(debug_msg)

        # self.get_logger().info(f'alpha a fr: {motor_command.alpha_a_fr}')
        # self.get_logger().info(f'alpha b fr: {motor_command.alpha_b_fr}')
        # self.get_logger().info(f'alpha c fr: {motor_command.alpha_c_fr}')

        # self.get_logger().info(f'alpha a fl: {motor_command.alpha_a_fl}')
        # self.get_logger().info(f'alpha b fl: {motor_command.alpha_b_fl}')
        # self.get_logger().info(f'alpha c fl: {motor_command.alpha_c_fl}')

        # self.get_logger().info(f'alpha a rr: {motor_command.alpha_a_rr}')
        # self.get_logger().info(f'alpha b rr: {motor_command.alpha_b_rr}')
        # self.get_logger().info(f'alpha c rr: {motor_command.alpha_c_rr}')

        # self.get_logger().info(f'alpha a rl: {motor_command.alpha_a_rl}')
        # self.get_logger().info(f'alpha b rl: {motor_command.alpha_b_rl}')
        # self.get_logger().info(f'alpha c rl: {motor_command.alpha_c_rl}')



def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
