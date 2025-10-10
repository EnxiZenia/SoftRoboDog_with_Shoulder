import math
import rclpy
from rclpy.node import Node  # <-- Add this import
from my_parameters.msg import MotorParameters
from my_parameters.msg import Stm32Data

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_subscriber')
        

        self.subscription = self.create_subscription(
            MotorParameters,  
            '/motor_command',
            self.function,
            20
        )
        self.publisher_ = self.create_publisher(Stm32Data, '/StmData', 20)

        self.motor1_fr = 0
        self.motor2_fr = 0
        self.motor3_fr = 0

        self.motor1_fl = 0
        self.motor2_fl = 0
        self.motor3_fl = 0

        self.motor1_rr = 0
        self.motor2_rr = 0
        self.motor3_rr = 0

        self.motor1_rl = 0
        self.motor2_rl = 0
        self.motor3_rl = 0

        self.pretight1_fr = 0
        self.pretight2_fr = 0
        self.pretight3_fr = 0

        self.pretight1_fl = 0
        self.pretight2_fl = 0
        self.pretight3_fl = 0

        self.pretight1_rr = 0
        self.pretight2_rr = 0
        self.pretight3_rr = 0

        self.pretight1_rl = 0
        self.pretight2_rl = 0
        self.pretight3_rl = 0

    def function(self, msg):

        if msg.move == True:
            self.motor1_fr = (msg.alpha_a_fr * 180/math.pi) + 90
            self.motor2_fr = (msg.alpha_b_fr * 180/math.pi) + 90
            self.motor3_fr = (msg.alpha_c_fr * 180/math.pi) + 90

            self.motor1_fl = (msg.alpha_a_fl * 180/math.pi) + 90
            self.motor2_fl = (msg.alpha_b_fl * 180/math.pi) + 90
            self.motor3_fl = (msg.alpha_c_fl * 180/math.pi) + 90

            self.motor1_rr = (msg.alpha_a_rr * 180/math.pi) + 90
            self.motor2_rr = (msg.alpha_b_rr * 180/math.pi) + 90
            self.motor3_rr = (msg.alpha_c_rr * 180/math.pi) + 90

            self.motor1_rl = (msg.alpha_a_rl * 180/math.pi) + 90
            self.motor2_rl = (msg.alpha_b_rl * 180/math.pi) + 90
            self.motor3_rl = (msg.alpha_c_rl * 180/math.pi) + 90
        else:
            self.motor1_fr =  90
            self.motor2_fr =  90
            self.motor3_fr =  90

            self.motor1_fl =  90
            self.motor2_fl =  90
            self.motor3_fl =  90

            self.motor1_rr =  90
            self.motor2_rr =  90
            self.motor3_rr =  90

            self.motor1_rl =  90
            self.motor2_rl =  90
            self.motor3_rl =  90   


        if msg.pre_tight == True:
            const = 10
        else:
            const = 0

        # self.pretight1_fr = 3 + const
        # self.pretight2_fr = 5 + const
        # self.pretight3_fr = -1 + const

        # self.pretight1_fl = 2 + const
        # self.pretight2_fl = 3 + const
        # self.pretight3_fl = 3 + const
        
        # self.pretight1_rr = 0 + const
        # self.pretight2_rr = -4 + const
        # self.pretight3_rr = 4 + const

        # self.pretight1_rl = -3 + const
        # self.pretight2_rl = 1 + const
        # self.pretight3_rl = 7 + const

        self.pretight1_fr = -2 + const
        self.pretight2_fr = -4 + const
        self.pretight3_fr = -1 + const

        self.pretight1_fl = -2 + const
        self.pretight2_fl = 0 + const
        self.pretight3_fl = 0 + const
        
        self.pretight1_rr = 6 + const
        self.pretight2_rr = -1 + const
        self.pretight3_rr = 13 + const

        self.pretight1_rl = 0 + const
        self.pretight2_rl = 3 + const
        self.pretight3_rl = 10 + const

        self.motor1_fr -=  self.pretight1_fr
        self.motor2_fr -=  self.pretight2_fr
        self.motor3_fr -=  self.pretight3_fr

        self.motor1_fl -=  self.pretight1_fl
        self.motor2_fl -=  self.pretight2_fl
        self.motor3_fl -=  self.pretight3_fl

        self.motor1_rr -=  self.pretight1_rr
        self.motor2_rr -=  self.pretight2_rr
        self.motor3_rr -=  self.pretight3_rr

        self.motor1_rl -=  self.pretight1_rl
        self.motor2_rl -=  self.pretight2_rl
        self.motor3_rl -=  self.pretight3_rl 
        
        msg_array = Stm32Data()
        msg_array.motor1_fr = math.floor(self.motor1_fr)
        msg_array.motor2_fr = math.floor(self.motor2_fr)
        msg_array.motor3_fr = math.floor(self.motor3_fr)

        msg_array.motor1_fl = math.floor(self.motor1_fl)
        msg_array.motor2_fl = math.floor(self.motor2_fl)
        msg_array.motor3_fl = math.floor(self.motor3_fl)

        msg_array.motor1_rr = math.floor(self.motor1_rr)
        msg_array.motor2_rr = math.floor(self.motor2_rr)
        msg_array.motor3_rr = math.floor(self.motor3_rr)

        msg_array.motor1_rl = math.floor(self.motor1_rl)
        msg_array.motor2_rl = math.floor(self.motor2_rl)
        msg_array.motor3_rl = math.floor(self.motor3_rl)
        msg_array.enabled = msg.pre_tight

        self.publisher_.publish(msg_array)

        # self.get_logger().info(f'FR Motors: {self.motor1_fr}, {self.motor2_fr}, {self.motor3_fr}')
        # self.get_logger().info(f'FL Motors: {self.motor1_fl}, {self.motor2_fl}, {self.motor3_fl}')
        # self.get_logger().info(f'RR Motors: {self.motor1_rr}, {self.motor2_rr}, {self.motor3_rr}')
        # self.get_logger().info(f'RL Motors: {self.motor1_rl}, {self.motor2_rl}, {self.motor3_rl}')





def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
