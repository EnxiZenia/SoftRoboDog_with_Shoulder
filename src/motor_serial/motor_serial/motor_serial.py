import rclpy
from rclpy.node import Node
import serial
import time
import threading
from my_parameters.msg import Stm32Data
from my_parameters.msg import MotorParameters
import signal

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # --- Initialize both serial ports ---
        try:
            self.ser1 = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
            self.ser1.flushInput()
            self.ser1.flushOutput()

            self.ser2 = serial.Serial("/dev/ttyACM1", 115200, timeout=0.1)
            self.ser2.flushInput()
            self.ser2.flushOutput()

            time.sleep(1)
            self.get_logger().info("✅ Serial connected to STM32 on both ports!")
        except Exception as error:
            self.get_logger().error(f"❌ Error initializing serial: {error}")
            return
        
        self.lock1 = threading.Lock()
        self.lock2 = threading.Lock()
        self.releasing = False
        self.release_start_time = None

        self.motor_commands = bytearray([90]*16)  # 12 motors

        self.relay_commands = bytearray([10, 10])  # Default to stopped
        self.enabled = False

        self.header = bytearray([83])
        self.terminators = bytearray([0, 55, 69])
        self.debug_pub = self.create_publisher(Stm32Data, '/stm_debug', 10)
        self.create_subscription(Stm32Data, '/StmData', self.stm_data_callback, 20)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        self.send_motor_commands()

    def stm_data_callback(self, msg):
        """ Update motor commands """
        self.motor_commands = bytearray([
            msg.motor1_fr, msg.motor2_fr, msg.motor3_fr,
            msg.motor1_fl, msg.motor2_fl, msg.motor3_fl,
            msg.motor1_rr, msg.motor2_rr, msg.motor3_rr,
            msg.motor1_rl, msg.motor2_rl, msg.motor3_rl, 
            msg.step_fr, msg.step_fl,
            msg.step_rr, msg.step_rl
        ])

        if not self.enabled and msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.enabled = True
        elif self.enabled and not msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.motor_commands = bytearray([90]*16)
            self.releasing = True
            self.release_start_time = time.time()
            self.enabled = False

    def send_motor_commands(self):
        """ Send motor commands to both serial devices """
        try:
            if self.releasing and (time.time() - self.release_start_time > 2.0):
                self.relay_commands = bytearray([10, 10])
                self.releasing = False

            data1 = self.get_data_to_stm32(front=True)
            data2 = self.get_data_to_stm32(front=False)

            if data1:
                with self.lock1:
                    self.ser1.write(data1)
                    time.sleep(0.005)
                    self.ser1.read(19)
            if data2:
                with self.lock2:
                    self.ser2.write(data2)
                    time.sleep(0.005)
                    self.ser2.read(19)
        except Exception as error:
            self.get_logger().error(f"Error in serial communication: {error}")

    def get_data_to_stm32(self, front=True):
        """ Combine commands for each serial """
        if front:
            motors = self.motor_commands[0:6]  # fr + fl
        else:
            motors = self.motor_commands[6:16]  # rr + rl

        data = self.header + motors + self.relay_commands + self.terminators
        if len(data) != 16:  # Adjust length if needed
            return data  # or None if strict
        return data

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = MotorSerialNode()

    def shutdown_handler(signum, frame):
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTSTP, shutdown_handler)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
