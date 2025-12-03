import rclpy
from rclpy.node import Node
import serial
import time
import threading
from my_parameters.msg import Stm32Data
from my_parameters.msg import MotorParameters, JoystickParameters
import signal
import struct
from collections import deque

class PeakDetector:
    def __init__(self, min_peak_height=1):
        self.last_value = 0
        self.rising = False
        self.current_peak = 0
        self.min_peak_height = min_peak_height

    def update(self, value):
        """Returns peak value when a true peak is detected, otherwise None."""
        if value > self.last_value:
            self.rising = True
            self.current_peak = value

        elif self.rising and value < self.last_value:
            self.rising = False
            if self.current_peak >= self.min_peak_height:
                peak = self.current_peak
                self.current_peak = 0
                self.last_value = value
                return peak
            self.current_peak = 0

        self.last_value = value
        return None

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

        self.motor_commands = bytearray([0]*16)  # 12 motors

        #####Setting the motor moving parameters
        self.motors_idle1 = True
        self.motors_idle2 = True #Are ALL motors running or available
        self.pending_motor_commands = deque() #This is the queue of motor commands that gets sent when the motors are idling

        self.peak_detector = PeakDetector(min_peak_height=1)
        #self.peak_toggle = True   # Temporary variable

        self.relay_commands = bytearray([10, 10])  # Default to stopped
        self.enabled = False

        # Joystick data
        self.joystick_data = None  # Latest joystick message

        self.header = bytearray([83])
        self.terminators = bytearray([0, 55, 69])
        self.debug_pub = self.create_publisher(Stm32Data, '/stm_debug', 10)
        self.create_subscription(Stm32Data, '/StmData', self.stm_data_callback, 20)
        # Subscribe to joystick commands
        self.create_subscription(JoystickParameters, '/joystick_command', self.joystick_callback, 20)

        self.timer = self.create_timer(0.0005, self.timer_callback)



    def timer_callback(self):
        self.send_motor_commands()

    def float_to_byte(self, x):
        x = max(-127, min(127, int(x)))
        return x + 127
    
    def joystick_callback(self, msg):
        """Store latest joystick data."""
        self.joystick_data = msg


    def stm_data_callback(self, msg):

        target_value = msg.motor2_fr
        peak = self.peak_detector.update(target_value)

        if peak is not None:
            peak_byte = self.float_to_byte(peak)
            #value = 15 if self.peak_toggle else 0
            #peak_packet = bytearray([value] * 16)
            peak_packet = (bytearray([
            (int(msg.motor1_fr)), (int(msg.motor2_fr)), (int(msg.motor3_fr)),
            (int(msg.motor1_fl)), (int(msg.motor2_fl)), (int(msg.motor3_fl)),
            (int(msg.motor1_rr)), (int(msg.motor2_rr)), (int(msg.motor3_rr)),
            (int(msg.motor1_rl)), (int(msg.motor2_rl)), (int(msg.motor3_rl)), 
            self.float_to_byte(int(msg.step_fr)), self.float_to_byte((msg.step_fl)),
            self.float_to_byte(int(msg.step_rr)), self.float_to_byte((msg.step_rl))])) #bytearray([peak_byte] * 16)
            self.pending_motor_commands.append(peak_packet)
            self.get_logger().info(f"PEAK DETECTED: {peak} → queued")

        if abs(target_value) < 1:
            zero_packet = bytearray([0] * 16)
            self.pending_motor_commands.append(zero_packet)
            self.get_logger().info(f"ZERO detected → queued stop packet")

        if not self.enabled and msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.enabled = True

        elif self.enabled and not msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.motor_commands = bytearray([0]*16)
            self.releasing = True
            self.release_start_time = time.time()
            self.enabled = False


    def send_motor_commands(self):
        """Send motor commands to both serial devices and log sent/received bytes."""
        try:
            
            if self.releasing and (time.time() - self.release_start_time > 2.0):
                self.relay_commands = bytearray([10, 10])
                self.releasing = False

            self.get_logger().info(f"Motors idle: 1 - {self.motors_idle1}, 2 - {self.motors_idle2}")
            self.get_logger().info(f"Queue length before cleanup: {len(self.pending_motor_commands)}")

            if self.motors_idle1 and self.motors_idle2 and self.pending_motor_commands:
                # --- Clean up the queue: if next command is zero, jump to latest zero ---
                if all(b == 0 for b in self.pending_motor_commands[0]):
                    # find the last zero packet in the queue
                    last_zero_index = None
                    for i, packet in enumerate(self.pending_motor_commands):
                        if all(b == 0 for b in packet):
                            last_zero_index = i
                    if last_zero_index is not None:
                        # remove everything up to the last zero
                        for _ in range(last_zero_index):
                            self.pending_motor_commands.popleft()
                        self.get_logger().info(f"Jumping to most recent ZERO in queue, removed {last_zero_index} stale packets")

                self.motor_commands = self.pending_motor_commands.popleft()
                self.get_logger().info(f"Sending command: {[b for b in self.motor_commands]}")

            
            # Prepare data for STM1 (FR + RL DC motors + all servos)
            data1 = self.get_data_to_stm32(stm=1)
            # Prepare data for STM2 (FL + RR DC motors)
            data2 = self.get_data_to_stm32(stm=2)

            # --- Send to STM1 ---
            if data1:
                with self.lock1:
                    self.get_logger().info(f"→ Sending ({len(data1)} bytes) to STM1: {[b for b in data1]}")
                    self.get_logger().info(f"Expected encoder value: {self.motor_commands[3]*4*7*30}")
                    self.ser1.write(data1)
                    self.ser1.flush()
                    time.sleep(0.05)  # small delay

                    # Read 16-byte echo from STM1
                    response = bytearray()
                    timeout = time.time() + 0.5  # 0.5s timeout
                    while len(response) < 16 and time.time() < timeout:
                        response += self.ser1.read(16 - len(response))

                    ##New debugging unpacking
                    if len(response) == 16:
                        if response[0] == ord('P'):
                            try:
                                p1,p2,p3 = struct.unpack('<iii', response[1:13])
                                m1_moving = response[13]
                                m2_moving = response[14]
                                m3_moving = response[15]

                                self.motors_idle1 = not (m1_moving or m2_moving or m3_moving)
                                
                                self.get_logger().info(f"\n1 - position 1: {p1}\n1 - position 2: {p2}\n1 - position 3: {p3}"
                                                       f"\nM1_1 is moving: {m1_moving}"
                                                       f"\nM2_1 is moving: {m2_moving}"
                                                       f"\nM3_1 is moving: {m3_moving}")
                            except struct.error as e:
                                self.get_logger().warn(f"Failed to unpack positions: {e}")
                        else:
                            self.get_logger().warn(f"Unexpected header in response: {response[0]}, raw={list(response)}")
                    else:
                        self.get_logger().warn(f"Incomplete response from STM1 ({len(response)} bytes)")

            # --- Send to STM2 ---
            if data2:
                with self.lock2:
                    self.get_logger().info(f"→ Sending ({len(data2)} bytes) to STM2: {[b for b in data2]}")
                    self.get_logger().info(f"Expected encoder value: {self.motor_commands[3]*4*7*30}")
                    self.ser2.write(data2)
                    self.ser2.flush()
                    time.sleep(0.05)  # small delay

                    # Read 16-byte echo from STM1
                    response2 = bytearray()
                    timeout2 = time.time() + 0.5  # 0.5s timeout
                    while len(response2) < 16 and time.time() < timeout2:
                        response2 += self.ser2.read(16 - len(response2))

                    ##New debugging unpacking
                    if len(response2) == 16:
                        if response2[0] == ord('P'):
                            try:
                                p1_2,p2_2,p3_2 = struct.unpack('<iii', response2[1:13])
                                m1_moving_2 = response2[13]
                                m2_moving_2 = response2[14]
                                m3_moving_2 = response2[15]

                                self.motors_idle2 = not (m1_moving_2 or m2_moving_2 or m3_moving_2)
                                
                                self.get_logger().info(f"\n2 - position 1: {p1_2}\n2 - position 2: {p2_2}\n2 - position 3: {p3_2}"
                                                       f"\nM1_2 is moving: {m1_moving_2}"
                                                       f"\nM2_2 is moving: {m2_moving_2}"
                                                       f"\nM3_2 is moving: {m3_moving_2}")
                            except struct.error as e:
                                self.get_logger().warn(f"Failed to unpack positions: {e}")
                        else:
                            self.get_logger().warn(f"Unexpected header in response: {response2[0]}, raw={list(response2)}")
                    else:
                        self.get_logger().warn(f"Incomplete response from STM1 ({len(response2)} bytes)")

        except Exception as error:
            self.get_logger().error(f"Error in serial communication: {error}")


    def get_data_to_stm32(self, stm=1):

        reset_int = int(self.joystick_data.reset) if self.joystick_data else 0
        save_int = int(self.joystick_data.save) if self.joystick_data else 0
        
        if stm == 1:
            # STM1 -> FR + FL DC motors + all servos
            motors_list = [
                self.motor_commands[1],
                self.motor_commands[1],
                self.motor_commands[1],
            ] + list(self.motor_commands[12:16])

            motors = bytearray(motors_list)
            #motors = self.motor_commands[1] + self.motor_commands[1] + self.motor_commands[1] + self.motor_commands[12:16] #self.motor_commands[0:6] + self.motor_commands[12:16]
            data = self.header + bytes([reset_int]) + bytes([save_int]) + motors + self.relay_commands + self.terminators + bytearray([0])
        else:
            # STM2 -> RR + RL DC motors
            motors_list = [
                self.motor_commands[1],
                self.motor_commands[1],
                self.motor_commands[1],
            ] + list(self.motor_commands[12:16])
            motors = bytearray(motors_list)
            data = self.header + bytes([reset_int]) + bytes([save_int]) + motors + self.relay_commands + self.terminators + bytearray([0])
            #motors = self.motor_commands[6:9]
            #data = self.header + bytes([reset_int]) + bytes([save_int]) + motors + self.relay_commands + self.terminators + bytearray([0, 0, 0, 0, 0])

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
