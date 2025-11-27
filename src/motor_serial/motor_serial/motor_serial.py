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

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # --- Initialize both serial ports ---
        try:
            self.ser1 = serial.Serial("/dev/ttyACM1", 115200, timeout=0.1)
            self.ser1.flushInput()
            self.ser1.flushOutput()

            #self.ser2 = serial.Serial("/dev/ttyACM1", 115200, timeout=0.1)
            #self.ser2.flushInput()
            #self.ser2.flushOutput()

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
        self.motors_idle = True #Are ALL motors running or available
        self.pending_motor_commands = deque() #This is the queue of motor commands that gets sent when the motors are idling

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
        """ Update motor commands """
        #####UPDATED THIS SO IT ADDS TO pending_motor_commands INSTEAD OF MOTOR COMMANDS#####
        self.pending_motor_commands.append(bytearray([
        self.float_to_byte(int(msg.motor1_fr)),(int(msg.motor2_fr)), self.float_to_byte((msg.motor3_fr)),
        self.float_to_byte(int(msg.motor1_fl)), self.float_to_byte((msg.motor2_fl)), self.float_to_byte((msg.motor3_fl)),
        self.float_to_byte(int(msg.motor1_rr)), self.float_to_byte((msg.motor2_rr)), self.float_to_byte((msg.motor3_rr)),
        self.float_to_byte(int(msg.motor1_rl)), self.float_to_byte((msg.motor2_rl)), self.float_to_byte((msg.motor3_rl)), 
        self.float_to_byte(int(msg.step_fr)), self.float_to_byte((msg.step_fl)),
        self.float_to_byte(int(msg.step_rr)), self.float_to_byte((msg.step_rl))
    ]))
        


        if not self.enabled and msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.enabled = True
        elif self.enabled and not msg.enabled:
            self.relay_commands = bytearray([200, 200])
            self.motor_commands = bytearray([127]*16)
            self.releasing = True
            self.release_start_time = time.time()
            self.enabled = False

    def send_motor_commands(self):
        """Send motor commands to both serial devices and log sent/received bytes."""
        try:
            # Handle relay release timing
            if self.releasing and (time.time() - self.release_start_time > 2.0):
                self.relay_commands = bytearray([10, 10])
                self.releasing = False


            self.get_logger().info(f"We not in this bitch, motors idle: {self.motors_idle}")
            self.get_logger().info(f"Current queue length: {len(self.pending_motor_commands)}")

            if self.motors_idle and self.pending_motor_commands:
                self.get_logger().info(f"Sike! We IN THIS BITCH, motors idle: {self.motors_idle}")
                self.motor_commands = self.pending_motor_commands.popleft() #Pop out the next command in the queue
            
            # Prepare data for STM1 (FR + RL DC motors + all servos)
            data1 = self.get_data_to_stm32(stm=1)
            # Prepare data for STM2 (FL + RR DC motors)
            #data2 = self.get_data_to_stm32(stm=2)

            # --- Send to STM1 ---
            if data1:
                with self.lock1:
                    self.get_logger().info(f"→ Sending ({len(data1)} bytes) to STM1: {[b for b in data1]}")
                    self.ser1.write(data1)
                    self.ser1.flush()
                    time.sleep(0.005)  # small delay

                    # Read 16-byte echo from STM1
                    response = bytearray()
                    timeout = time.time() + 0.5  # 0.5s timeout
                    while len(response) < 16 and time.time() < timeout:
                        response += self.ser1.read(16 - len(response))

                    # if len(response) == 16:
                    #    self.get_logger().info(f"← Echo ({len(response)} bytes) from STM1: {[b for b in response]}")
                    # else:
                    #    self.get_logger().warn(f"⚠️ Incomplete echo from STM1 ({len(response)} bytes)")

                    ##New debugging unpacking
                    if len(response) == 16:
                        if response[0] == ord('P'):
                            try:
                                p1,p2,p3 = struct.unpack('<iii', response[1:13])
                                # != 0 in the following is to make sure that we only say motors arent moving when its 0. If signal is noisy and sends something besides 0 or 1 its safer to assume the motor is moving
                                #m1_moving = response[13] != 0 #Is motor1 moving
                                #m2_moving = response[14] != 0 #Is motor2 moving
                                #m3_moving = response[15] != 0 #Is motor3 moving

                                m1_moving = response[13]
                                m2_moving = response[14]
                                m3_moving = response[15]

                                self.motors_idle = not (m1_moving or m2_moving)
                                
                                self.get_logger().info(f"\nPosition: {p1}\nPosition 2: {p2}\nPosition 3: {p3}"
                                                       f"\nM1 is moving: {m1_moving}"
                                                       f"\nM2 is moving: {m2_moving}")
                                                       #f"\nM3 is moving: {m3_moving}")
                            except struct.error as e:
                                self.get_logger().warn(f"Failed to unpack positions: {e}")
                        else:
                            self.get_logger().warn(f"Unexpected header in response: {response[0]}, raw={list(response)}")
                    else:
                        self.get_logger().warn(f"Incomplete response from STM1 ({len(response)} bytes)")

            # --- Send to STM2 ---
            #if data2:
            #    with self.lock2:
            #        self.get_logger().info(f"→ Sending ({len(data2)} bytes) to STM2: {[hex(b) for b in data2]}")
            #        self.ser2.write(data2)
            #        self.ser2.flush()
            #        time.sleep(0.05)

            #        # Read 16-byte echo from STM2
            #        response = bytearray()
            #        timeout = time.time() + 0.5
            #        while len(response) < 16 and time.time() < timeout:
            #            response += self.ser2.read(16 - len(response))

            #        if len(response) == 16:
            #            self.get_logger().info(f"← Echo ({len(response)} bytes) from STM2: {[hex(b) for b in response]}")
            #        else:
            #            self.get_logger().warn(f"⚠️ Incomplete echo from STM2 ({len(response)} bytes)")

        except Exception as error:
            self.get_logger().error(f"Error in serial communication: {error}")

    

    #def send_motor_commands(self):
    #    """ Send motor commands to both serial devices """
    #    try:
    #        if self.releasing and (time.time() - self.release_start_time > 2.0):
    #            self.relay_commands = bytearray([10, 10])
    #            self.releasing = False
#
    #        data1 = self.get_data_to_stm32(front=True)
    #        #data2 = self.get_data_to_stm32(front=False)
#
    #        if data1:
    #            #with self.lock1:
    #            #    self.ser1.write(data1)
    #            #    time.sleep(0.005)
    #            #    self.ser1.read(19)
#
    #            with self.lock1:
    #                self.ser1.write(data1)
    #                self.get_logger().info(f"→ Sent to STM32: {list(data1)}")
    #                time.sleep(0.5)
    #                response = self.ser1.readline().decode(errors='ignore').strip()
    #                if response:
    #                    self.get_logger().info(f"← Received from STM32: {response}")
#
    #        #if data2:
    #        #    with self.lock2:
    #        #        self.ser2.write(data2)
    #        #        time.sleep(0.005)
    #        #        self.ser2.read(19)
    #    except Exception as error:
    #        self.get_logger().error(f"Error in serial communication: {error}")

    def get_data_to_stm32(self, stm=1):
        if stm == 1:
            # STM1 -> FR + FL DC motors + all servos
            motors_list = [
                self.motor_commands[1],
                self.motor_commands[1],
                self.motor_commands[1],
            ] + list(self.motor_commands[12:16])

            motors = bytearray(motors_list)
            #motors = self.motor_commands[1] + self.motor_commands[1] + self.motor_commands[1] + self.motor_commands[12:16] #self.motor_commands[0:6] + self.motor_commands[12:16]
        else:
            # STM2 -> RR + RL DC motors
            motors = self.motor_commands[6:12]

        reset_int = int(self.joystick_data.reset) if self.joystick_data else 0
        save_int = int(self.joystick_data.save) if self.joystick_data else 0
        data = self.header + bytes([reset_int]) + bytes([save_int]) + motors + self.relay_commands + self.terminators

        #if len(data) != 16:  # Adjust length if needed
        #    return data  # or None if strict
        #return data
        # Make sure data has 16 bytes
        if len(data) < 16:
            data += bytearray([0] * (16 - len(data)))

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
