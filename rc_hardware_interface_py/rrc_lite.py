"""
rc_hardware_interface_py.rrc_node

Denne modulen håndterer kommunikasjon med RRC Lite-controlleren over USB-serial.
Alt som leses fra kontrolleren publiseres på ROS2-kanaler, og alt som skrives
til kontrolleren kommer fra ROS2-kanaler.

Publiserte/abonnerte ROS2-kanaler:
- to be announced
"""

# pylint: disable=import-error
import struct
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from rc_hardware_interface_py.utils.crc8 import checksum_crc8

PORT = "/dev/ttyACM0"  # Endre til riktig port
BAUD = 1000000          # RRC Lite sender med 1Mbaud

def read_frame(ser: serial.Serial):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0xAA:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != 0x55:
            continue

        func = ser.read(1)
        length = ser.read(1)
        if not func or not length:
            return None
        func_code = func[0]
        data_len = length[0]

        params = ser.read(data_len)
        if len(params) != data_len:
            return None

        chksum = ser.read(1)
        if not chksum:
            return None

        if checksum_crc8(func + length + params) != chksum[0]:
            print("Feil på checksum!")
            return None

        return {"function": func_code, "length": data_len, "params": params, "checksum": chksum[0]}

def decode_function7(params: bytes):
    if len(params) != 24:
        return None
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack('<6f', params)
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def decode_function0(params: bytes):
    if params[0] == 0x04:
        return struct.unpack('<H', params[1:])[0]
    return None

class USBNode(Node):
    def __init__(self):
        super().__init__('usb_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery', 10)
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            self.get_logger().info(f"Lytter på {PORT} @ {BAUD} baud...")
        except Exception as e:
            self.get_logger().error(f"Kunne ikke åpne {PORT}: {e}")
            self.ser = None
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.ser:
            return
        frame = read_frame(self.ser)
        if not frame:
            return

        if frame['function'] == 0x07:
            imu_data = decode_function7(frame['params'])
            if imu_data:
                msg = Imu()
                msg.linear_acceleration.x = imu_data[0]
                msg.linear_acceleration.y = imu_data[1]
                msg.linear_acceleration.z = imu_data[2]
                msg.angular_velocity.x = imu_data[3]
                msg.angular_velocity.y = imu_data[4]
                msg.angular_velocity.z = imu_data[5]
                self.imu_pub.publish(msg)

        elif frame['function'] == 0x00:
            battery = decode_function0(frame['params'])
            if battery:
                msg = Float32()
                msg.data = battery / 1000.0  # eksempel konvertering til Volt
                self.get_logger().info(f"Batterispenning: {msg.data} V")
                self.battery_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = USBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
