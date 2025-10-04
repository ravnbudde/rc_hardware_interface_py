import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

PORT = "/dev/ttyACM0"  # Endre til riktig port
BAUD = 1000000          # RRC Lite sender med 1Mbaud

crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

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
