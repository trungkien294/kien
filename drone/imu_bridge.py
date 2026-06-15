import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import math
import time
import threading

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.pub = self.create_publisher(Imu, '/imu/data', 50)
        
        # Mở Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 8890))
        # BỎ chế độ Non-blocking để dùng Thread siêu tốc
        
        self.yaw = 0.0
        self.last_time = time.time()
        
        self.fil_ax, self.fil_ay, self.fil_az = 0.0, 0.0, 9.81
        
        self.is_calibrating = True
        self.calib_samples = 0
        self.gz_bias = 0.0
        self.get_logger().info('⏳ ĐANG CALIBRATE IMU... ĐỂ YÊN DRONE TRONG 3 GIÂY!')
        
        # Kích hoạt luồng chạy ngầm siêu tốc độ (Không ăn CPU, không trễ)
        self.thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.thread.start()

    def udp_loop(self):
        while rclpy.ok():
            try:
                # Code sẽ "đứng đợi" ở đây. Hễ ESP32 bắn tín hiệu là nó chụp ngay lập tức (Zero Latency)
                data, addr = self.sock.recvfrom(1024) 
                text_data = data.decode('utf-8')
                vals = text_data.split(',')
                if len(vals) < 8: continue
                
                ax, ay, az = float(vals[0]), float(vals[1]), float(vals[2])
                gx, gy, gz = float(vals[3]), float(vals[4]), float(vals[5])
                roll_f, pitch_f = float(vals[6]), float(vals[7])
                
                # Giải mã dữ liệu
                accel_x, accel_y, accel_z = ax / 16384.0 * 9.81, ay / 16384.0 * 9.81, az / 16384.0 * 9.81
                gyro_x, gyro_y, gyro_z = gx / 131.0 * (math.pi / 180.0), gy / 131.0 * (math.pi / 180.0), gz / 131.0 * (math.pi / 180.0)
                
                if self.is_calibrating:
                    self.gz_bias += gyro_z
                    self.calib_samples += 1
                    if self.calib_samples >= 150: 
                        self.gz_bias /= 150.0
                        self.is_calibrating = False
                        self.last_time = time.time()
                        self.get_logger().info(f'✅ CALIBRATE XONG! Bias Gyro Z: {self.gz_bias:.5f} rad/s')
                    continue
                    
                gyro_z -= self.gz_bias 
                if abs(gyro_z) < (1.0 * math.pi / 180.0): gyro_z = 0.0
                
                # [QUAN TRỌNG NHẤT]: KHÔNG lọc Gyro Z. Truyền thẳng số liệu THÔ vào cho Cartographer "nắn" tia Lidar.
                alpha = 0.8
                self.fil_ax = self.fil_ax * (1 - alpha) + accel_x * alpha
                self.fil_ay = self.fil_ay * (1 - alpha) + accel_y * alpha
                self.fil_az = self.fil_az * (1 - alpha) + accel_z * alpha
                
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                self.yaw += gyro_z * dt 
                
                roll_est = roll_f * (math.pi / 180.0)
                pitch_est = pitch_f * (math.pi / 180.0)
                
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                
                qx, qy, qz, qw = euler_to_quaternion(roll_est, pitch_est, self.yaw)
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
                
                # Bắn vận tốc góc thô (Real-time) lên hệ thống
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro_x, gyro_y, gyro_z
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = self.fil_ax, self.fil_ay, self.fil_az
                
                self.pub.publish(msg)
            except Exception as e:
                pass

def main():
    rclpy.init()
    node = ImuBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
