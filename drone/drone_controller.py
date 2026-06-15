import socket
import sys
import tty
import termios
import select
import threading
import os
import rclpy
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# ============================================================
# CẤU HÌNH KẾT NỐI
# ============================================================
DRONE_IP = "192.168.4.1"
CMD_PORT = 8888

os.system(f"fuser -k -9 {CMD_PORT}/udp 2>/dev/null")
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

C_RESET = '\033[0m'
C_BOLD = '\033[1m'
C_GREEN = '\033[92m'
C_RED = '\033[91m'
C_YELLOW = '\033[93m'
C_CYAN = '\033[96m'


# ============================================================
# THÔNG SỐ TỰ HÀNH / RVIZ / APF
# ============================================================
TF_TIMEOUT = 0.7
SCAN_TIMEOUT = 0.7

GOAL_REACHED_DIST = 0.25

K_ATT = 0.45
MAX_VEL_NAV = 0.20
NAV_FORCE_GAIN = 9.0
NAV_MAX_ANGLE = 3.0

# ROS base_link: x trước, y trái.
# Drone command: roll dương thường là sang phải.
# Vì vậy mặc định dùng -1.0 để đổi ROS-y-left sang roll-right.
# Nếu RViz đi ngang ngược trái/phải thì đổi thành +1.0.
NAV_ROLL_SIGN = -1.0

# APF
K_REP = 1.2
SAFE_DIST = 1.0
APF_ACTIVE_FORCE = 0.35
APF_MAX_FORCE = 4.0

# Manual
MANUAL_ANGLE = 3.0
YAW_MANUAL_CMD = 18.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutoNavNode(Node):
    def __init__(self):
        super().__init__('drone_station')

        self.mode = 0
        self.flight_mode_status = 0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.slam_yaw = 0.0

        self.nav_goal_x = None
        self.nav_goal_y = None

        self.repulsive_x = 0.0
        self.repulsive_y = 0.0

        self.last_tf_time = 0.0
        self.last_tf_warn_time = 0.0
        self.last_scan_time = 0.0

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.get_tf_pose)

    def tf_ok(self):
        return (time.time() - self.last_tf_time) < TF_TIMEOUT

    def scan_ok(self):
        return (time.time() - self.last_scan_time) < SCAN_TIMEOUT

    def clear_nav(self):
        self.mode = 0
        self.nav_goal_x = None
        self.nav_goal_y = None
        self.repulsive_x = 0.0
        self.repulsive_y = 0.0

    def get_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            self.pos_x = t.transform.translation.x
            self.pos_y = t.transform.translation.y

            q = t.transform.rotation
            self.slam_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

            self.last_tf_time = time.time()

        except TransformException as e:
            now = time.time()
            if now - self.last_tf_warn_time > 1.5:
                self.last_tf_warn_time = now
                print(f"\n{C_RED}[TF FAIL] Không có TF map -> base_link. Kiểm tra SLAM/imu_bridge/static_tf.{C_RESET}")

    def goal_cb(self, msg):
        if self.flight_mode_status != 1:
            print(f"\n{C_RED}[CẢNH BÁO] Chưa bật giữ độ cao H, không nhận goal RViz.{C_RESET}")
            return

        if not self.tf_ok():
            print(f"\n{C_RED}[CẢNH BÁO] Chưa có TF map->base_link, không nhận goal RViz.{C_RESET}")
            return

        self.nav_goal_x = msg.pose.position.x
        self.nav_goal_y = msg.pose.position.y
        self.mode = 2

        print(
            f"\n{C_GREEN}[RVIZ NAV] Đã khóa đích: "
            f"X={self.nav_goal_x:.2f}, Y={self.nav_goal_y:.2f}.{C_RESET}"
        )

    def scan_cb(self, msg):
        rep_x = 0.0
        rep_y = 0.0

        for i, r in enumerate(msg.ranges):
            if 0.15 < r < SAFE_DIST and not math.isinf(r) and not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment

                # r càng gần thì lực đẩy càng mạnh
                force = K_REP * ((1.0 / r - 1.0 / SAFE_DIST) ** 2)

                # LaserScan thường theo base_link: x trước, y trái.
                # Vật ở trước angle=0 => rep_x âm, nghĩa là đẩy lùi.
                rep_x += -force * math.cos(angle)
                rep_y += -force * math.sin(angle)

        mag = math.sqrt(rep_x ** 2 + rep_y ** 2)
        if mag > APF_MAX_FORCE:
            rep_x = rep_x / mag * APF_MAX_FORCE
            rep_y = rep_y / mag * APF_MAX_FORCE

        self.repulsive_x = self.repulsive_x * 0.7 + rep_x * 0.3
        self.repulsive_y = self.repulsive_y * 0.7 + rep_y * 0.3
        self.last_scan_time = time.time()


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


def main():
    rclpy.init()

    nav_node = AutoNavNode()
    threading.Thread(target=rclpy.spin, args=(nav_node,), daemon=True).start()

    throttle = 1000
    arm = 0
    flight_mode = 0
    auto_land_flag = 0
    pos_hold_flag = 0

    saved_throttle = 1000

    # Khi bật H:
    # throttle gửi sang ESP32 không còn là ga thật nữa.
    # Nó là stick độ cao:
    # 1500 = giữ cao
    # 1600 = bay lên
    # 1450 = hạ xuống
    alt_stick = 1500
    ALT_STICK_CENTER = 1500
    ALT_STICK_UP = 1600
    ALT_STICK_DOWN = 1450

    target_yaw = 0.0
    int_yaw = 0.0
    is_yawing_prev = False

    last_time = time.time()

    print(f"{C_CYAN}====================================================={C_RESET}")
    print(f"{C_YELLOW}{C_BOLD} TRẠM ĐIỀU KHIỂN DRONE {C_RESET}")
    print(f"{C_CYAN}-----------------------------------------------------{C_RESET}")
    print(f" {C_BOLD}W/S, A/D{C_RESET} : Tiến/Lùi, Trái/Phải")
    print(f" {C_BOLD}J/L{C_RESET}      : Xoay trái/phải")
    print(f" {C_BOLD}I/K{C_RESET}      : MAN = tăng/giảm ga | ALT = leo/hạ")
    print(f" {C_BOLD}U/O{C_RESET}      : Tinh chỉnh ga nhỏ khi MAN")
    print(f" {C_BOLD}H{C_RESET}        : Bật/tắt GIỮ ĐỘ CAO")
    print(f" {C_BOLD}M{C_RESET}        : Thoát tự hành RViz")
    print(f" {C_BOLD}SPACE{C_RESET}    : ARM / DISARM")
    print(f" {C_YELLOW}{C_BOLD}C{C_RESET}        : AUTO LAND")
    print(f" {C_RED}{C_BOLD}X{C_RESET}        : RESET KHẨN")
    print(f" {C_RED}{C_BOLD}Q{C_RESET}        : Thoát")
    print(f"{C_CYAN}====================================================={C_RESET}\n")

    try:
        while True:
            now_loop = time.time()
            dt_loop = max(0.01, now_loop - last_time)
            last_time = now_loop

            key = get_key()

            tf_ok = nav_node.tf_ok()
            scan_ok = nav_node.scan_ok()

            # ============================================================
            # 1. PHÍM CHỨC NĂNG
            # ============================================================
            if key in ['m', 'M']:
                nav_node.clear_nav()
                pos_hold_flag = 1
                int_yaw = 0.0
                if tf_ok:
                    target_yaw = nav_node.slam_yaw
                print(f"\n{C_YELLOW}[MANUAL] Đã thoát tự hành RViz, quay về HOLD bằng MTF02.{C_RESET}")

            if key in ['c', 'C']:
                auto_land_flag = 1
                flight_mode = 1
                nav_node.flight_mode_status = 1
                nav_node.clear_nav()
                throttle = ALT_STICK_CENTER
                print(f"\n{C_YELLOW}[LAND] Bắt đầu AUTO LAND, hủy RViz/autonav.{C_RESET}")

            if key in ['h', 'H']:
                flight_mode = 1 if flight_mode == 0 else 0
                nav_node.flight_mode_status = flight_mode

                if flight_mode == 1:
                    saved_throttle = throttle

                    # Gửi ga hover thật trước để ESP32 khóa hover_throttle.
                    sock_cmd.sendto(
                        f"{int(throttle)},0,0,0,{arm},{flight_mode},{auto_land_flag},1".encode(),
                        (DRONE_IP, CMD_PORT)
                    )

                    for _ in range(5):
                        sock_cmd.sendto(
                            f"{int(saved_throttle)},0,0,0,{arm},{flight_mode},{auto_land_flag},1".encode(),
                            (DRONE_IP, CMD_PORT)
                        )
                        time.sleep(0.02)

                    alt_stick = ALT_STICK_CENTER
                    throttle = alt_stick
                    print(f"\n{C_GREEN}[ALT] Bật H, khóa hover throttle = {saved_throttle}.{C_RESET}")

                else:
                    throttle = saved_throttle
                    nav_node.clear_nav()
                    print(f"\n{C_YELLOW}[MANUAL] Tắt H, hủy RViz/autonav.{C_RESET}")

            if key == ' ':
                arm = 1 if arm == 0 else 0
                auto_land_flag = 0

                if arm == 0:
                    target_yaw = nav_node.slam_yaw if tf_ok else 0.0
                    int_yaw = 0.0
                    flight_mode = 0
                    nav_node.flight_mode_status = 0
                    nav_node.clear_nav()
                    throttle = 1000
                    alt_stick = ALT_STICK_CENTER
                    print(f"\n{C_YELLOW}[DISARM] Đã disarm, reset mode.{C_RESET}")
                else:
                    target_yaw = nav_node.slam_yaw if tf_ok else 0.0
                    int_yaw = 0.0
                    print(f"\n{C_GREEN}[ARM] Đã arm.{C_RESET}")

            if key in ['x', 'X']:
                throttle = 1000
                arm = 0
                flight_mode = 0
                auto_land_flag = 0
                pos_hold_flag = 0
                nav_node.flight_mode_status = 0
                nav_node.clear_nav()
                alt_stick = ALT_STICK_CENTER
                int_yaw = 0.0
                target_yaw = nav_node.slam_yaw if tf_ok else 0.0
                print(f"\n{C_RED}[RESET] Reset khẩn: ga thấp, disarm, tắt mode.{C_RESET}")

            if key in ['q', 'Q']:
                break

            # ============================================================
            # 2. ĐIỀU KHIỂN GA / ĐỘ CAO
            # ============================================================
            if auto_land_flag == 0:

                if flight_mode == 0:
                    # MANUAL: i/k/u/o là tăng giảm ga thật
                    if key == 'i':
                        throttle = min(2000, throttle + 10)
                    elif key == 'k':
                        throttle = max(1000, throttle - 10)
                    elif key == 'u':
                        throttle = min(2000, throttle + 2)
                    elif key == 'o':
                        throttle = max(1000, throttle - 2)

                else:
                    # ALT HOLD: i/k là stick leo/hạ, thả phím = giữ cao
                    if key == 'i':
                        alt_stick = ALT_STICK_UP
                    elif key == 'k':
                        alt_stick = ALT_STICK_DOWN
                    else:
                        alt_stick = ALT_STICK_CENTER

                    throttle = alt_stick

            else:
                # Auto-land do ESP32 xử lý theo MTF02.
                throttle = ALT_STICK_CENTER

            # ============================================================
            # 3. KIỂM TRA LÁI TAY / RVIZ / APF
            # ============================================================
            is_moving = key in ['w', 's', 'a', 'd']

            pitch_cmd = MANUAL_ANGLE if key == 'w' else -MANUAL_ANGLE if key == 's' else 0.0
            roll_cmd = -MANUAL_ANGLE if key == 'a' else MANUAL_ANGLE if key == 'd' else 0.0

            apf_mag = math.sqrt(nav_node.repulsive_x ** 2 + nav_node.repulsive_y ** 2)

            is_dodging = (
                nav_node.mode == 2
                and flight_mode == 1
                and scan_ok
                and apf_mag > APF_ACTIVE_FORCE
            )

            is_navigating = False
            dist_to_goal = 0.0

            if nav_node.mode == 2 and nav_node.nav_goal_x is not None:
                if flight_mode != 1:
                    nav_node.clear_nav()
                    print(f"\n{C_RED}[AUTO FAIL] Chưa bật giữ độ cao H, hủy tự hành.{C_RESET}")

                elif not tf_ok:
                    nav_node.clear_nav()
                    print(f"\n{C_RED}[AUTO FAIL] Không có TF map->base_link, hủy tự hành.{C_RESET}")

                else:
                    dist_to_goal = math.sqrt(
                        (nav_node.nav_goal_x - nav_node.pos_x) ** 2
                        + (nav_node.nav_goal_y - nav_node.pos_y) ** 2
                    )

                    if dist_to_goal > GOAL_REACHED_DIST:
                        is_navigating = True
                    else:
                        nav_node.clear_nav()
                        print(f"\n{C_GREEN}[RVIZ NAV] Đã tới đích, quay về HOLD MTF02.{C_RESET}")

            # ============================================================
            # 4. POS HOLD FLAG
            # ============================================================
            # 0: ESP32 không giữ XY bằng MTF02 vì đang có lệnh điều khiển / RViz / né vật cản
            # 1: ESP32 giữ XY bằng MTF02
            if auto_land_flag == 1:
                pos_hold_flag = 0
            else:
                pos_hold_flag = 0 if (is_moving or is_navigating or is_dodging) else 1

            send_pitch = 0.0
            send_roll = 0.0

            # ============================================================
            # 5. TÍNH LỆNH ROLL / PITCH
            # ============================================================
            if is_moving:
                # Lái tay theo hệ thân drone, không lấy XY từ Lidar.
                send_pitch = pitch_cmd
                send_roll = roll_cmd

                send_pitch = clamp(send_pitch, -4.0, 4.0)
                send_roll = clamp(send_roll, -4.0, 4.0)

            elif is_navigating:
                # Tự hành RViz dùng pose map->base_link để đi tới goal.
                # Không dùng XY Lidar để HOLD. HOLD vẫn là MTF02 khi pos_hold_flag=1.
                err_x = nav_node.nav_goal_x - nav_node.pos_x
                err_y = nav_node.nav_goal_y - nav_node.pos_y

                yaw_rad = nav_node.slam_yaw

                # Lực hút về goal
                att_x = K_ATT * err_x
                att_y = K_ATT * err_y

                # Đổi lực đẩy APF từ hệ thân base_link sang hệ map
                rep_gx = (
                    nav_node.repulsive_x * math.cos(yaw_rad)
                    - nav_node.repulsive_y * math.sin(yaw_rad)
                )

                rep_gy = (
                    nav_node.repulsive_x * math.sin(yaw_rad)
                    + nav_node.repulsive_y * math.cos(yaw_rad)
                )

                target_vel_x = att_x + rep_gx
                target_vel_y = att_y + rep_gy

                mag = math.sqrt(target_vel_x ** 2 + target_vel_y ** 2)

                if mag > MAX_VEL_NAV:
                    target_vel_x = target_vel_x / mag * MAX_VEL_NAV
                    target_vel_y = target_vel_y / mag * MAX_VEL_NAV

                force_x_global = target_vel_x * NAV_FORCE_GAIN
                force_y_global = target_vel_y * NAV_FORCE_GAIN

                # Đổi lực từ map về hệ thân base_link
                force_x_local = (
                    force_x_global * math.cos(yaw_rad)
                    + force_y_global * math.sin(yaw_rad)
                )

                force_y_local = (
                    -force_x_global * math.sin(yaw_rad)
                    + force_y_global * math.cos(yaw_rad)
                )

                send_pitch = clamp(force_x_local, -NAV_MAX_ANGLE, NAV_MAX_ANGLE)

                # ROS y-local dương là trái, roll dương là phải nên mặc định nhân -1.
                send_roll = clamp(NAV_ROLL_SIGN * force_y_local, -NAV_MAX_ANGLE, NAV_MAX_ANGLE)

            elif is_dodging:
                # Trường hợp chỉ có APF nhưng không còn goal: né nhẹ rồi về HOLD.
                send_pitch = clamp(nav_node.repulsive_x * 2.0, -2.0, 2.0)
                send_roll = clamp(NAV_ROLL_SIGN * nav_node.repulsive_y * 2.0, -2.0, 2.0)

            # ============================================================
            # 6. XỬ LÝ YAW
            # ============================================================
            is_yawing = False

            # Nếu có TF/SLAM thì dùng yaw hold theo slam_yaw.
            # Nếu không có TF thì yaw manual-only để tránh tự quay phải.
            if key == 'j':
                is_yawing = True
                if tf_ok:
                    target_yaw += 0.8 * dt_loop

            elif key == 'l':
                is_yawing = True
                if tf_ok:
                    target_yaw -= 0.8 * dt_loop

            target_yaw = wrap_pi(target_yaw)

            if arm == 0:
                target_yaw = nav_node.slam_yaw if tf_ok else 0.0
                yaw = 0.0
                int_yaw = 0.0
                is_yawing_prev = False

            else:
                if not tf_ok:
                    # Không có SLAM/TF: không giữ yaw tự động, chỉ yaw khi bấm J/L.
                    if key == 'j':
                        yaw = YAW_MANUAL_CMD
                    elif key == 'l':
                        yaw = -YAW_MANUAL_CMD
                    else:
                        yaw = 0.0

                    int_yaw = 0.0
                    is_yawing_prev = False

                else:
                    # Có SLAM/TF: dùng yaw hold theo slam_yaw.
                    if not is_yawing and is_yawing_prev:
                        target_yaw = nav_node.slam_yaw

                    is_yawing_prev = is_yawing

                    err_yaw = wrap_pi(target_yaw - nav_node.slam_yaw)

                    if not is_yawing:
                        int_yaw = clamp(int_yaw + err_yaw * dt_loop, -10.0, 10.0)

                    if abs(err_yaw) < 0.035 and not is_yawing:
                        yaw_cmd = int_yaw * 1.2
                    else:
                        yaw_cmd = err_yaw * 10.0 + int_yaw * 1.2

                    yaw = clamp(yaw_cmd, -20.0, 20.0)

            # ============================================================
            # 7. GỬI UDP SANG ESP32
            # Format:
            # throttle,roll,pitch,yaw,arm,flight_mode,auto_land_flag,pos_hold_flag
            # ============================================================
            msg = (
                f"{int(throttle)},"
                f"{send_roll:.3f},"
                f"{send_pitch:.3f},"
                f"{yaw:.3f},"
                f"{arm},"
                f"{flight_mode},"
                f"{auto_land_flag},"
                f"{pos_hold_flag}"
            )

            sock_cmd.sendto(msg.encode(), (DRONE_IP, CMD_PORT))

            # ============================================================
            # 8. HIỂN THỊ TERMINAL
            # ============================================================
            arm_str = f"{C_RED}ARM{C_RESET}" if arm else f"{C_GREEN}DISARM{C_RESET}"
            alt_str = f"{C_GREEN}[ALT]{C_RESET}" if flight_mode == 1 else f"{C_CYAN}[MAN]{C_RESET}"
            warn_str = f" {C_RED}⚠ NÉ!{C_RESET}" if is_dodging else ""

            tf_str = f"{C_GREEN}TF OK{C_RESET}" if tf_ok else f"{C_RED}NO TF{C_RESET}"
            scan_str = f"{C_GREEN}SCAN OK{C_RESET}" if scan_ok else f"{C_RED}NO SCAN{C_RESET}"

            if auto_land_flag == 1:
                mode_str = f"{C_YELLOW}[LAND]{C_RESET}"
                extra_info = f" | {tf_str} {scan_str}"

            elif is_navigating:
                mode_str = f"{C_YELLOW}[RVIZ]{C_RESET}"
                extra_info = (
                    f" | D:{dist_to_goal:.2f}m"
                    f" X:{nav_node.pos_x:.2f} Y:{nav_node.pos_y:.2f}"
                    f" R:{send_roll:.2f} P:{send_pitch:.2f}"
                    f" APF:{apf_mag:.2f}"
                    f" | {tf_str} {scan_str}"
                )

            elif is_moving or is_dodging:
                mode_str = f"{C_CYAN}[LÁI]{C_RESET}"
                extra_info = (
                    f" | R:{send_roll:4.1f} P:{send_pitch:4.1f}"
                    f" APF:{apf_mag:.2f}"
                    f" | {tf_str} {scan_str}"
                )

            else:
                mode_str = f"{C_GREEN}[HOLD]{C_RESET}"
                extra_info = f" | {tf_str} {scan_str}"

            if flight_mode == 1:
                if throttle > 1505:
                    alt_cmd_str = "LEO"
                elif throttle < 1495:
                    alt_cmd_str = "HA"
                else:
                    alt_cmd_str = "GIU"

                status = (
                    f"{mode_str} | {alt_str} - {arm_str}"
                    f" | AltStick:{int(throttle)} [{alt_cmd_str}]"
                    f" | PH:{pos_hold_flag}"
                    f" | YAW:{yaw:.2f}"
                    f"{warn_str}{extra_info}"
                )

            else:
                status = (
                    f"{mode_str} | {alt_str} - {arm_str}"
                    f" | Ga PWM:{int(throttle)}"
                    f" | PH:{pos_hold_flag}"
                    f" | YAW:{yaw:.2f}"
                    f"{warn_str}{extra_info}"
                )

            sys.stdout.write(f"\033[2K\r{status}")
            sys.stdout.flush()

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        sock_cmd.sendto(b"1000,0,0,0,0,0,0,0", (DRONE_IP, CMD_PORT))

        try:
            rclpy.shutdown()
        except Exception:
            pass

        print(f"\n{C_GREEN}Đã dừng hệ thống an toàn.{C_RESET}")


if __name__ == '__main__':
    main()
