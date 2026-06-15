/*
  main.cpp - ESP32 flight controller
  Giả định / lưu ý:
  - UART MTF02: RX=GPIO17, TX=GPIO15, baud 115200, protocol MSPv2 .
  - Cảm biến IMU tại địa chỉ 0x68, đọc thanh ghi kiểu MPU6050/ICM 
  - Accelerometer scale: mặc định ±2g => 16384 LSB/g. // tránh sai scale gây drift góc
  - Gyro scale: dùng hệ số 0.00763358 deg/s/LSB như code gốc (tương đương ±250 dps). // giữ tương thích với tune cũ
  - Thuật toán giữ vị trí/độ cao được cập nhật theo hướng của báo cáo optical-flow: bù gyro cho flow, bù ToF theo tilt, cascade position->velocity->acceleration->angle và altitude->climb-rate->throttle.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// ================= HARDWARE =================
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ICM_ADDR 0x68

#define M1_PIN 4
#define M2_PIN 5
#define M3_PIN 6
#define M4_PIN 7

#define LIDAR_RX 18
#define LIDAR_MOT 16

#define MTF02_RX_PIN 17
#define MTF02_TX_PIN 15

HardwareSerial SerialMTF(1);

const char *ssid = "F450_Drone_Kien";
const char *password = "88888888";
const char *laptop_ip = "192.168.4.2";

WiFiUDP udp_lidar, udp_cmd, udp_imu;
WebServer server(80);
Adafruit_BMP280 *bmp;

// ================= FLIGHT STATE =================
float throttle = 1000.0f;
float r_set = 0.0f;
float p_set = 0.0f;
float y_set = 0.0f;

int is_armed_int = 0;
int flight_mode = 0;
int auto_land_flag = 0;
int pos_hold_flag = 0;

unsigned long timer = 0;
unsigned long last_debug_time = 0;
unsigned long last_cmd_time = 0;

// ================= MTF02 RAW =================
float mtf_distance = 0.0f;  // khoảng cách raw từ MTF02
float mtf_vel_x = 0.0f;     // vận tốc X sau lọc
float mtf_vel_y = 0.0f;     // vận tốc Y sau lọc
uint8_t mtf_quality = 0;

float mtf_vel_x_fil = 0.0f;
float mtf_vel_y_fil = 0.0f;

float FLOW_LPF = 0.55f;          // LPF nhanh cho flow raw
float FLOW_SPIKE_LIMIT = 1.20f;  // bỏ spike flow bất thường

const int FLOW_MA_SIZE = 8;      // moving-average 8 mẫu cho flow
float flow_ma_x[FLOW_MA_SIZE] = {0};
float flow_ma_y[FLOW_MA_SIZE] = {0};
float flow_ma_sum_x = 0.0f;
float flow_ma_sum_y = 0.0f;
int flow_ma_idx = 0;

// ================= IMU + ATTITUDE PID =================
float roll_f = 0.0f;
float pitch_f = 0.0f;
float yaw_est = 0.0f;  // yaw tương đối từ gyro để đổi BF<->RF khi giữ vị trí

float gyro_x_cal = 0.0f;
float gyro_y_cal = 0.0f;
float gyro_z_cal = 0.0f;

float acc_roll_cal = 0.0f;
float acc_pitch_cal = 0.0f;

float rate_r_fil = 0.0f;
float rate_p_fil = 0.0f;
float rate_y_fil = 0.0f;

float acc_r_fil = 0.0f;
float acc_p_fil = 0.0f;

// giữ lại tên biến cũ để ít ảnh hưởng phần khác
float kalman_roll = 0.0f;
float kalman_pitch = 0.0f;
float P_roll = 1.0f;
float P_pitch = 1.0f;
float Q_angle = 0.0f;
float R_angle = 0.0f;

float Kp_angle_rp = 4.5f;

float Kp_rate_rp = 0.60f;
float Ki_rate_rp = 0.030f;
float Kd_rate_rp = 0.055f;

float r_i = 0.0f;
float p_i = 0.0f;

float r_prev_rate_e = 0.0f;
float p_prev_rate_e = 0.0f;

float Kp_rate_yaw = 4.5f;
float Ki_rate_yaw = 0.02f;
float Kd_rate_yaw = 0.0f;

float y_i = 0.0f;
float y_prev_rate_e = 0.0f;

float D_ALPHA = 0.08f;
float r_d_fil = 0.0f;
float p_d_fil = 0.0f;
float y_d_fil = 0.0f;

float GYRO_LPF_ALPHA = 0.30f;
float ACC_LPF_ALPHA = 0.18f;

float ACC_NORM_MIN = 0.65f;
float ACC_NORM_MAX = 1.45f;
float ACC_LSB_PER_G = 16384.0f;   // giả định mặc định ±2g
float ATT_GYRO_WEIGHT = 0.985f;   // complementary filter: tin gyro ngắn hạn
float GYRO_DEADBAND_DPS = 0.12f;  // cắt trôi gyro nhỏ khi đứng yên

// ================= KF / NAV STATE =================
const float GRAVITY_MS2 = 9.80665f;

struct KF1D {
  float pos = 0.0f;
  float vel = 0.0f;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

KF1D kf_x;
KF1D kf_y;
KF1D kf_z;

struct NavState {
  float x = 0.0f;   // vị trí RF
  float y = 0.0f;   // vị trí RF
  float z = 0.0f;   // độ cao đã bù tilt

  float vx = 0.0f;  // vận tốc RF
  float vy = 0.0f;  // vận tốc RF
  float vz = 0.0f;  // vận tốc lên/xuống

  float target_x = 0.0f;
  float target_y = 0.0f;
  float target_z = 0.0f;

  float ref_yaw = 0.0f;  // yaw mốc khi bắt đầu bộ ước lượng XY

  bool range_ok = false;
  bool flow_ok = false;
  bool airborne = false;

  bool xy_hold_ready = false;
  bool z_hold_ready = false;

  unsigned long airborne_start_ms = 0;
  unsigned long flow_good_start_ms = 0;
};

NavState nav;

// ================= FLIGHT GATES =================
float TAKEOFF_THR_MIN = 1250.0f;     // ga đủ lớn mới xét đang bay
float LAND_THR_MAX = 1120.0f;        // ga thấp coi như landed

float AIRBORNE_MIN_Z = 0.20f;        // dưới mức này không cho hold
float AIRBORNE_MAX_Z = 2.20f;

uint8_t FLOW_MIN_QUALITY = 35;

uint16_t AIRBORNE_CONFIRM_MS = 400;  // xác nhận airborne có trễ để tránh false trigger
uint16_t FLOW_GOOD_CONFIRM_MS = 250; // xác nhận quality đủ tốt liên tục

// ================= OPTICAL FLOW COMPENSATION + KF PARAMS =================
float FLOW_GYRO_ROLL_SIGN = 1.0f;    // đổi dấu nếu khi nghiêng roll mà bù sai chiều
float FLOW_GYRO_PITCH_SIGN = 1.0f;   // đổi dấu nếu khi nghiêng pitch mà bù sai chiều
float FLOW_COMP_GAIN = 0.35f;        // gain bù gyro cho flow, tune dần từ 0 -> 1  0.4
float FLOW_BIAS_ALPHA = 0.004f;      // học bias flow khi chưa bay
float FLOW_POS_LEAK = 0.0005f;       // xả trôi chậm cho vị trí XY

float FLOW_VEL_DEADBAND = 0.025f;    // bỏ nhiễu flow nhỏ
float FLOW_R_MIN = 0.015f;           // quality cao -> R nhỏ
float FLOW_R_MAX = 0.120f;           // quality thấp -> R lớn

float RANGE_R_MIN = 0.006f;          // ToF quality cao -> R nhỏ
float RANGE_R_MAX = 0.060f;          // ToF quality thấp -> R lớn

float KF_XY_Q_POS = 0.00002f;
float KF_XY_Q_VEL = 0.00150f;
float KF_Z_Q_POS  = 0.00002f;
float KF_Z_Q_VEL  = 0.00200f;

float flow_bias_vx = 0.0f;
float flow_bias_vy = 0.0f;

// ================= POSITION HOLD XY =================
// Vòng ngoài vị trí -> vận tốc đặt, vòng trong vận tốc -> gia tốc -> góc
float XY_POS_DEADBAND = 0.03f;     // bỏ nhiễu vị trí 6cm
float XY_MAX_POS = 2.0f;           // giới hạn tích vị trí

float XY_POS_P_SMALL = 0.95f;      // gain phi tuyến: lỗi nhỏ   0.55
float XY_POS_P_MID   = 1.25f;      // gain phi tuyến: lỗi vừa   0.85
float XY_POS_P_BIG   = 1.45f;      // gain phi tuyến: lỗi lớn   1.15

float XY_MAX_VEL = 0.35f;          // giới hạn vận tốc đặt
float XY_VEL_SP_LPF_HZ = 12.0f;    // LPF setpoint vận tốc

float XY_VEL_P = 0.9f;            // vận tốc lỗi -> gia tốc
float XY_VEL_I = 0.01f;            // để 0 trước tránh tự bò góc do flow bias
float XY_I_LIMIT = 0.18f;          // gioi han tich phan , don vi gan m/s2
float XY_VEL_D = 0.06f;
float XY_D_FILTER_HZ = 15.0f;      // Lọc D để đỡ nhiễu

float XY_MAX_ACCEL = 0.5f;        // tăng nhẹ để pitch có lực hơn
float XY_MAX_ANGLE = 2.0f;         // giữ lại, dùng như giá trị tham khảo

float XY_ROLL_HOLD_GAIN  = 1.5f;   // trái/phải đang ổn, giữ nguyên
float XY_PITCH_HOLD_GAIN = 6.5f;   // tăng riêng trục tiến/lùi

float XY_MAX_ROLL_ANGLE  = 4.0f;   // roll giữ như cũ
float XY_MAX_PITCH_ANGLE = 5.0f;   // pitch cho mạnh hơn

int XY_POS_LOOP_DIV = 5;           // outer loop chạy chậm hơn inner loop

float xy_target_vx_fil = 0.0f;
float xy_target_vy_fil = 0.0f;
float xy_i_x = 0.0f;
float xy_i_y = 0.0f;
float xy_last_vel_err_x = 0.0f;
float xy_last_vel_err_y = 0.0f;
float xy_d_x_fil = 0.0f;
float xy_d_y_fil = 0.0f;
int xy_outer_loop_counter = 0;

// ================= ALT HOLD Z =================
// Outer loop: altitude -> climb rate, inner loop: climb rate -> throttle correction
float ALT_DEADBAND = 0.04f;

float ALT_POS_P_SMALL = 0.75f;
float ALT_POS_P_MID   = 1.05f;
float ALT_POS_P_BIG   = 1.35f;

float ALT_MAX_CLIMB_RATE = 0.22f;
float ALT_TARGET_VZ_LPF_HZ = 10.0f;

float ALT_VEL_P = 170.0f;
float ALT_VEL_I = 1.0f;
float ALT_VEL_D = 10.0f;
float ALT_D_FILTER_HZ = 12.0f;

float ALT_MAX_CORRECTION = 170.0f;
float ALT_OUTPUT_SLEW = 150.0f;   //giam toc do thay doi ga

float alt_target_vz_fil = 0.0f;
float alt_i = 0.0f;
float alt_last_vz_err = 0.0f;
float alt_d_fil = 0.0f;
float alt_output_limited = 0.0f;

// ================= BMP280 BACKUP =================
float ground_pressure = 0.0f;
uint8_t barometer_counter = 0;
float P = 0.0f;

int32_t pressure_rotating_mem[20] = {0};
uint8_t pressure_rotating_mem_location = 0;
int32_t pressure_total_avarage = 0;

float actual_pressure_fast = 0.0f;
float actual_pressure_slow = 0.0f;
float actual_pressure_diff = 0.0f;
float actual_pressure = 0.0f;

uint8_t parachute_rotating_mem_location = 0;
int32_t parachute_buffer[30] = {0};
int32_t parachute_throttle = 0;

float pressure_parachute_previous = 0.0f;

uint8_t manual_altitude_change = 1;

float pid_altitude_setpoint = 0.0f;
float pid_altitude_input = 0.0f;
float pid_error_temp = 0.0f;
float pid_error_gain_altitude = 0.0f;

float pid_i_mem_altitude = 0.0f;
float pid_output_altitude = 0.0f;
float manual_throttle = 0.0f;
float hover_throttle = 1500.0f;

float pid_p_gain_altitude = 1.5f;
float pid_i_gain_altitude = 0.2f;
float pid_d_gain_altitude = 0.75f;

int pid_max_altitude = 180;

float current_manual_throttle = 1000.0f;
bool recovering_from_althold = false;

// giữ lại tên cũ để tránh sửa lan sang phần khác
float mtf_alt_setpoint = 0.0f;
float mtf_alt_i = 0.0f;
float last_mtf_distance = 0.0f;
float mtf_alt_filtered = 0.0f;
float mtf_vz_filtered = 0.0f;
float mtf_alt_target = 0.0f;
bool mtf_alt_hold_was_off = true;
float alt_vz_i = 0.0f;
float last_vz_error = 0.0f;
float Kp_alt_pos = 0.0f;
float Kp_alt_vz = 0.0f;
float Ki_alt_vz = 0.0f;
float Kd_alt_vz = 0.0f;
float MAX_CLIMB_RATE = 0.0f;
float MAX_ALT_CORRECTION = 0.0f;
float ALT_OUTPUT_SLEW_OLD = 0.0f;
float pid_output_altitude_limited = 0.0f;

// ================= RAW IMU GLOBAL =================
int16_t imu_ax = 0;
int16_t imu_ay = 0;
int16_t imu_az = 0;
int16_t imu_gx = 0;
int16_t imu_gy = 0;
int16_t imu_gz = 0;

// ================= HELPERS =================
float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

float lowPassHz(float input, float prev, float cut_off_hz, float dt) {
  if (cut_off_hz <= 0.0f || dt <= 0.0f) return input;
  float rc = 1.0f / (cut_off_hz * 2.0f * PI);
  float alpha = dt / (rc + dt);
  return prev + alpha * (input - prev);
}

// ================= SAFE IMU READ =================
bool readICMRaw(
  int16_t &ax,
  int16_t &ay,
  int16_t &az,
  int16_t &gx,
  int16_t &gy,
  int16_t &gz
) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x3B);

  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;

  int n = Wire.requestFrom((uint16_t)ICM_ADDR, (size_t)14, true);
  if (n != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

// ================= KF HELPERS =================
void kfReset(KF1D &kf, float pos0, float vel0) {
  kf.pos = pos0;
  kf.vel = vel0;

  kf.P00 = 0.05f;
  kf.P01 = 0.0f;
  kf.P10 = 0.0f;
  kf.P11 = 0.10f;
}

void kfPredict(KF1D &kf, float dt, float q_pos, float q_vel) {
  kf.pos += kf.vel * dt;  // predict x = x + v*dt

  float P00 = kf.P00 + dt * (kf.P10 + kf.P01) + dt * dt * kf.P11 + q_pos;
  float P01 = kf.P01 + dt * kf.P11;
  float P10 = kf.P10 + dt * kf.P11;
  float P11 = kf.P11 + q_vel;

  kf.P00 = P00;
  kf.P01 = P01;
  kf.P10 = P10;
  kf.P11 = P11;
}

void kfUpdateVel(KF1D &kf, float vel_meas, float R) {
  float P00 = kf.P00;
  float P01 = kf.P01;
  float P10 = kf.P10;
  float P11 = kf.P11;

  float y = vel_meas - kf.vel;  // innovation của vận tốc
  float S = P11 + R;
  if (S < 1.0e-6f) return;

  float K0 = P01 / S;
  float K1 = P11 / S;

  kf.pos += K0 * y;
  kf.vel += K1 * y;

  kf.P00 = P00 - K0 * P10;
  kf.P01 = P01 - K0 * P11;
  kf.P10 = P10 - K1 * P10;
  kf.P11 = P11 - K1 * P11;
}

void kfUpdatePos(KF1D &kf, float pos_meas, float R) {
  float P00 = kf.P00;
  float P01 = kf.P01;
  float P10 = kf.P10;
  float P11 = kf.P11;

  float y = pos_meas - kf.pos;  // innovation của vị trí
  float S = P00 + R;
  if (S < 1.0e-6f) return;

  float K0 = P00 / S;
  float K1 = P10 / S;

  kf.pos += K0 * y;
  kf.vel += K1 * y;

  kf.P00 = P00 - K0 * P00;
  kf.P01 = P01 - K0 * P01;
  kf.P10 = P10 - K1 * P00;
  kf.P11 = P11 - K1 * P01;
}

float qualityToRFlow(uint8_t q) {
  float t = constrain((float)q / 100.0f, 0.0f, 1.0f);
  return FLOW_R_MAX - t * (FLOW_R_MAX - FLOW_R_MIN);  // quality cao -> R nhỏ hơn
}

float qualityToRRange(uint8_t q) {
  float t = constrain((float)q / 100.0f, 0.0f, 1.0f);
  return RANGE_R_MAX - t * (RANGE_R_MAX - RANGE_R_MIN);  // quality cao -> R nhỏ hơn
}

// ================= OPTICAL FLOW / RANGE HELPERS =================
float getTiltCompensatedHeight() {
  float cr = cosf(roll_f * DEG_TO_RAD);
  float cp = cosf(pitch_f * DEG_TO_RAD);
  float z = mtf_distance * cr * cp;   // theo báo cáo: bù độ cao bằng cos(phi)*cos(theta)
  return constrain(z, 0.0f, 3.0f);
}

// ================= MTF02 MSP V2 READ =================
int32_t readI32LE(uint8_t *p) {
  uint32_t v =
    ((uint32_t)p[0]) |
    ((uint32_t)p[1] << 8) |
    ((uint32_t)p[2] << 16) |
    ((uint32_t)p[3] << 24);

  return (int32_t)v;
}

void pushFlowMovingAverage(float vx_in, float vy_in) {
  flow_ma_sum_x -= flow_ma_x[flow_ma_idx];
  flow_ma_sum_y -= flow_ma_y[flow_ma_idx];

  flow_ma_x[flow_ma_idx] = vx_in;
  flow_ma_y[flow_ma_idx] = vy_in;

  flow_ma_sum_x += flow_ma_x[flow_ma_idx];
  flow_ma_sum_y += flow_ma_y[flow_ma_idx];

  flow_ma_idx++;
  if (flow_ma_idx >= FLOW_MA_SIZE) flow_ma_idx = 0;

  float vx_ma = flow_ma_sum_x / (float)FLOW_MA_SIZE;
  float vy_ma = flow_ma_sum_y / (float)FLOW_MA_SIZE;

  mtf_vel_x = vx_ma;
  mtf_vel_y = vy_ma;
}

void readMTF02() {
  static uint8_t state = 0;
  static uint8_t payload[16];
  static uint8_t payload_idx = 0;

  static uint16_t msg_id = 0;
  static uint16_t payload_size = 0;

  while (SerialMTF.available()) {
    uint8_t c = SerialMTF.read();

    switch (state) {
      case 0:
        if (c == 0x24) state = 1;
        break;

      case 1:
        if (c == 0x58) state = 2;
        else state = 0;
        break;

      case 2:
        if (c == 0x3C) state = 3;
        else state = 0;
        break;

      case 3:
        state = 4;
        break;

      case 4:
        msg_id = c;
        state = 5;
        break;

      case 5:
        msg_id |= (c << 8);
        state = 6;
        break;

      case 6:
        payload_size = c;
        state = 7;
        break;

      case 7:
        if (payload_size > 0 && payload_size <= 16) {
          payload_idx = 0;
          state = 8;
        } else {
          state = 0;
        }
        break;

      case 8:
        payload[payload_idx++] = c;
        if (payload_idx >= payload_size) state = 9;
        break;

      case 9:
        if (msg_id == 0x1F01 && payload_size == 5) {
          mtf_quality = payload[0];

          int32_t dist_mm = readI32LE(&payload[1]);
          mtf_distance = (float)dist_mm / 1000.0f;
        }
        else if (msg_id == 0x1F02 && payload_size == 9) {
          mtf_quality = payload[0];

          int32_t flow_x = readI32LE(&payload[1]);
          int32_t flow_y = readI32LE(&payload[5]);

          // Theo đo thực tế của bạn: tiến trước -> X âm, sang phải -> Y âm
          float vx_new = -(float)flow_x * 0.01f;  // vx > 0 = trôi về trước
          float vy_new = -(float)flow_y * 0.01f;  // vy > 0 = trôi sang phải

          // Chặn spike trước khi đưa vào moving-average
          if (fabsf(vx_new - mtf_vel_x_fil) < FLOW_SPIKE_LIMIT) {
            mtf_vel_x_fil = mtf_vel_x_fil * (1.0f - FLOW_LPF) + vx_new * FLOW_LPF;
          }
          if (fabsf(vy_new - mtf_vel_y_fil) < FLOW_SPIKE_LIMIT) {
            mtf_vel_y_fil = mtf_vel_y_fil * (1.0f - FLOW_LPF) + vy_new * FLOW_LPF;
          }

          pushFlowMovingAverage(mtf_vel_x_fil, mtf_vel_y_fil);  // theo báo cáo: moving-average 8 mẫu
        }

        state = 0;
        break;
    }
  }
}

// ================= NAV ESTIMATOR =================
void resetXYEstimator() {
  kfReset(kf_x, 0.0f, 0.0f);
  kfReset(kf_y, 0.0f, 0.0f);

  nav.x = 0.0f;
  nav.y = 0.0f;
  nav.vx = 0.0f;
  nav.vy = 0.0f;

  nav.target_x = 0.0f;
  nav.target_y = 0.0f;

  nav.ref_yaw = yaw_est;   // yaw hiện tại thành mốc mới cho RF tương đối
  nav.xy_hold_ready = false;

  xy_target_vx_fil = 0.0f;
  xy_target_vy_fil = 0.0f;
  xy_i_x = 0.0f;
  xy_i_y = 0.0f;
  xy_last_vel_err_x = 0.0f;
  xy_last_vel_err_y = 0.0f;
  xy_d_x_fil = 0.0f;
  xy_d_y_fil = 0.0f;
  xy_outer_loop_counter = 0;
}

void resetZEstimator() {
  kfReset(kf_z, 0.0f, 0.0f);

  nav.z = 0.0f;
  nav.vz = 0.0f;
  nav.target_z = 0.0f;

  nav.z_hold_ready = false;

  alt_target_vz_fil = 0.0f;
  alt_i = 0.0f;
  alt_last_vz_err = 0.0f;
  alt_d_fil = 0.0f;
  alt_output_limited = 0.0f;
}

void updateNavEstimator(float dt) {
  unsigned long now = millis();

  float z_tilt = getTiltCompensatedHeight();

  nav.range_ok = (
    z_tilt > AIRBORNE_MIN_Z &&
    z_tilt < AIRBORNE_MAX_Z &&
    mtf_quality >= FLOW_MIN_QUALITY
  );

  bool throttle_ok_for_airborne = (throttle > TAKEOFF_THR_MIN);

  // Chưa ARM hoặc ga thấp -> không cho estimator XY/Z công kích điều khiển
  if (is_armed_int == 0 || throttle < LAND_THR_MAX) {
    nav.airborne = false;
    nav.flow_ok = false;
    nav.airborne_start_ms = 0;
    nav.flow_good_start_ms = 0;

    resetXYEstimator();
    resetZEstimator();
    return;
  }

  // Học bias flow khi chưa bay thật để giảm drift đứng yên
  if (!nav.airborne && mtf_quality >= FLOW_MIN_QUALITY) {
    flow_bias_vx = flow_bias_vx * (1.0f - FLOW_BIAS_ALPHA) + mtf_vel_x * FLOW_BIAS_ALPHA;
    flow_bias_vy = flow_bias_vy * (1.0f - FLOW_BIAS_ALPHA) + mtf_vel_y * FLOW_BIAS_ALPHA;
  }

  // Flight gate: xác nhận airborne
  if (is_armed_int == 1 && throttle_ok_for_airborne && nav.range_ok) {
    if (nav.airborne_start_ms == 0) nav.airborne_start_ms = now;
    if (now - nav.airborne_start_ms >= AIRBORNE_CONFIRM_MS) {
      nav.airborne = true;
    }
  } else {
    nav.airborne_start_ms = 0;
  }

  // Flight gate: xác nhận flow tốt liên tục
  if (nav.airborne && nav.range_ok) {
    if (nav.flow_good_start_ms == 0) nav.flow_good_start_ms = now;
    if (now - nav.flow_good_start_ms >= FLOW_GOOD_CONFIRM_MS) {
      nav.flow_ok = true;
    }
  } else {
    nav.flow_good_start_ms = 0;
    nav.flow_ok = false;
  }

  // EKF Z: predict + update từ ToF đã bù tilt
  if (nav.range_ok && nav.airborne) {
    if (kf_z.pos <= 0.01f) {
      kfReset(kf_z, z_tilt, 0.0f);
    }

    kfPredict(kf_z, dt, KF_Z_Q_POS, KF_Z_Q_VEL);
    kfUpdatePos(kf_z, z_tilt, qualityToRRange(mtf_quality));

    nav.z = kf_z.pos;
    nav.vz = kf_z.vel;
  } else {
    resetZEstimator();
  }

  // Chỉ tích vị trí XY khi airborne && flow_ok như yêu cầu
if (!nav.airborne) {
  resetXYEstimator();
  return;
}

if (!nav.flow_ok) {
  // Mất flow thì không cập nhật XY, nhưng KHÔNG reset target.
  nav.vx = 0.0f;
  nav.vy = 0.0f;
  return;
}

  float h = constrain(nav.z, 0.20f, 2.50f);

  // Bù flow do quay roll/pitch: pixel = flow - gyro ở báo cáo, ở đây đã quy ra m/s ~ rate * h
  float roll_rate_rad = rate_r_fil * DEG_TO_RAD;
  float pitch_rate_rad = rate_p_fil * DEG_TO_RAD;

  float vx_rot = FLOW_GYRO_PITCH_SIGN * pitch_rate_rad * h * FLOW_COMP_GAIN;  // quay pitch làm sinh flow ảo trục X
  float vy_rot = FLOW_GYRO_ROLL_SIGN  * roll_rate_rad  * h * FLOW_COMP_GAIN;  // quay roll làm sinh flow ảo trục Y

  float vx_body = mtf_vel_x - flow_bias_vx - vx_rot;
  float vy_body = mtf_vel_y - flow_bias_vy - vy_rot;

  if (fabsf(vx_body) < FLOW_VEL_DEADBAND) vx_body = 0.0f;
  if (fabsf(vy_body) < FLOW_VEL_DEADBAND) vy_body = 0.0f;

  // Theo báo cáo: velocity trả về ở body frame, cần đổi sang reference frame bằng yaw
  float yaw_rel = wrapPi(yaw_est - nav.ref_yaw);
  float cy = cosf(yaw_rel);
  float sy = sinf(yaw_rel);

  float vx_ref = cy * vx_body - sy * vy_body;  // BF -> RF
  float vy_ref = sy * vx_body + cy * vy_body;

  // KF XY: predict + update vận tốc
  kfPredict(kf_x, dt, KF_XY_Q_POS, KF_XY_Q_VEL);
  kfPredict(kf_y, dt, KF_XY_Q_POS, KF_XY_Q_VEL);

  float R_flow = qualityToRFlow(mtf_quality);

  kfUpdateVel(kf_x, vx_ref, R_flow);
  kfUpdateVel(kf_y, vy_ref, R_flow);

  // xả trôi chậm để tránh tích phân sai số vô hạn
  kf_x.pos *= (1.0f - FLOW_POS_LEAK);
  kf_y.pos *= (1.0f - FLOW_POS_LEAK);

  kf_x.pos = constrain(kf_x.pos, -XY_MAX_POS, XY_MAX_POS);
  kf_y.pos = constrain(kf_y.pos, -XY_MAX_POS, XY_MAX_POS);

  nav.x = kf_x.pos;
  nav.y = kf_y.pos;
  nav.vx = kf_x.vel;
  nav.vy = kf_y.vel;
}

// ================= CONTROLLERS =================
void computeXYHold(float dt, float &roll_out, float &pitch_out) {
  roll_out = r_set;
  pitch_out = p_set;

  bool xy_hold_active = (
    nav.airborne &&
    nav.flow_ok &&
    flight_mode == 1 &&
    is_armed_int == 1 &&
    pos_hold_flag == 1
  );

  if (!xy_hold_active) {
    nav.xy_hold_ready = false;

    xy_target_vx_fil = 0.0f;
    xy_target_vy_fil = 0.0f;
    xy_i_x = 0.0f;
    xy_i_y = 0.0f;
    xy_last_vel_err_x = 0.0f;
    xy_last_vel_err_y = 0.0f;
    xy_d_x_fil = 0.0f;
    xy_d_y_fil = 0.0f;
    xy_outer_loop_counter = 0;
    return;
  }

  // Vừa bật hold: khóa vị trí hiện tại làm target
  if (!xy_hold_active) {
  // Chỉ reset target khi thật sự thoát hold:
  // disarm, tắt H, hoặc Python báo không hold.
  if (is_armed_int == 0 || flight_mode != 1 || pos_hold_flag == 0) {
    nav.xy_hold_ready = false;

    xy_target_vx_fil = 0.0f;
    xy_target_vy_fil = 0.0f;
    xy_i_x = 0.0f;
    xy_i_y = 0.0f;
  }

  xy_last_vel_err_x = 0.0f;
  xy_last_vel_err_y = 0.0f;
  xy_d_x_fil = 0.0f;
  xy_d_y_fil = 0.0f;
  xy_outer_loop_counter = 0;

  return;
}

  float pos_err_x = nav.target_x - nav.x;
  float pos_err_y = nav.target_y - nav.y;

  if (fabsf(pos_err_x) < XY_POS_DEADBAND) pos_err_x = 0.0f;
  if (fabsf(pos_err_y) < XY_POS_DEADBAND) pos_err_y = 0.0f;

  float kp_x = XY_POS_P_SMALL;
  float kp_y = XY_POS_P_SMALL;

  if (fabsf(pos_err_x) > 0.35f) kp_x = XY_POS_P_BIG;
  else if (fabsf(pos_err_x) > 0.12f) kp_x = XY_POS_P_MID;

  if (fabsf(pos_err_y) > 0.35f) kp_y = XY_POS_P_BIG;
  else if (fabsf(pos_err_y) > 0.12f) kp_y = XY_POS_P_MID;

  // Outer loop vị trí có thể chạy chậm hơn inner loop vận tốc
  float target_vx_raw = xy_target_vx_fil;
  float target_vy_raw = xy_target_vy_fil;

  xy_outer_loop_counter++;
  if (xy_outer_loop_counter >= XY_POS_LOOP_DIV) {
    xy_outer_loop_counter = 0;
    target_vx_raw = constrain(pos_err_x * kp_x, -XY_MAX_VEL, XY_MAX_VEL);
    target_vy_raw = constrain(pos_err_y * kp_y, -XY_MAX_VEL, XY_MAX_VEL);
  }

  // lọc setpoint vận tốc để hệ không bị giật
  xy_target_vx_fil = lowPassHz(target_vx_raw, xy_target_vx_fil, XY_VEL_SP_LPF_HZ, dt);
  xy_target_vy_fil = lowPassHz(target_vy_raw, xy_target_vy_fil, XY_VEL_SP_LPF_HZ, dt);

  float vel_err_x = xy_target_vx_fil - nav.vx;
  float vel_err_y = xy_target_vy_fil - nav.vy;

  xy_i_x += vel_err_x * dt;
  xy_i_y += vel_err_y * dt;

  xy_i_x = constrain(xy_i_x, -XY_I_LIMIT, XY_I_LIMIT);
  xy_i_y = constrain(xy_i_y, -XY_I_LIMIT, XY_I_LIMIT);

  if (pos_err_x == 0.0f && fabsf(nav.vx) < FLOW_VEL_DEADBAND) {
    vel_err_x = 0.0f;
    xy_i_x = 0.0f;
    xy_last_vel_err_x = 0.0f;
    xy_d_x_fil = 0.0f;
  }

  if (pos_err_y == 0.0f && fabsf(nav.vy) < FLOW_VEL_DEADBAND) {
    vel_err_y = 0.0f;
    xy_i_y = 0.0f;
    xy_last_vel_err_y = 0.0f;
    xy_d_y_fil = 0.0f;
  }


  float d_x_raw = (vel_err_x - xy_last_vel_err_x) / dt;
  float d_y_raw = (vel_err_y - xy_last_vel_err_y) / dt;

  xy_d_x_fil = lowPassHz(d_x_raw, xy_d_x_fil, XY_D_FILTER_HZ, dt);  // lọc D để đỡ nhiễu flow
  xy_d_y_fil = lowPassHz(d_y_raw, xy_d_y_fil, XY_D_FILTER_HZ, dt);

  xy_last_vel_err_x = vel_err_x;
  xy_last_vel_err_y = vel_err_y;

  float acc_ref_x =
    vel_err_x * XY_VEL_P +
    xy_i_x * XY_VEL_I +
    xy_d_x_fil * XY_VEL_D;

  float acc_ref_y =
    vel_err_y * XY_VEL_P +
    xy_i_y * XY_VEL_I +
    xy_d_y_fil * XY_VEL_D;

  acc_ref_x = constrain(acc_ref_x, -XY_MAX_ACCEL, XY_MAX_ACCEL);
  acc_ref_y = constrain(acc_ref_y, -XY_MAX_ACCEL, XY_MAX_ACCEL);

  // controller vị trí/vận tốc làm việc trên RF, còn output góc phải trả về BF
  float yaw_rel = wrapPi(yaw_est - nav.ref_yaw);
  float cy = cosf(yaw_rel);
  float sy = sinf(yaw_rel);

  float acc_body_x = cy * acc_ref_x + sy * acc_ref_y;   // RF -> BF
  float acc_body_y = -sy * acc_ref_x + cy * acc_ref_y;  // RF -> BF

  // Báo cáo dùng asin(u*m/T), ở đây gần đúng T≈mg nên atan2(acc/g) ổn định và đơn giản hơn
  float pitch_cmd = atan2f(acc_body_x, GRAVITY_MS2) * 57.2957795f * XY_PITCH_HOLD_GAIN;
  float roll_cmd  = atan2f(acc_body_y, GRAVITY_MS2) * 57.2957795f * XY_ROLL_HOLD_GAIN;

  pitch_out = constrain(pitch_cmd, -XY_MAX_PITCH_ANGLE, XY_MAX_PITCH_ANGLE);
  roll_out  = constrain(roll_cmd,  -XY_MAX_ROLL_ANGLE, XY_MAX_ROLL_ANGLE);
}

bool computeAltHoldMTF02(float dt) {
  bool alt_hold_active = (
    nav.airborne &&
    nav.range_ok &&
    flight_mode == 1 &&
    is_armed_int == 1 &&
    mtf_quality > 30
  );

  if (!alt_hold_active) {
    nav.z_hold_ready = false;

    alt_target_vz_fil = 0.0f;
    alt_i = 0.0f;
    alt_last_vz_err = 0.0f;
    alt_d_fil = 0.0f;
    alt_output_limited = 0.0f;
    return false;
  }

  if (!nav.z_hold_ready) {
    nav.target_z = nav.z;  // khóa độ cao hiện tại khi vừa bật hold

    alt_target_vz_fil = 0.0f;
    alt_i = 0.0f;
    alt_last_vz_err = 0.0f;
    alt_d_fil = 0.0f;
    alt_output_limited = 0.0f;

    nav.z_hold_ready = true;
  }

  // Stick throttle chỉ đổi climb-rate, không đổi trực tiếp PWM; đỡ giật cao độ
  float stick_climb_rate = 0.0f;

  if (throttle > 1550.0f) {
    stick_climb_rate = (throttle - 1550.0f) / 450.0f * ALT_MAX_CLIMB_RATE;
  } else if (throttle < 1450.0f) {
    stick_climb_rate = (throttle - 1450.0f) / 450.0f * ALT_MAX_CLIMB_RATE;
  }

  nav.target_z += stick_climb_rate * dt;
  nav.target_z = constrain(nav.target_z, 0.30f, 1.80f);

  float alt_err = nav.target_z - nav.z;
  if (fabsf(alt_err) < ALT_DEADBAND) alt_err = 0.0f;

  float kp_alt = ALT_POS_P_SMALL;
  if (fabsf(alt_err) > 0.30f) kp_alt = ALT_POS_P_BIG;
  else if (fabsf(alt_err) > 0.10f) kp_alt = ALT_POS_P_MID;

  float target_vz_raw = alt_err * kp_alt + stick_climb_rate;
  target_vz_raw = constrain(target_vz_raw, -ALT_MAX_CLIMB_RATE, ALT_MAX_CLIMB_RATE);

  alt_target_vz_fil = lowPassHz(target_vz_raw, alt_target_vz_fil, ALT_TARGET_VZ_LPF_HZ, dt);  // setpoint mượt hơn

  float vz_err = alt_target_vz_fil - nav.vz;

  alt_i += vz_err * dt;
  alt_i = constrain(alt_i, -1.5f, 1.5f);

  float d_vz_raw = (vz_err - alt_last_vz_err) / dt;
  alt_d_fil = lowPassHz(d_vz_raw, alt_d_fil, ALT_D_FILTER_HZ, dt);
  alt_last_vz_err = vz_err;

  float raw_out =
    ALT_VEL_P * vz_err +
    ALT_VEL_I * alt_i +
    ALT_VEL_D * alt_d_fil;

  raw_out = constrain(raw_out, -ALT_MAX_CORRECTION, ALT_MAX_CORRECTION);

  float max_step = ALT_OUTPUT_SLEW * dt;  // slew limit để ga không nhảy đột ngột
  if (raw_out > alt_output_limited + max_step) {
    alt_output_limited += max_step;
  } else if (raw_out < alt_output_limited - max_step) {
    alt_output_limited -= max_step;
  } else {
    alt_output_limited = raw_out;
  }

  pid_output_altitude = alt_output_limited;

  // đồng bộ với nhánh baro backup để không bị xung khi rơi sang backup
  pid_altitude_setpoint = actual_pressure;
  manual_altitude_change = 0;
  manual_throttle = 0.0f;

  return true;
}

// ================= MOTOR =================
void writeMotor(int ch, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = (us * 16384) / 20000;
  ledcWrite(ch, duty);
}

// ================= WEB PID =================
void handleRoot() {
  char html[1600];

  snprintf(
    html,
    sizeof(html),
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>body{font-family:Arial;background:#1e1e1e;color:#fff;padding:20px;text-align:center;}"
    "input{width:60px;padding:6px;margin:2px;border-radius:5px;border:none;text-align:center;font-weight:bold;}"
    ".btn{padding:12px 25px;background:#00c853;color:white;border:none;border-radius:5px;font-size:16px;font-weight:bold;cursor:pointer;width:100%%;margin-top:15px;}"
    ".card{background:#2d2d2d;padding:10px;border-radius:10px;margin-bottom:10px;}"
    "h3{margin-top:5px;margin-bottom:10px;color:#4caf50;}</style></head>"
    "<body><h2>TUY CHINH PID</h2><form action='/update'>"
    "<div class='card'><h3>Roll/Pitch Rate</h3>"
    "P:<input type='text' name='pr' value='%.3f'>"
    "I:<input type='text' name='ir' value='%.4f'>"
    "D:<input type='text' name='dr' value='%.4f'></div>"
    "<div class='card'><h3>Roll/Pitch Angle</h3>"
    "P_Goc:<input type='text' name='pa' value='%.3f'></div>"
    "<div class='card'><h3>Yaw Rate</h3>"
    "P:<input type='text' name='py' value='%.3f'>"
    "I:<input type='text' name='iy' value='%.4f'></div>"
    "<input type='submit' class='btn' value='CAP NHAT'></form></body></html>",
    Kp_rate_rp,
    Ki_rate_rp,
    Kd_rate_rp,
    Kp_angle_rp,
    Kp_rate_yaw,
    Ki_rate_yaw
  );

  server.send(200, "text/html", html);
}

void handleUpdate() {
  if (server.hasArg("pr")) Kp_rate_rp = server.arg("pr").toFloat();
  if (server.hasArg("ir")) Ki_rate_rp = server.arg("ir").toFloat();
  if (server.hasArg("dr")) Kd_rate_rp = server.arg("dr").toFloat();
  if (server.hasArg("pa")) Kp_angle_rp = server.arg("pa").toFloat();
  if (server.hasArg("py")) Kp_rate_yaw = server.arg("py").toFloat();
  if (server.hasArg("iy")) Ki_rate_yaw = server.arg("iy").toFloat();

  r_i = 0.0f;
  p_i = 0.0f;
  y_i = 0.0f;

  r_d_fil = 0.0f;
  p_d_fil = 0.0f;
  y_d_fil = 0.0f;

  server.sendHeader("Location", "/");
  server.send(303);
}

// ================= IMU CALIB =================
void calibrateIMU() {
  Serial.println("\n[SYSTEM] DANG CALIBRATE IMU... DE YEN DRONE");

  int valid_samples = 0;

  while (valid_samples < 500) {
    int16_t ax, ay, az, gx, gy, gz;

    if (readICMRaw(ax, ay, az, gx, gy, gz)) {
      gyro_x_cal += gx;
      gyro_y_cal += gy;
      gyro_z_cal += gz;

      acc_roll_cal += atan2f((float)ay, (float)az) * 180.0f / PI;
      acc_pitch_cal += atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / PI;

      valid_samples++;
    }

    delay(4);
  }

  gyro_x_cal /= 500.0f;
  gyro_y_cal /= 500.0f;
  gyro_z_cal /= 500.0f;

  acc_roll_cal /= 500.0f;
  acc_pitch_cal /= 500.0f;

  // reset toàn bộ filter sau calib để triệt giá trị cũ
  kalman_roll = 0.0f;
  kalman_pitch = 0.0f;
  roll_f = 0.0f;
  pitch_f = 0.0f;
  yaw_est = 0.0f;

  rate_r_fil = 0.0f;
  rate_p_fil = 0.0f;
  rate_y_fil = 0.0f;
  acc_r_fil = 0.0f;
  acc_p_fil = 0.0f;

  flow_bias_vx = 0.0f;
  flow_bias_vy = 0.0f;
  memset(flow_ma_x, 0, sizeof(flow_ma_x));
  memset(flow_ma_y, 0, sizeof(flow_ma_y));
  flow_ma_sum_x = 0.0f;
  flow_ma_sum_y = 0.0f;
  flow_ma_idx = 0;

  resetXYEstimator();
  resetZEstimator();

  Serial.println("[SYSTEM] CALIBRATE XONG!");
}

// ================= COMM TASK CORE 0 =================
void CommTask(void *pv) {
  uint8_t lidar_buf[512];

  static unsigned long last_failsafe_tick = 0;
  static unsigned long ground_touch_time = 0;

  for (;;) {
    server.handleClient();

    int packetSize = udp_cmd.parsePacket();

    if (packetSize) {
      char buf[128];
      int len = udp_cmd.read(buf, 127);

      if (len > 0) {
        buf[len] = 0;

        float temp_throttle = 1000.0f;
        int arm_cmd = 0;

        // parse 8 trường UDP: throttle, roll, pitch, yaw, arm, H, auto_land, pos_hold
        int parsed = sscanf(
          buf,
          "%f,%f,%f,%f,%d,%d,%d,%d",
          &temp_throttle,
          &r_set,
          &p_set,
          &y_set,
          &arm_cmd,
          &flight_mode,
          &auto_land_flag,
          &pos_hold_flag
        );

        if (parsed == 8) {
          last_cmd_time = millis();

          if (arm_cmd == 1 && is_armed_int == 0) {
            if (temp_throttle <= 1050.0f) {
              is_armed_int = 1;
              Serial.println("[UDP] ARM OK");
            } else {
              Serial.println("[UDP] ARM REJECT - throttle high");
            }
          }

          if (arm_cmd == 0 && is_armed_int == 1) {
            is_armed_int = 0;
            throttle = 1000.0f;
            flight_mode = 0;
            auto_land_flag = 0;
            Serial.println("[UDP] DISARM OK");
          }

          if (auto_land_flag == 0) {
            throttle = temp_throttle;
          }

          // Serial.printf(
          //   "[UDP] T:%4.0f R:%5.2f P:%5.2f Y:%5.2f ARM:%d FM:%d LAND:%d HOLD:%d\n",
          //   throttle, r_set, p_set, y_set, is_armed_int, flight_mode, auto_land_flag, pos_hold_flag
          // );
        }
      }
    }

    if (auto_land_flag == 1 || (millis() - last_cmd_time > 1000)) {
      r_set = 0.0f;
      p_set = 0.0f;
      y_set = 0.0f;
      flight_mode = 0;

      if (millis() - last_failsafe_tick > 200) {
        throttle -= 1.0f;
        last_failsafe_tick = millis();
      }

      if (throttle < (hover_throttle - 80.0f) && abs(parachute_throttle) < 15) {
        if (ground_touch_time == 0) {
          ground_touch_time = millis();
        } else if (millis() - ground_touch_time > 1000) {
          throttle = 1000.0f;
          is_armed_int = 0;
          auto_land_flag = 0;
          ground_touch_time = 0;
        }
      } else {
        ground_touch_time = 0;
      }

      if (throttle <= 1050.0f) {
        throttle = 1000.0f;
        is_armed_int = 0;
        auto_land_flag = 0;
        ground_touch_time = 0;
      }
    }

    int availableBytes = Serial2.available();

    if (availableBytes >= 100) {
      int bytesToRead = (availableBytes > 512) ? 512 : availableBytes;
      int len = Serial2.read(lidar_buf, bytesToRead);

      if (len > 0) {
        udp_lidar.beginPacket(laptop_ip, 8889);
        udp_lidar.write(lidar_buf, len);
        udp_lidar.endPacket();
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(20);

  // Wake up sensor
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro/acc DLPF như code gốc
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x1D);
  Wire.write(0x06);
  Wire.endTransmission();

  calibrateIMU();

  SerialMTF.begin(115200, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);

  bmp = new Adafruit_BMP280(&Wire);

  if (!bmp->begin(0x76)) {
    Serial.println("[BMP280] 0x76 FAIL, TRY 0x77");
    bmp->begin(0x77);
  }

  delay(100);

  float init_p = bmp->readPressure();

  for (int i = 0; i < 20; i++) {
    pressure_rotating_mem[i] = (int32_t)init_p;
  }

  pressure_total_avarage = (int32_t)(init_p * 20.0f);
  actual_pressure_slow = init_p;
  actual_pressure_fast = init_p;
  ground_pressure = init_p;

  for (int i = 0; i < 4; i++) {
    ledcSetup(i, 50, 14);

    int p = (i == 0) ? M1_PIN : (i == 1) ? M2_PIN : (i == 2) ? M3_PIN : M4_PIN;

    ledcAttachPin(p, i);
    writeMotor(i, 1000);
  }

  Serial2.begin(115200, SERIAL_8N1, LIDAR_RX, -1);

  ledcSetup(4, 10000, 8);
  ledcAttachPin(LIDAR_MOT, 4);
  ledcWrite(4, 50);

  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_GET, handleUpdate);
  server.begin();

  udp_cmd.begin(8888);

  delay(1000);

  last_cmd_time = millis();
  timer = micros();
  last_debug_time = millis();

  xTaskCreatePinnedToCore(CommTask, "Comm", 8192, NULL, 1, NULL, 0);

  Serial.println("[SYSTEM] READY");
}

// ================= LOOP 250Hz =================
void loop() {
  unsigned long loop_start = micros();

  float dt = (loop_start - timer) * 0.000001f;
  timer = loop_start;

  if (dt <= 0.0f || dt > 0.02f) {
    dt = 0.004f;
  }

  static float final_throttle = 1000.0f;

  readMTF02();

  // ===== IMU READ + FILTER =====
  int16_t ax, ay, az, gx, gy, gz;
  bool imu_ok = readICMRaw(ax, ay, az, gx, gy, gz);

  if (!imu_ok) {
    while (micros() - loop_start < 4000) { }
    return;
  }

  imu_ax = ax;
  imu_ay = ay;
  imu_az = az;
  imu_gx = gx;
  imu_gy = gy;
  imu_gz = gz;

  float rate_r_raw = (gx - gyro_x_cal) * 0.00763358f;
  float rate_p_raw = (gy - gyro_y_cal) * 0.00763358f;
  float rate_y_raw = (gz - gyro_z_cal) * 0.00763358f;

  rate_r_fil = rate_r_fil * (1.0f - GYRO_LPF_ALPHA) + rate_r_raw * GYRO_LPF_ALPHA;
  rate_p_fil = rate_p_fil * (1.0f - GYRO_LPF_ALPHA) + rate_p_raw * GYRO_LPF_ALPHA;
  rate_y_fil = rate_y_fil * (1.0f - GYRO_LPF_ALPHA) + rate_y_raw * GYRO_LPF_ALPHA;

  float rate_r = rate_r_fil;
  float rate_p = rate_p_fil;
  float yaw_rate = rate_y_fil;

  float acc_norm = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / ACC_LSB_PER_G;
  bool acc_trusted = (acc_norm > ACC_NORM_MIN && acc_norm < ACC_NORM_MAX);

  float acc_r_raw = atan2f((float)ay, (float)az) * 57.2957795f - acc_roll_cal;
  float acc_p_raw = atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * 57.2957795f - acc_pitch_cal;

  acc_r_fil = acc_r_fil * (1.0f - ACC_LPF_ALPHA) + acc_r_raw * ACC_LPF_ALPHA;
  acc_p_fil = acc_p_fil * (1.0f - ACC_LPF_ALPHA) + acc_p_raw * ACC_LPF_ALPHA;

  // ===== COMPLEMENTARY FILTER ROLL/PITCH =====
  // Thay Kalman cũ bằng complementary filter để accel kéo drift gyro tốt hơn
  float rate_r_angle = rate_r;
  float rate_p_angle = rate_p;
  float yaw_rate_angle = yaw_rate;

  if (fabsf(rate_r_angle) < GYRO_DEADBAND_DPS) rate_r_angle = 0.0f;
  if (fabsf(rate_p_angle) < GYRO_DEADBAND_DPS) rate_p_angle = 0.0f;
  if (fabsf(yaw_rate_angle) < GYRO_DEADBAND_DPS) yaw_rate_angle = 0.0f;

  float gyro_roll_angle = roll_f + rate_r_angle * dt;
  float gyro_pitch_angle = pitch_f + rate_p_angle * dt;

  float yaw_delta = yaw_rate_angle * dt * DEG_TO_RAD;
  gyro_pitch_angle -= gyro_roll_angle * sinf(yaw_delta);
  gyro_roll_angle  += gyro_pitch_angle * sinf(yaw_delta);

  yaw_est = wrapPi(yaw_est + yaw_delta);  // yaw tương đối để đổi hệ trục XY

  if (acc_trusted) {
    roll_f  = ATT_GYRO_WEIGHT * gyro_roll_angle  + (1.0f - ATT_GYRO_WEIGHT) * acc_r_fil;
    pitch_f = ATT_GYRO_WEIGHT * gyro_pitch_angle + (1.0f - ATT_GYRO_WEIGHT) * acc_p_fil;
  } else {
    roll_f = gyro_roll_angle;
    pitch_f = gyro_pitch_angle;
  }

  // Đồng bộ tên biến cũ để phần khác không bị ảnh hưởng
  kalman_roll = roll_f;
  kalman_pitch = pitch_f;

  // ===== NAV ESTIMATOR =====
  updateNavEstimator(dt);

  // ===== XY HOLD =====
  float final_r_set = r_set;
  float final_p_set = p_set;

  computeXYHold(dt, final_r_set, final_p_set);

  float r_target_rate = constrain((final_r_set - roll_f) * Kp_angle_rp, -100.0f, 100.0f);
  float p_target_rate = constrain((final_p_set - pitch_f) * Kp_angle_rp, -100.0f, 100.0f);

  // ===== BMP280 FILTER =====
  barometer_counter++;

  if (barometer_counter == 1) {
    P = bmp->readPressure();
  }

  if (barometer_counter == 2) {
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem[pressure_rotating_mem_location] = (int32_t)P;
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];

    pressure_rotating_mem_location++;
    if (pressure_rotating_mem_location == 20) pressure_rotating_mem_location = 0;

    actual_pressure_fast = (float)pressure_total_avarage / 20.0f;
    actual_pressure_slow = actual_pressure_slow * 0.985f + actual_pressure_fast * 0.015f;

    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8.0f) actual_pressure_diff = 8.0f;
    if (actual_pressure_diff < -8.0f) actual_pressure_diff = -8.0f;

    if (actual_pressure_diff > 1.0f || actual_pressure_diff < -1.0f) {
      actual_pressure_slow -= actual_pressure_diff / 6.0f;
    }

    actual_pressure = actual_pressure_slow;
  }

  if (barometer_counter == 3) {
    barometer_counter = 0;

    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10.0f;
    }

    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = (int32_t)(actual_pressure * 10.0f - pressure_parachute_previous);
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];

    pressure_parachute_previous = actual_pressure * 10.0f;

    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;

    if (is_armed_int == 0) {
      ground_pressure = actual_pressure;
    }

    // ===== ALT HOLD Z: MTF02 FIRST, BMP280 BACKUP =====
    if (flight_mode == 1 && is_armed_int == 1) {
      bool mtf_alt_used = computeAltHoldMTF02(dt);

      if (!mtf_alt_used) {
        // Nhánh backup giữ cao bằng BMP280 giữ nguyên cấu trúc cũ
        if (pid_altitude_setpoint == 0.0f) {
          pid_altitude_setpoint = actual_pressure;
        }

        manual_altitude_change = 0;
        manual_throttle = 0.0f;

        if (throttle > 1550.0f) {
          manual_altitude_change = 1;
          pid_altitude_setpoint = actual_pressure;
          manual_throttle = (throttle - 1550.0f) / 3.0f;
        }

        if (throttle < 1450.0f) {
          manual_altitude_change = 1;
          pid_altitude_setpoint = actual_pressure;
          manual_throttle = (throttle - 1450.0f) / 5.0f;
        }

        pid_altitude_input = actual_pressure;
        pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

        if (pid_error_temp > -4.0f && pid_error_temp < 4.0f) {
          pid_error_temp = 0.0f;
        }

        pid_error_gain_altitude = 0.0f;
        if (pid_error_temp > 10.0f || pid_error_temp < -10.0f) {
          pid_error_gain_altitude = (fabsf(pid_error_temp) - 10.0f) / 20.0f;
          if (pid_error_gain_altitude > 3.0f) pid_error_gain_altitude = 3.0f;
        }

        pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
        pid_i_mem_altitude = constrain(pid_i_mem_altitude, -(float)pid_max_altitude, (float)pid_max_altitude);

        pid_output_altitude =
          (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp +
          pid_i_mem_altitude +
          pid_d_gain_altitude * parachute_throttle;

        pid_output_altitude = constrain(pid_output_altitude, -(float)pid_max_altitude, (float)pid_max_altitude);

        nav.z_hold_ready = false;  // nếu rơi qua backup thì lần sau dùng MTF02 sẽ lock lại target_z
      }
    } else {
      if (pid_altitude_setpoint != 0.0f || nav.z_hold_ready == true) {
        pid_altitude_setpoint = 0.0f;
        mtf_alt_setpoint = 0.0f;
        mtf_alt_i = 0.0f;

        pid_output_altitude = 0.0f;
        pid_i_mem_altitude = 0.0f;

        alt_target_vz_fil = 0.0f;
        alt_i = 0.0f;
        alt_last_vz_err = 0.0f;
        alt_d_fil = 0.0f;
        alt_output_limited = 0.0f;

        manual_throttle = 0.0f;
        manual_altitude_change = 1;

        nav.z_hold_ready = false;
      }
    }
  }

  // ===== DEBUG + IMU UDP =====
  if (millis() - last_debug_time >= 50) {
    last_debug_time = millis();

    Serial.printf(
      "ARM:%d H:%d AIR:%d FLOW:%d HOLD:%d Q:%d "
      "\nR:%5.2f P:%5.2f Yaw:%.2f "
      "\nFR:%5.2f FP:%5.2f "
      "\nRawZ:%.2f Z:%.2f VZ:%.2f ZT:%.2f OUT:%5.1f "
      "\nVRawX:%.3f VRawY:%.3f VX:%.3f VY:%.3f X:%.2f Y:%.2f TX:%.2f TY:%.2f "
      "\nBvx:%.3f Bvy:%.3f ACC:%.2f TRUST:%d\n",
      is_armed_int,
      flight_mode,
      nav.airborne,
      nav.flow_ok,
      pos_hold_flag,
      mtf_quality,
      roll_f,
      pitch_f,
      yaw_est * 57.2957795f,
      final_r_set,
      final_p_set,
      mtf_distance,
      nav.z,
      nav.vz,
      nav.target_z,
      pid_output_altitude,
      mtf_vel_x,
      mtf_vel_y,
      nav.vx,
      nav.vy,
      nav.x,
      nav.y,
      nav.target_x,
      nav.target_y,
      flow_bias_vx,
      flow_bias_vy,
      acc_norm,
      acc_trusted ? 1 : 0
    );

    char imu_buf[128];
    snprintf(
      imu_buf,
      sizeof(imu_buf),
      "%d,%d,%d,%d,%d,%d,%.2f,%.2f",
      imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz, roll_f, pitch_f
    );

    udp_imu.beginPacket(laptop_ip, 8890);
    udp_imu.printf("%s", imu_buf);
    udp_imu.endPacket();
  }

  // ===== RATE PID =====
  float r_rate_e = r_target_rate - rate_r;
  if (throttle > 1150.0f) {
    r_i += r_rate_e * dt;
    r_i = constrain(r_i, -400.0f, 400.0f);
  } else {
    r_i = 0.0f;
  }

  float r_d_raw = (r_rate_e - r_prev_rate_e) / dt;
  r_d_fil = r_d_fil + D_ALPHA * (r_d_raw - r_d_fil);
  float r_out = r_rate_e * Kp_rate_rp + r_i * Ki_rate_rp + r_d_fil * Kd_rate_rp;
  r_prev_rate_e = r_rate_e;

  float p_rate_e = p_target_rate - rate_p;
  if (throttle > 1150.0f) {
    p_i += p_rate_e * dt;
    p_i = constrain(p_i, -400.0f, 400.0f);
  } else {
    p_i = 0.0f;
  }

  float p_d_raw = (p_rate_e - p_prev_rate_e) / dt;
  p_d_fil = p_d_fil + D_ALPHA * (p_d_raw - p_d_fil);
  float p_out = p_rate_e * Kp_rate_rp + p_i * Ki_rate_rp + p_d_fil * Kd_rate_rp;
  p_prev_rate_e = p_rate_e;

  float y_target_rate = y_set;
  float y_rate_e = y_target_rate - yaw_rate;
  if (throttle > 1150.0f) {
    y_i += y_rate_e * dt;
    y_i = constrain(y_i, -250.0f, 250.0f);
  } else {
    y_i = 0.0f;
  }

  float y_d_raw = (y_rate_e - y_prev_rate_e) / dt;
  y_d_fil = y_d_fil + D_ALPHA * (y_d_raw - y_d_fil);
  float y_out = y_rate_e * Kp_rate_yaw + y_i * Ki_rate_yaw + y_d_fil * Kd_rate_yaw;
  y_prev_rate_e = y_rate_e;
  y_out = constrain(y_out, -300.0f, 300.0f);

  // ===== THROTTLE / MIXER =====
  static int last_flight_mode = 0;
  final_throttle = 1000.0f;

  if (flight_mode == 1) {
    recovering_from_althold = false;

    if (last_flight_mode == 0) {
      hover_throttle = throttle;

      pid_altitude_setpoint = actual_pressure;
      pid_i_mem_altitude = 0.0f;

      pid_output_altitude = 0.0f;
      manual_throttle = 0.0f;

      alt_target_vz_fil = 0.0f;
      alt_i = 0.0f;
      alt_last_vz_err = 0.0f;
      alt_d_fil = 0.0f;
      alt_output_limited = 0.0f;
      nav.z_hold_ready = false;

      if (!pos_hold_flag) {
        nav.xy_hold_ready = false;
      }

      manual_altitude_change = 0;
    }

    final_throttle = hover_throttle + pid_output_altitude + manual_throttle;
    final_throttle = constrain(final_throttle, 1100.0f, 1800.0f);
    current_manual_throttle = final_throttle;
  } else {
    if (last_flight_mode == 1) {
      recovering_from_althold = true;
    }

    if (recovering_from_althold) {
      if (current_manual_throttle < throttle) {
        current_manual_throttle += 250.0f * dt;
      } else if (current_manual_throttle > throttle) {
        current_manual_throttle -= 250.0f * dt;
      }

      if (fabsf(current_manual_throttle - throttle) < 5.0f) {
        recovering_from_althold = false;
      }

      final_throttle = current_manual_throttle;
    } else {
      final_throttle = throttle;
    }
  }

  last_flight_mode = flight_mode;

  // Failsafe góc lớn
  if (fabsf(roll_f) > 60.0f || fabsf(pitch_f) > 60.0f) {
    is_armed_int = 0;
    final_throttle = 1000.0f;
  }

  if (is_armed_int == 1) {
    if (final_throttle > 1050.0f) {
      writeMotor(0, (int)(final_throttle - p_out + r_out + y_out));
      writeMotor(1, (int)(final_throttle - p_out - r_out - y_out));
      writeMotor(2, (int)(final_throttle + p_out + r_out - y_out));
      writeMotor(3, (int)(final_throttle + p_out - r_out + y_out));
    } else {
      if (fabsf(roll_f) < 10.0f && fabsf(pitch_f) < 10.0f) {
        for (int i = 0; i < 4; i++) writeMotor(i, 1050);
      } else {
        for (int i = 0; i < 4; i++) writeMotor(i, 1000);
      }

      r_i = 0.0f;
      p_i = 0.0f;
      y_i = 0.0f;
      pid_i_mem_altitude = 0.0f;
      alt_i = 0.0f;
      xy_i_x = 0.0f;
      xy_i_y = 0.0f;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      writeMotor(i, 1000);
    }

    r_i = 0.0f;
    p_i = 0.0f;
    y_i = 0.0f;
    pid_i_mem_altitude = 0.0f;
    alt_i = 0.0f;
    xy_i_x = 0.0f;
    xy_i_y = 0.0f;
  }

  while (micros() - loop_start < 4000) { }
}
