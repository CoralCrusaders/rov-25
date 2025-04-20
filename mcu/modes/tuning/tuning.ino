#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "MS5837.h"

#define INC_PROG_ITER(i) i++
#define SERIAL_TRANSMISSION_WRAP (50)

#define AMP_LIMIT (20) // fuse melts at 25 amps, leave 5 amp clearance
#define HOR_AMP_LIMIT (10)
#define VERT_AMP_LIMIT (AMP_LIMIT - HOR_AMP_LIMIT)
// (AMPS) taken from blue robotics t200 specs @ 12V
// https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx
#define AMP_LIST {17.03, 17.08, 16.76, 16.52, 16.08, 15.69, 15.31, 15.00, 14.51, 14.17, 13.82, 13.46, 13.08, 12.80, 12.40, 12.00, 11.66, 11.31, 11.10, 10.74, 10.50, 10.11, 9.84, 9.50, 9.20, 8.90, 8.60, 8.30, 8.00, 7.70, 7.40, 7.10, 6.90, 6.60, 6.40, 6.20, 5.99, 5.77, 5.50, 5.32, 5.17, 4.90, 4.70, 4.56, 4.30, 4.10, 3.90, 3.73, 3.60, 3.40, 3.30, 3.10, 2.98, 2.80, 2.70, 2.41, 2.30, 2.10, 2.00, 1.90, 1.80, 1.70, 1.60, 1.50, 1.31, 1.30, 1.20, 1.10, 1.00, 0.90, 0.80, 0.80, 0.70, 0.60, 0.50, 0.50, 0.41, 0.40, 0.40, 0.30, 0.29, 0.20, 0.20, 0.20, 0.10, 0.10, 0.10, 0.05, 0.05, 0.05, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.05, 0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20, 0.30, 0.30, 0.40, 0.40, 0.50, 0.50, 0.60, 0.70, 0.70, 0.80, 0.80, 1.00, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.80, 2.00, 2.10, 2.20, 2.30, 2.50, 2.80, 2.90, 3.00, 3.20, 3.30, 3.50, 3.67, 3.80, 4.00, 4.20, 4.40, 4.60, 4.80, 5.00, 5.20, 5.40, 5.69, 5.80, 6.00, 6.30, 6.50, 6.70, 7.00, 7.20, 7.50, 7.80, 8.00, 8.32, 8.64, 8.90, 9.24, 9.50, 9.82, 10.14, 10.45, 10.72, 11.10, 11.32, 11.62, 12.01, 12.37, 12.61, 13.04, 13.44, 13.70, 14.11, 14.40, 14.76, 15.13, 15.52, 15.87, 16.30, 16.74, 16.86, 16.91};
// divide pwm power by 4 and add 100 to get index
// ternaries are spaghetti to round to higher current value if not divisible by 4
#define PWR_TO_AMPS(x) (pwr_to_current[(x % 4 == 0) ? (x/4 + 100) : ((x < 0) ? (x/4 + 100 - 1) : (x/4 + 100 + 1))])

#define ESC_MAGNITUDE (400)
#define JOYSTICK_MAGNITUDE (32767)
#define TRIGER_MAGNITUDE (255)
#define JOYSTICK_DEADZONE_CONST 3000
#define TRIGGER_DEADZONE_CONST 10
#define NORMALIZE_JOYSTICK(x) ((double)((double)x / (double)(JOYSTICK_MAGNITUDE)))
#define NORMALIZE_TRIGGER(x) ((double)((double)x / (double)(TRIGER_MAGNITUDE)))
#define THRUSTER_POWER(x) (int32_t)(1500 + x)

#define FOR_GAIN ((double)0.5)
#define LAT_GAIN ((double)0.4)
#define ROT_GAIN ((double)0.2)

#define DEFAULT_MULT ((double)1.0)
#define SLOW_MULT ((double)0.5)

#define S1 A8
#define S2 A0
#define S3 A1
#define S4 A11

#define VFL 8
#define VFR 4
#define VBL 7
#define VBR 3
#define HFL 9
#define HFR 5
#define HBL 6
#define HBR 2

#define VFL_DIR 1
#define VFR_DIR 1
#define VBL_DIR 1
#define VBR_DIR 1
#define HFL_DIR 1
#define HFR_DIR 1
#define HBL_DIR 1
#define HBR_DIR 1


class PIDFS {
  #define PIDF_FORWARD 1
  #define PIDF_REVERSE -1 
  public:
    uint32_t passed_time;
    uint32_t cur_time;
    uint32_t prev_time;

    double cur_err;   
    double prev_err;
    double tot_err;
    double drv_err;  
    
    double Kp;
    double Ki;
    double Kd;
    double Kf;
    double Ks;
    double target;

    double last;

    PIDFS(double kp, double ki, double kd, double kf, double ks);
    double compute(double input);
    void set_target(double tg);
};

PIDFS::PIDFS(double kp, double ki, double kd, double kf, double ks) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  Kf = kf;
  Ks = ks;
}

double PIDFS::compute(double input) {
  cur_time = millis();
  passed_time = cur_time - prev_time;

  cur_err = target - input;
  tot_err += cur_err * passed_time;
  drv_err = (cur_err-prev_err) / passed_time;

  prev_time = cur_time;
  prev_err = cur_err;

  //Serial.print(cur_err);
  //Serial.print(" ");
  //Serial.print(tot_err);
  //Serial.print("\n");

  last = Kp * cur_err + Ki * tot_err + Kd * drv_err + Kf * target + Ks;
  return last;
}

void PIDFS::set_target(double tg) {
    target = tg;
    //prev_time = millis();
    // prev_err = 0;
}

typedef struct __attribute__((packed)) {
    double depth_rate_p;
    double depth_rate_i;
    double depth_rate_d;
    double depth_rate_f;
    double depth_rate_s;
    double depth_pwr_p;
    double depth_pwr_i;
    double depth_pwr_d;
    double depth_pwr_f;
    double depth_pwr_s;
    double yaw_rate_p;
    double yaw_rate_i;
    double yaw_rate_d;
    double yaw_rate_f;
    double yaw_rate_s;
    double yaw_pwr_p;
    double yaw_pwr_i;
    double yaw_pwr_d;
    double yaw_pwr_f;
    double yaw_pwr_s;
    int64_t ABS_LX; // left stick x
    int64_t ABS_LY; // left stick y
    int64_t ABS_RX; // right stick x
    int64_t ABS_RY; // right stick y
    int64_t BTN_THUMBL; // left stick btn
    int64_t BTN_THUMBR; // right stick btn
    int64_t ABS_HAT0X; // dpad x (-1, 0, 1)
    int64_t ABS_HAT0Y; // dpad y (-1, 0, 1)
    int64_t BTN_SOUTH; // A
    int64_t BTN_EAST; // B
    int64_t BTN_NORTH; // X ???
    int64_t BTN_WEST; // Y ???
    int64_t BTN_LB; // LB
    int64_t BTN_RB; // RB
    int64_t ABS_LT; // LT (0, 255)
    int64_t ABS_RT; // RT (0, 255)
    int64_t BTN_START; // start btn
    int64_t BTN_SELECT; // select btn
} input_data_t;

typedef struct {
    double yaw_deg_s;
    double pitch_deg_s;
    double roll_deg_s;
    double yaw_deg;
    double pitch_deg;
    double roll_deg;
    double depth_m;
    double depth_m_s;
} sensor_data_t;

typedef struct {
    int32_t vfl_pwr;
    int32_t vfr_pwr;
    int32_t vbl_pwr;
    int32_t vbr_pwr;
    int32_t hfl_pwr;
    int32_t hfr_pwr;
    int32_t hbl_pwr;
    int32_t hbr_pwr;
} thruster_power_t;
  
typedef struct {
    Servo vfl;
    Servo vfr;
    Servo vbl;
    Servo vbr;
    Servo hfl;
    Servo hfr;
    Servo hbl;
    Servo hbr;
} thrusters_t;

uint32_t prog_iter = 0;

MS5837 bar02_sensor;
Adafruit_BNO08x imu(-1); // -1 means i2c autodetect

sh2_SensorValue_t sensor_value;

double pwr_to_current[201] = AMP_LIST;

double mult;

char sig[6];
char checksum_char[3];

bool slowmode = false;
bool slowmode_btn_avl = true;

input_data_t input_data;
sensor_data_t sensor_data;
thruster_power_t thruster_power;
thrusters_t thrusters;

PIDFS depth_pwr_controller(1, 0, 0, 0, 0);
PIDFS yaw_pwr_controller(0.005, 0, 0, 0, 0);
PIDFS pitch_pwr_controller(0.005, 0, 0, 0, 0);
PIDFS roll_pwr_controller(0.005, 0, 0, 0, 0);

PIDFS depth_rate_controller(1, 0, 0, 0, 0);
PIDFS yaw_rate_controller(1, 0, 0, 0, 0);
PIDFS pitch_rate_controller(1, 0, 0, 0, 0);
PIDFS roll_rate_controller(1, 0, 0, 0, 0);

bool depth_set_avl = true;
bool yaw_set_avl = true;
bool pitch_set_avl = true;
bool roll_set_avl = true;

double yaw_target = 0;
double pitch_target = 0;
double roll_target = 0;

uint32_t last_depth_read;

inline void bar02_setup() {
  while(!bar02_sensor.init()) {
    Serial.println("Failed to initialize bar02");
    delay(1000);
  }
  bar02_sensor.setModel(MS5837::MS5837_02BA);
  bar02_sensor.setFluidDensity(997);
}

inline void BNO08x_setup() {
  while(!imu.begin_I2C()) {
    Serial.println("Failed to initialize BNO08x");
    delay(1000);
  }
  // // linear acceleration
  // while(!imu.enableReport(SH2_LINEAR_ACCELERATION, 2500)) {
  //   Serial.println("Failed to enable BNO08x linear acceleration");
  //   delay(1000);
  // }
  // angular velocity
  while(!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000)) {
    Serial.println("Failed to enable BNO08x angular velocity");
    delay(1000);
  }
  // orientation
  while(!imu.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
    Serial.println("Failed to enable BNO08x game rotation vector");
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  Wire.begin();
  Wire.setClock(400000); // 400kHz
  
  bar02_setup();
  BNO08x_setup();

  thrusters.vfl.attach(VFL);
  thrusters.vfr.attach(VFR);
  thrusters.vbl.attach(VBL);
  thrusters.vbr.attach(VBR);
  thrusters.hfl.attach(HFL);
  thrusters.hfr.attach(HFR);
  thrusters.hbl.attach(HBL);
  thrusters.hbr.attach(HBR);
}

// https://github.com/adafruit/Adafruit_BNO08x/blob/master/examples/quaternion_yaw_pitch_roll/quaternion_yaw_pitch_roll.ino
inline void quaternionToEuler(float qr, float qi, float qj, float qk) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  sensor_data.yaw_deg = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  sensor_data.pitch_deg = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  sensor_data.roll_deg = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  sensor_data.yaw_deg *= RAD_TO_DEG;
  sensor_data.pitch_deg *= RAD_TO_DEG;
  sensor_data.roll_deg *= RAD_TO_DEG;
}

inline void quaternionToEulerRV(sh2_RotationVector_t* rotational_vector) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k);
} 

inline double shortest_angle(double a, double b) {
  double diff = fmod(b - a + 180, 360);
  if (diff < 0) {
    diff += 360;
  }
  return diff - 180;
}

inline bool nmea_checksum() {
  uint8_t checksum = 0;
  for (int i = 0; i < 5; i++)
  {
    checksum ^= sig[i];
  }
  for (int i = 0; i < sizeof(input_data); i++)
  {
    checksum ^= ((char*)&input_data)[i];
  }
  return checksum == strtol(checksum_char, NULL, 16);
}

inline void drain_serial_input() {
  while(Serial.available() > 0 && Serial.peek() != '$') {
    Serial.read();
  }
}

inline void read_serial_input() {
  if(Serial.available() > 0) {
    if(Serial.peek() != '$') {
      Serial.read();
    }
    if(Serial.available() == 0) {
      return;
    }
    if(Serial.peek() != '$') {
      Serial.print("ERRSTX ");
      Serial.println(Serial.read(), HEX);
      drain_serial_input();
      return;
    }
    Serial.read();
    Serial.readBytes(sig, 5);
    if(strcmp(sig, "RPCTL") != 0) {
      Serial.print("ERRSIG ");
      Serial.println(sig);
      drain_serial_input();
      return;
    }
    Serial.readBytes((char*)&input_data, sizeof(input_data));
    if(Serial.peek() != '*') {
      Serial.print("ERRTRM ");
      Serial.println(Serial.read());
      drain_serial_input();
      return;
    }
    Serial.read();
    Serial.readBytes(checksum_char, 2);
    if(!nmea_checksum()) {
      Serial.print("ERRCHK\n");
      drain_serial_input();
      return;
    }
  }
}

inline void set_consts() {
  if(input_data.BTN_LB) {
    if(slowmode_btn_avl) {
      slowmode = !slowmode;
    }
    slowmode_btn_avl = false;  
  } else {
    slowmode_btn_avl = true;
  }
  
  if(slowmode) {
    mult = SLOW_MULT;
  } else { 
    mult = DEFAULT_MULT;
  }
}

inline void read_imu() {
  bool angular_velocity_read = false;
  bool orientation_read = false;
  while(!angular_velocity_read || !orientation_read) {
    if(imu.getSensorEvent(&sensor_value)) {
      if(sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        sensor_data.yaw_deg_s = sensor_value.un.gyroscope.x * RAD_TO_DEG;
        sensor_data.pitch_deg_s = sensor_value.un.gyroscope.y * RAD_TO_DEG;
        sensor_data.roll_deg_s = sensor_value.un.gyroscope.z * RAD_TO_DEG;
        angular_velocity_read = true;
      } else if(sensor_value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        quaternionToEulerRV(&sensor_value.un.gameRotationVector);
        orientation_read = true;
      }
    }
  }
  // if(imu.getSensorEvent(&sensor_value)) {
  //   quaternionToEulerRV(&sensor_value.un.gameRotationVector);
  // }
}

inline void read_bar02() {
  bar02_sensor.read();
  double diff = bar02_sensor.depth() - sensor_data.depth_m;
  sensor_data.depth_m_s = diff / ((millis() - last_depth_read) / 1000.0f);
  last_depth_read = millis();
  sensor_data.depth_m += diff;
  //Serial.println(bar02_sensor.pressure());
}

inline void calc_vert_power() {
  double vfl_pwr = 0;
  double vfr_pwr = 0;
  double vbl_pwr = 0;
  double vbr_pwr = 0;

  double pitch_pwr;
  double desired_pitch_rate;

  double roll_pwr;
  double desired_roll_rate;

  double depth_pwr;
  double desired_depth_rate = 0;

  if(abs(input_data.ABS_RY) != 0) {
    desired_depth_rate = NORMALIZE_JOYSTICK(input_data.ABS_RY) * 1.0f;
    depth_set_avl = true;
  } else {
    if(depth_set_avl) {
      depth_set_avl = false;
      depth_rate_controller.set_target(sensor_data.depth_m);
    }
    desired_depth_rate = depth_rate_controller.compute(sensor_data.depth_m);
  }

  depth_pwr_controller.set_target(desired_depth_rate);
  depth_pwr = depth_pwr_controller.compute(sensor_data.depth_m_s);

  vfl_pwr += depth_pwr;
  vfr_pwr += depth_pwr;
  vbl_pwr += depth_pwr;
  vbr_pwr += depth_pwr;

  double normalize;
  double max_pwr = max(max(max(fabs(vfl_pwr), fabs(vfr_pwr)), fabs(vbl_pwr)), fabs(vbr_pwr));
  if(max_pwr > 1.0f) {
    normalize = 1.0f / max_pwr;
  } else {
    normalize = 1.0f;
  }

  thruster_power.vfl_pwr = (int32_t)(vfl_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.vfr_pwr = (int32_t)(vfr_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.vbl_pwr = (int32_t)(vbl_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.vbr_pwr = (int32_t)(vbr_pwr * ESC_MAGNITUDE * normalize);
}

inline void calc_hor_power() {
  double hfl_pwr = 0;
  double hfr_pwr = 0;
  double hbl_pwr = 0;
  double hbr_pwr = 0;

  if(abs(input_data.ABS_LY) != 0) {
    hfl_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LY) * FOR_GAIN * HFL_DIR * mult;
    hfr_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LY) * FOR_GAIN * HFR_DIR * mult;
    hbl_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LY) * FOR_GAIN * HBL_DIR * mult;
    hbr_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LY) * FOR_GAIN * HBR_DIR * mult;
  }

  if(abs(input_data.ABS_LX) != 0) {
    hfl_pwr -= NORMALIZE_JOYSTICK(input_data.ABS_LX) * LAT_GAIN * HFL_DIR * mult;
    hfr_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LX) * LAT_GAIN * HFR_DIR * mult;
    hbl_pwr += NORMALIZE_JOYSTICK(input_data.ABS_LX) * LAT_GAIN * HBL_DIR * mult;
    hbr_pwr -= NORMALIZE_JOYSTICK(input_data.ABS_LX) * LAT_GAIN * HBR_DIR * mult;
  }

  double yaw_pwr = 0;
  double desired_yaw_rate;
  if(abs(input_data.ABS_LT) != 0 || abs(input_data.ABS_RT) != 0) {
    desired_yaw_rate = NORMALIZE_TRIGGER(input_data.ABS_LT - input_data.ABS_RT) * 180.0f; // replace 180 with how many deg / s at max throttle
    yaw_set_avl = true;
  } else {
    if(yaw_set_avl) {
      yaw_set_avl = false;
      yaw_target = sensor_data.yaw_deg;
      yaw_rate_controller.set_target(0);
    }
    // yaw rate is based on current error 
    desired_yaw_rate = yaw_rate_controller.compute(shortest_angle(sensor_data.yaw_deg, yaw_target));
  }
  yaw_pwr_controller.set_target(desired_yaw_rate);
  yaw_pwr = yaw_pwr_controller.compute(sensor_data.yaw_deg_s);
  hfl_pwr -= yaw_pwr;
  hfr_pwr += yaw_pwr;
  hbl_pwr -= yaw_pwr;
  hbr_pwr += yaw_pwr;

  double normalize;
  double max_pwr = max(max(max(fabs(hfl_pwr), fabs(hfr_pwr)), fabs(hbl_pwr)), fabs(hbr_pwr));
  if(max_pwr > 1.0f) {
    normalize = 1.0f / max_pwr;
  } else {
    normalize = 1.0f;
  }
  thruster_power.hfl_pwr = (int32_t)(hfl_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.hfr_pwr = (int32_t)(hfr_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.hbl_pwr = (int32_t)(hbl_pwr * ESC_MAGNITUDE * normalize);
  thruster_power.hbr_pwr = (int32_t)(hbr_pwr * ESC_MAGNITUDE * normalize);
}

inline double vert_amps() {
  return PWR_TO_AMPS(thruster_power.vfl_pwr) + PWR_TO_AMPS(thruster_power.vfr_pwr) + PWR_TO_AMPS(thruster_power.vbl_pwr) + PWR_TO_AMPS(thruster_power.vbr_pwr);
}

inline double hor_amps() {
  return PWR_TO_AMPS(thruster_power.hfl_pwr) + PWR_TO_AMPS(thruster_power.hfr_pwr) + PWR_TO_AMPS(thruster_power.hbl_pwr) + PWR_TO_AMPS(thruster_power.hbr_pwr);
}

inline void limit_current() {
  if(vert_amps() + hor_amps() <= AMP_LIMIT) {
    return;
  }

  if(vert_amps() > VERT_AMP_LIMIT && hor_amps() > HOR_AMP_LIMIT) {
    double vert_ratio = VERT_AMP_LIMIT / vert_amps();
    double hor_ratio = HOR_AMP_LIMIT / hor_amps();
    thruster_power.vfl_pwr *= vert_ratio;
    thruster_power.vfr_pwr *= vert_ratio;
    thruster_power.vbl_pwr *= vert_ratio;
    thruster_power.vbr_pwr *= vert_ratio;
    thruster_power.hfl_pwr *= hor_ratio;
    thruster_power.hfr_pwr *= hor_ratio;
    thruster_power.hbl_pwr *= hor_ratio;
    thruster_power.hbr_pwr *= hor_ratio;
  } else if(vert_amps() > VERT_AMP_LIMIT) {
    double vert_ratio = (AMP_LIMIT - hor_amps()) / vert_amps();
    thruster_power.vfl_pwr *= vert_ratio;
    thruster_power.vfr_pwr *= vert_ratio;
    thruster_power.vbl_pwr *= vert_ratio;
    thruster_power.vbr_pwr *= vert_ratio;
  } else if(hor_amps() > HOR_AMP_LIMIT) {
    double hor_ratio = (AMP_LIMIT - vert_amps()) / hor_amps();
    thruster_power.hfl_pwr *= hor_ratio;
    thruster_power.hfr_pwr *= hor_ratio;
    thruster_power.hbl_pwr *= hor_ratio;
    thruster_power.hbr_pwr *= hor_ratio;
  }
}

inline void power_thrusters() {
  thrusters.vfl.writeMicroseconds(THRUSTER_POWER(thruster_power.vfl_pwr));
  thrusters.vfr.writeMicroseconds(THRUSTER_POWER(thruster_power.vfr_pwr));
  thrusters.vbl.writeMicroseconds(THRUSTER_POWER(thruster_power.vbl_pwr));
  thrusters.vbr.writeMicroseconds(THRUSTER_POWER(thruster_power.vbr_pwr));
  thrusters.hfl.writeMicroseconds(THRUSTER_POWER(thruster_power.hfl_pwr));
  thrusters.hfr.writeMicroseconds(THRUSTER_POWER(thruster_power.hfr_pwr));
  thrusters.hbl.writeMicroseconds(THRUSTER_POWER(thruster_power.hbl_pwr));
  thrusters.hbr.writeMicroseconds(THRUSTER_POWER(thruster_power.hbr_pwr));
}

inline void elim_deadzones() {
  if(abs(input_data.ABS_LX) < JOYSTICK_DEADZONE_CONST) {
    input_data.ABS_LX = 0;
  }
  if(abs(input_data.ABS_LY) < JOYSTICK_DEADZONE_CONST) {
    input_data.ABS_LY = 0;
  }
  if(abs(input_data.ABS_RX) < JOYSTICK_DEADZONE_CONST) {
    input_data.ABS_RX = 0;
  }
  if(abs(input_data.ABS_RY) < JOYSTICK_DEADZONE_CONST) {
    input_data.ABS_RY = 0;
  }
  if(abs(input_data.ABS_LT) < TRIGGER_DEADZONE_CONST) {
    input_data.ABS_LT = 0;
  }
  if(abs(input_data.ABS_RT) < TRIGGER_DEADZONE_CONST) {
    input_data.ABS_RT = 0;
  }
}

inline void transmit_rov_data() {
  Serial.print("$TNCTL");
  Serial.print(",");
  Serial.print(thruster_power.vfl_pwr);
  Serial.print(",");
  Serial.print(thruster_power.vfr_pwr);
  Serial.print(",");
  Serial.print(thruster_power.vbl_pwr);
  Serial.print(",");
  Serial.print(thruster_power.vbr_pwr);
  Serial.print(",");
  Serial.print(thruster_power.hfl_pwr);
  Serial.print(",");
  Serial.print(thruster_power.hfr_pwr);
  Serial.print(",");
  Serial.print(thruster_power.hbl_pwr);
  Serial.print(",");
  Serial.print(thruster_power.hbr_pwr);
  Serial.print(",");
  Serial.print(sensor_data.depth_m);
  Serial.print(",");
  Serial.print(sensor_data.depth_m_s);
  Serial.print(",");
  Serial.print(sensor_data.yaw_deg);
  Serial.print(",");
  Serial.print(sensor_data.pitch_deg);
  Serial.print(",");
  Serial.print(sensor_data.roll_deg);
  Serial.print(",");
  Serial.print(sensor_data.yaw_deg_s);
  Serial.print(",");
  Serial.print(sensor_data.pitch_deg_s);
  Serial.print(",");
  Serial.print(sensor_data.roll_deg_s);
  Serial.print(",");
  Serial.print(depth_pwr_controller.target);
  Serial.print(",");
  Serial.print(depth_pwr_controller.last);
  Serial.print(",");
  Serial.print(yaw_pwr_controller.target);
  Serial.print(",");
  Serial.print(yaw_pwr_controller.last);
  Serial.print(",");
  // throaway checksum
  Serial.println("*FF");
}

void loop() {
  read_serial_input();
  elim_deadzones();
  set_consts();
  read_imu();
  read_bar02();
  calc_vert_power();
  calc_hor_power();   
  limit_current();
  //power_thrusters();
  if(prog_iter == 0) {
    transmit_rov_data();
  }
  prog_iter = (prog_iter + 1) % SERIAL_TRANSMISSION_WRAP;
}
 
