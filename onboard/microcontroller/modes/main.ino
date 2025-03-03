#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "MS5837.h"

#define INC_PROG_ITER(i) i++
#define SERIAL_TRANSMISSION_WRAP (5)

#define AMP_LIMIT (20) // fuse melts at 25 amps, leave 5 amp clearance
#define TRANS_ROT_AMP_LIMIT (13)
#define VERT_AMP_LIMIT (AMP_LIMIT - TRANS_ROT_AMP_LIMIT)
// (AMPS) taken from blue robotics t200 specs @ 12V
// https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx
#define AMP_LIST double pwr_to_current[201] = {17.03, 17.08, 16.76, 16.52, 16.08, 15.69, 15.31, 15.00, 14.51, 14.17, 13.82, 13.46, 13.08, 12.80, 12.40, 12.00, 11.66, 11.31, 11.10, 10.74, 10.50, 10.11, 9.84, 9.50, 9.20, 8.90, 8.60, 8.30, 8.00, 7.70, 7.40, 7.10, 6.90, 6.60, 6.40, 6.20, 5.99, 5.77, 5.50, 5.32, 5.17, 4.90, 4.70, 4.56, 4.30, 4.10, 3.90, 3.73, 3.60, 3.40, 3.30, 3.10, 2.98, 2.80, 2.70, 2.41, 2.30, 2.10, 2.00, 1.90, 1.80, 1.70, 1.60, 1.50, 1.31, 1.30, 1.20, 1.10, 1.00, 0.90, 0.80, 0.80, 0.70, 0.60, 0.50, 0.50, 0.41, 0.40, 0.40, 0.30, 0.29, 0.20, 0.20, 0.20, 0.10, 0.10, 0.10, 0.05, 0.05, 0.05, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.05, 0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20, 0.30, 0.30, 0.40, 0.40, 0.50, 0.50, 0.60, 0.70, 0.70, 0.80, 0.80, 1.00, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.80, 2.00, 2.10, 2.20, 2.30, 2.50, 2.80, 2.90, 3.00, 3.20, 3.30, 3.50, 3.67, 3.80, 4.00, 4.20, 4.40, 4.60, 4.80, 5.00, 5.20, 5.40, 5.69, 5.80, 6.00, 6.30, 6.50, 6.70, 7.00, 7.20, 7.50, 7.80, 8.00, 8.32, 8.64, 8.90, 9.24, 9.50, 9.82, 10.14, 10.45, 10.72, 11.10, 11.32, 11.62, 12.01, 12.37, 12.61, 13.04, 13.44, 13.70, 14.11, 14.40, 14.76, 15.13, 15.52, 15.87, 16.30, 16.74, 16.86, 16.91};
// divide pwm power by 4 and add 100 to get index
// ternaries are spaghetti to round to higher current value if not divisible by 4
#define PWR_TO_AMPS(x) (pwr_to_current[(x % 4 == 0) ? (x/4 + 100) : ((x < 0) ? (x/4 + 100 - 1) : (x/4 + 100 + 1))])

#define ESC_MAGNITUDE (400)
#define JOYSTICK_MAGNITUDE (32767)
#define DPAD_MAGNITUDE (1)
#define NORMALIZE_JOYSTICK_FACTOR ((double)ESC_MAGNITUDE / (double)JOYSTICK_MAGNITUDE)
#define NORMALIZE_DPAD_FACTOR ((double)ESC_MAGNITUDE / (double)DPAD_MAGNITUDE)
#define NORMALIZE_JOYSTICK(x) ((int32_t)((int32_t)x * (double)NORMALIZE_JOYSTICK_FACTOR))
#define NORMALIZE_DPAD(x) ((int32_t)(((double)x * (double)NORMALIZE_DPAD_FACTOR)))
#define THRUSTER_POWER(x) (int32_t)(1500 + x)

#define DPAD_VERT_GAIN ((double)0.45)

#define ROT_GAIN ((double)0.3)
#define STRAFE_GAIN ((double)0.5)

#define DEADZONE_CONST 2000

#define NEUTRAL_ROLL_DEG 0

#define THRUSTER_LVERT_PIN 2
#define THRUSTER_RVERT_PIN 12
#define THRUSTER_FL_PIN 4
#define THRUSTER_FR_PIN 5
#define THRUSTER_BL_PIN 6
#define THRUSTER_BR_PIN 7

#define THRUSTER_VERT_DIR -1
#define THRUSTER_FL_DIR -1
#define THRUSTER_FR_DIR -1
#define THRUSTER_BL_DIR 1
#define THRUSTER_BR_DIR 1

class PIDF {
  #define PIDF_FORWARD 1
  #define PIDF_REVERSE -1 
  private:
    uint32_t passed_time;
    uint32_t cur_time;
    uint32_t prev_time;

    double cur_err;   
    double prev_err;
    double tot_err;
    double drv_err;  
  public:
    double Kp;
    double Ki;
    double Kd;
    double Kf;
    double target;

    PIDF(double kp, double ki, double kd);
    double compute_PIDF(double input);
    void set_target(double tg);
};

PIDF::PIDF(double kp, double ki, double kd, double kf) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  Kf = kf;
}

double PIDF::compute_PIDF(double input) {
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

  return Kp * cur_err + Ki * tot_err + Kd * drv_err + Kf;
}

void PIDF::set_target(double tg) {
    target = tg;
    prev_time = millis();
    prev_err = 0;
}

typedef struct __attribute__((packed)) {
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
  //double x_accel;
  //double y_accel;
  //double z_accel;
  double pressure_mbar;
  double depth;
} aux_data_t;

typedef struct {
  int32_t lvert_power;
  int32_t rvert_power;
  int32_t fl_power;
  int32_t fr_power;
  int32_t bl_power;
  int32_t br_power;
} control_data_t;

typedef struct {
  Servo lvert;
  Servo rvert;
  Servo fl;
  Servo fr;
  Servo bl;
  Servo br;
} thrusters_t;

typedef struct {
  double yaw;
  double pitch;
  double roll;
} orientation_t;

// declared globally 
uint64_t prog_iter;

AMP_LIST;

MS5837 bar02_sensor;
Adafruit_BNO08x imu(-1); // -1 means i2c autodetect

sh2_SensorValue_t sensor_value;

// 3 power per ~10mm of error
PIDF depth_hold_controller(3, 0, 0, -30);
// 8 power per degree of error
PIDF yaw_hold_controller(8, 0, 0, 0);
int32_t yaw_abs_target = 0;
int32_t yaw_abs_cur = 0;
int32_t yaw_rel_offset = 0;

double DEFAULT_MULT = (double)0.7;
double SLOW_MULT = (double)0.3;

char sig[6];
char checksum_char[3];

input_data_t input_data;
aux_data_t aux_data;
control_data_t control_data = {1500, 1500, 1500, 1500, 1500, 1500};
thrusters_t thrusters;
orientation_t orientation;

double mult = DEFAULT_MULT;

bool slowmode = false;
bool slowmode_btn_avl = true;

bool gain_dec_btn_avl = true;
bool gain_inc_btn_avl = true;

bool depth_hold = false;
bool stabilize = false;

bool depth_hold_btn_avl = true;
bool stabilize_btn_avl = true;

bool depth_set_avl = true;
bool yaw_set_avl = true;

char buff[200];

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  memset(&input_data, 0, sizeof(input_data));

  Wire.begin();
  
  thrusters.lvert.attach(THRUSTER_LVERT_PIN);
  thrusters.rvert.attach(THRUSTER_RVERT_PIN);
  thrusters.fl.attach(THRUSTER_FL_PIN);
  thrusters.fr.attach(THRUSTER_FR_PIN);
  thrusters.bl.attach(THRUSTER_BL_PIN);
  thrusters.br.attach(THRUSTER_BR_PIN);

  bar02_setup();
  BNO08x_setup();
  
  Serial.setTimeout(10);
  Serial.flush();
}

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
  while(!imu.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
    Serial.println("Failed to enable BNO08x reports");
    delay(1000);
  }
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
  while(Serial.peek() != '$') {
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
      //drain_serial_input();
      return;
    }
    Serial.read();
    Serial.readBytes(sig, 5);
    if(strcmp(sig, "RPCTL") != 0) {
      Serial.print("ERRSIG ");
      Serial.println(sig);
      //drain_serial_input();
      return;
    }
    Serial.readBytes((char*)&input_data, sizeof(input_data));
    if(Serial.peek() != '*') {
      Serial.print("ERRTRM ");
      Serial.println(Serial.read());
      //drain_serial_input();
      return;
    }
    Serial.read();
    Serial.readBytes(checksum_char, 2);
    if(!nmea_checksum()) {
      Serial.print("ERRCHK\n");
      //drain_serial_input();
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
  if(input_data.BTN_EAST) {
    if(depth_hold_btn_avl) {
      depth_hold = !depth_hold;
      stabilize = false;
    }
    depth_hold_btn_avl = false;
  } else {
    depth_hold_btn_avl = true;
  }
  if(input_data.ABS_HAT0X == -1) {
    if(gain_dec_btn_avl) {
      if(input_data.ABS_LT > 100) {
        if(SLOW_MULT >= 0.2) {
          SLOW_MULT -= 0.1;
        }
      } else {
        if(DEFAULT_MULT >= 0.2) {
          DEFAULT_MULT -= 0.1;
        }
      }
    }
    gain_dec_btn_avl = false;
  } else {
    gain_dec_btn_avl = true;
  }
  if(input_data.ABS_HAT0X == 1) {
    if(gain_inc_btn_avl) {
      if(input_data.ABS_LT > 100) {
        if(SLOW_MULT <= 0.9) {
          SLOW_MULT += 0.1;
        }
      } else {
        if(DEFAULT_MULT <= 0.9) {
          DEFAULT_MULT += 0.1;
        }
      }
    }
    gain_inc_btn_avl = false;
  } else {
    gain_inc_btn_avl = true;
  }
  if(input_data.BTN_WEST) {
    if(stabilize_btn_avl) {
      stabilize = !stabilize;
    }
    stabilize_btn_avl = false;
  } else {
    stabilize_btn_avl = true;
  }
  if(slowmode) {
    mult = SLOW_MULT;
  } else {
    mult = DEFAULT_MULT;
  }
}

inline void read_pressure_data() {
  // #define pconv ((double)200/250)
  // aux_data.pressure_mbar = (double)input_data.ABS_RT;
  bar02_sensor.read();
  aux_data.depth = bar02_sensor.depth();
  aux_data.pressure_mbar = bar02_sensor.pressure();
}

// https://github.com/adafruit/Adafruit_BNO08x/blob/master/examples/quaternion_yaw_pitch_roll/quaternion_yaw_pitch_roll.ino
inline void quaternionToEuler(float qr, float qi, float qj, float qk) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    orientation.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    orientation.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    orientation.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    orientation.yaw *= RAD_TO_DEG;
    orientation.pitch *= RAD_TO_DEG;
    orientation.roll *= RAD_TO_DEG;
}

inline void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k);
}

inline void read_imu_data() {
  //#define rconv ((double)150/JOYSTICK_MAGNITUDE)
  //aux_data.roll = input_data.ABS_LX * rconv;
  if(imu.getSensorEvent(&sensor_value)) {
    quaternionToEulerRV(&sensor_value.un.arvrStabilizedRV);
  }
  yaw_abs_cur = ((int32_t)orientation.yaw + 360) % 360;
}

inline void calc_vert_power() {
  int32_t unscaled_vert_power;
  if(!depth_hold || abs(input_data.ABS_RY) > 150) {
    depth_set_avl = true;
    unscaled_vert_power = NORMALIZE_JOYSTICK(input_data.ABS_RY) * mult * THRUSTER_VERT_DIR;
  } else if(depth_hold) {
    if(depth_set_avl) {
      depth_set_avl = false;
      depth_hold_controller.set_target(aux_data.pressure_mbar);
      Serial.print("--------------> Depth set: ");
      Serial.println(depth_hold_controller.target);
    }
    unscaled_vert_power = (depth_hold_controller.compute_PIDF(aux_data.pressure_mbar) * (double)THRUSTER_VERT_DIR * (double)PIDF_REVERSE);
    if(unscaled_vert_power > ESC_MAGNITUDE) {
      unscaled_vert_power = ESC_MAGNITUDE;
    } else if(unscaled_vert_power < -ESC_MAGNITUDE) {
      unscaled_vert_power = -ESC_MAGNITUDE;
    }
  }
  control_data.lvert_power = unscaled_vert_power;
  control_data.rvert_power = unscaled_vert_power;
}

inline void calc_trans_rot_power() {
  int32_t unscaled_fl_power;
  int32_t unscaled_fr_power;
  int32_t unscaled_bl_power;
  int32_t unscaled_br_power;
  double normalize = 1;
  if(!stabilize || abs(input_data.ABS_LX) > DEADZONE_CONST || abs(input_data.ABS_LY) > DEADZONE_CONST || abs(input_data.ABS_RX) > DEADZONE_CONST || abs(input_data.ABS_RY) > 2000 || input_data.ABS_HAT0X != 0 || input_data.ABS_HAT0Y != 0) {
    yaw_set_avl = true;
    unscaled_fl_power = NORMALIZE_JOYSTICK(input_data.ABS_LY) - (int32_t)(STRAFE_GAIN * (double)NORMALIZE_JOYSTICK(input_data.ABS_LX)) - (int32_t)(ROT_GAIN*(double)NORMALIZE_JOYSTICK(input_data.ABS_RX));
    unscaled_fr_power = NORMALIZE_JOYSTICK(input_data.ABS_LY) + (int32_t)(STRAFE_GAIN * (double)NORMALIZE_JOYSTICK(input_data.ABS_LX)) + (int32_t)(ROT_GAIN*(double)NORMALIZE_JOYSTICK(input_data.ABS_RX));
    unscaled_bl_power = NORMALIZE_JOYSTICK(input_data.ABS_LY) - (int32_t)(STRAFE_GAIN * (double)NORMALIZE_JOYSTICK(input_data.ABS_LX)) + (int32_t)(ROT_GAIN*(double)NORMALIZE_JOYSTICK(input_data.ABS_RX));
    unscaled_br_power = NORMALIZE_JOYSTICK(input_data.ABS_LY) + (int32_t)(STRAFE_GAIN * (double)NORMALIZE_JOYSTICK(input_data.ABS_LX)) - (int32_t)(ROT_GAIN*(double)NORMALIZE_JOYSTICK(input_data.ABS_RX));
    if ((NORMALIZE_JOYSTICK(abs(input_data.ABS_LY)) + NORMALIZE_JOYSTICK(abs(input_data.ABS_LX)) + NORMALIZE_JOYSTICK(abs(input_data.ABS_RX))) > ESC_MAGNITUDE) {
      normalize = (NORMALIZE_JOYSTICK(abs(input_data.ABS_LY)) + NORMALIZE_JOYSTICK(abs(input_data.ABS_LX)) + NORMALIZE_JOYSTICK(abs(input_data.ABS_RX)))/(double)ESC_MAGNITUDE;
    }
    normalize /= mult;
    unscaled_fl_power /= normalize;
    unscaled_fr_power /= normalize;
    unscaled_bl_power /= normalize;
    unscaled_br_power /= normalize;
    control_data.fl_power = unscaled_fl_power * THRUSTER_FL_DIR;
    control_data.fr_power = unscaled_fr_power * THRUSTER_FR_DIR;
    control_data.bl_power = unscaled_bl_power * THRUSTER_BL_DIR;
    control_data.br_power = unscaled_br_power * THRUSTER_BR_DIR;
  } else if(stabilize) {
    if(yaw_set_avl) {
      yaw_set_avl = false;
      yaw_hold_controller.set_target(0);
      yaw_abs_target = ((int32_t)orientation.yaw + 360) % 360;
    }
    yaw_rel_offset = (int32_t)((yaw_abs_target - yaw_abs_cur + 540)%360)-180;
    int32_t pwr = yaw_hold_controller.compute_PIDF(yaw_rel_offset);
    control_data.fl_power = pwr * THRUSTER_FL_DIR * PIDF_REVERSE;
    control_data.fr_power = pwr * THRUSTER_FR_DIR;
    control_data.bl_power = pwr * THRUSTER_BL_DIR;
    control_data.br_power = pwr * THRUSTER_BR_DIR * PIDF_REVERSE;
  }
}

inline double vert_amps() {
  return PWR_TO_AMPS(control_data.lvert_power) + PWR_TO_AMPS(control_data.rvert_power);
}

inline double trans_rot_amps() {
  return PWR_TO_AMPS(control_data.fl_power) + PWR_TO_AMPS(control_data.fr_power) + PWR_TO_AMPS(control_data.bl_power) + PWR_TO_AMPS(control_data.br_power);
}

inline void limit_current() {
  if(vert_amps() + trans_rot_amps() <= AMP_LIMIT) {
    return;
  }
  // if both are over their respective limits, bring both down to respective _AMP_LIMITs
  if(trans_rot_amps() > TRANS_ROT_AMP_LIMIT && vert_amps() > VERT_AMP_LIMIT) {
    double vert_amp_multiplier = VERT_AMP_LIMIT / vert_amps();
    control_data.lvert_power = (int32_t)((double)control_data.lvert_power * vert_amp_multiplier);
    control_data.rvert_power = (int32_t)((double)control_data.rvert_power * vert_amp_multiplier);
    double trans_rot_amp_multiplier = TRANS_ROT_AMP_LIMIT / trans_rot_amps();
    control_data.fl_power = (int32_t)((double)control_data.fl_power * trans_rot_amp_multiplier);
    control_data.fr_power = (int32_t)((double)control_data.fr_power * trans_rot_amp_multiplier);
    control_data.bl_power = (int32_t)((double)control_data.bl_power * trans_rot_amp_multiplier);
    control_data.br_power = (int32_t)((double)control_data.br_power * trans_rot_amp_multiplier);
  } 
  // trans rot amps are within limit, bring vertical amps down to overall AMP_LIMIT
  else if(vert_amps > VERT_AMP_LIMIT) {
    double vert_amp_multiplier = (AMP_LIMIT - trans_rot_amps()) / vert_amps();
    control_data.lvert_power = (int32_t)((double)control_data.lvert_power * vert_amp_multiplier);
    control_data.rvert_power = (int32_t)((double)control_data.rvert_power * vert_amp_multiplier);
  }
  // vertical amps are within limit, bring trans rot amps down to overall AMP_LIMIT
  else if(trans_rot_amps() > TRANS_ROT_AMP_LIMIT) {
    double trans_rot_amp_multiplier = (AMP_LIMIT - vert_amps()) / trans_rot_amps();
    control_data.fl_power = (int32_t)((double)control_data.fl_power * trans_rot_amp_multiplier);
    control_data.fr_power = (int32_t)((double)control_data.fr_power * trans_rot_amp_multiplier);
    control_data.bl_power = (int32_t)((double)control_data.bl_power * trans_rot_amp_multiplier);
    control_data.br_power = (int32_t)((double)control_data.br_power * trans_rot_amp_multiplier);
  }
}

inline void power_thrusters() {
  thrusters.lvert.writeMicroseconds(THRUSTER_POWER(control_data.lvert_power));
  thrusters.rvert.writeMicroseconds(THRUSTER_POWER(control_data.rvert_power));
  thrusters.fl.writeMicroseconds(THRUSTER_POWER(control_data.fl_power));
  thrusters.fr.writeMicroseconds(THRUSTER_POWER(control_data.fr_power));
  thrusters.bl.writeMicroseconds(THRUSTER_POWER(control_data.bl_power));
  thrusters.br.writeMicroseconds(THRUSTER_POWER(control_data.br_power));
}

inline void elim_deadzones() {
  // depends on controller
  if(abs(input_data.ABS_LX) < DEADZONE_CONST) {
    input_data.ABS_LX = 0;
  }
  if(abs(input_data.ABS_LY) < DEADZONE_CONST) {
    input_data.ABS_LY = 0;
  }
  if(abs(input_data.ABS_RX) < DEADZONE_CONST) {
    input_data.ABS_RX = 0;
  }
  if(abs(input_data.ABS_RY) < DEADZONE_CONST) {
    input_data.ABS_RY = 0;
  }
}

inline void transmit_rov_data() {
  Serial.print("$ARSLV");
  /*
  int64_t* input_iter = &input_data;
  while((uint64_t)input_iter < &input_data + sizeof(input_data)) {
    Serial.print(",");
    Serial.print(*input_iter);
    input_iter++;
  }*/
  Serial.print(",");
  Serial.print(control_data.lvert_power);
  Serial.print(",");
  Serial.print(control_data.rvert_power);
  Serial.print(",");
  Serial.print(control_data.fl_power);
  Serial.print(",");
  Serial.print(control_data.fr_power);
  Serial.print(",");
  Serial.print(control_data.bl_power);
  Serial.print(",");
  Serial.print(control_data.br_power);
  Serial.print(",");
  Serial.print(aux_data.depth);
  Serial.print(",");
  Serial.print(aux_data.pressure_mbar);
  Serial.print(",");
  Serial.print(orientation.yaw);
  Serial.print(",");
  Serial.print(orientation.pitch);
  Serial.print(",");
  Serial.print(orientation.roll);
  Serial.print(",");
  Serial.print(slowmode);
  Serial.print(",");
  Serial.print(depth_hold);
  Serial.print(",");
  Serial.print(depth_hold_controller.target);
  Serial.print(",");
  Serial.print(stabilize);
  Serial.print(",");
  Serial.print(yaw_abs_cur);
  Serial.print(",");
  Serial.print(yaw_abs_target);
  Serial.print(",");
  Serial.print(yaw_rel_offset);
  Serial.print(",");
  Serial.print(DEFAULT_MULT);
  Serial.print(",");
  Serial.print(SLOW_MULT);
  Serial.print(",");
  // throaway checksum
  Serial.println("*FF");
}

void loop() {
  read_serial_input();
  elim_deadzones();
  set_consts();
  read_pressure_data();
  read_imu_data();
  calc_vert_power();
  calc_trans_rot_power();
  limit_current();
  power_thrusters();
  if(prog_iter % SERIAL_TRANSMISSION_WRAP == 0) {
    transmit_rov_data();
  }
  prog_iter++;
}