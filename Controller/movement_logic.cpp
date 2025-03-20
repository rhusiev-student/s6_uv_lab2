#include "movement_logic.hpp"

int32_t to_motor_speed_cube(int32_t speed) {
  auto motor_speed = (motor_max - motor_min) * speed / controller_max * speed / controller_max * speed / controller_max;
  if (motor_speed > motor_start_movement) {
    motor_speed += motor_min;
  } else if (motor_speed < -motor_start_movement) {
    motor_speed -= motor_min;
  }
  return motor_speed;
}
int32_t to_motor_speed(int32_t speed) {
  auto motor_speed = (motor_max - motor_min - 600) * speed / controller_max;
  if (motor_speed > motor_start_movement) {
    motor_speed += motor_min + 600;
  } else if (motor_speed < -motor_start_movement) {
    motor_speed -= motor_min + 600;
  }
  return motor_speed;
}

void move_frame(ControllerPtr ctl) {
  auto forward = to_motor_speed_cube(-ctl->axisY());
  auto left = to_motor_speed(ctl->axisX());

  int movement[4];
  int32_t right;
  if (left > 0) {
    right = -left / 4;
    left = left * 3 / 4;
  } else {
    right = -left * 3 / 4;
    left = left / 4;
  }
  movement[0] += forward + left;
  movement[1] += forward + left;
  movement[2] += forward + right;
  movement[3] += forward + right;
  Motor_Move(movement[0], movement[1], movement[2], movement[3]);
}

int32_t angle_vertical = 90;
int32_t angle_horizontal = 90;

uint8_t servo_delays_hor = servo_delays_initial;
uint8_t servo_delays_ver = servo_delays_initial;

int32_t to_servo_speed(int32_t speed, bool hor, int32_t i) {
  auto diff = max_servo_speed * speed / controller_max;
  if (diff == 0) {
    if (hor) {
      servo_delays_hor = servo_delays_initial;
    } else {
      servo_delays_ver = servo_delays_initial;
    }
    return 0;
  }
  int32_t abs_diff;
  int32_t sign;
  if (diff > 0) {
    abs_diff = diff;
    sign = 1;
  } else {
    abs_diff = -diff;
    sign = -1;
  }
  if (abs_diff < servo_speed_bump) {
    if (hor) {
      servo_delays_hor = servo_delays_initial * (servo_speed_bump - abs_diff);
    } else {
      servo_delays_ver = servo_delays_initial * (servo_speed_bump - abs_diff);
    }
    return sign;
  }
  if (hor) {
    servo_delays_hor = servo_delays_initial;
  } else {
    servo_delays_ver = servo_delays_initial;
  }
  return diff - sign * (servo_speed_bump - 1);
}

void move_servo(ControllerPtr ctl, int32_t i) {
  auto up = -ctl->axisRY();
  auto left = -ctl->axisRX();
  if (i % servo_delays_hor == 0) {
    angle_horizontal += to_servo_speed(left, true, i);
    if (angle_horizontal > 170)
      angle_horizontal = 170;
    if (angle_horizontal < 0)
      angle_horizontal = 0;
  }
  if (i % servo_delays_ver == 0) {
    angle_vertical += to_servo_speed(up, false, i);
    if (angle_vertical > 150)
      angle_vertical = 150;
    if (angle_vertical < 90)
      angle_vertical = 90;
  }

  Servo_1_Angle(angle_horizontal);
  Servo_2_Angle(angle_vertical);
}