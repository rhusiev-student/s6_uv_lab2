#ifndef INCLUDE_MOVEMENT_LOGIC_HPP_
#define INCLUDE_MOVEMENT_LOGIC_HPP_

#include <Arduino.h>
#include "delays.hpp"
#include "gamepad.hpp"
#include "motor.hpp"

const int32_t controller_max = 512;
const int32_t controller_max_cube = controller_max * controller_max * controller_max;

const int32_t motor_max = 2000;
const int32_t motor_min = 600;
const int32_t motor_start_movement = 200;

int32_t to_motor_speed_cube(int32_t speed);
int32_t to_motor_speed(int32_t speed);

void move_frame(ControllerPtr ctl);

const int32_t max_servo_speed = 20;
const int32_t servo_speed_bump = 19;

int32_t to_servo_speed(int32_t speed, bool hor, int32_t i);

void move_servo(ControllerPtr ctl, int32_t i);

#endif // INCLUDE_MOVEMENT_LOGIC_HPP_