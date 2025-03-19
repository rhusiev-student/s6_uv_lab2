#include <Arduino.h>
#include "gamepad.hpp"
#include "motor.hpp"

const uint8_t single_delay = 2;
const uint8_t move_delays = 20 / single_delay;
const uint8_t update_delays = 50 / single_delay;
const uint8_t servo_delays_initial = 4 / single_delay;
uint8_t servo_delays_hor = servo_delays_initial;
uint8_t servo_delays_ver = servo_delays_initial;
const int32_t controller_max = 512;

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

const int32_t controller_max_cube = controller_max * controller_max * controller_max;
const int32_t motor_max = 2000;
const int32_t motor_min = 600;
const int32_t motor_start_movement = 200;

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
  // Serial.printf("Motor: %d %d %d %d (forward %d left %d)\n", movement[0], movement[1], movement[2], movement[3], forward, left);
}

int32_t angle_vertical = 90;
int32_t angle_horizontal = 90;
const int32_t max_servo_speed = 20;
const int32_t servo_speed_bump = 19;
int32_t to_servo_speed(int32_t speed, bool hor, int32_t i) {
  auto diff = max_servo_speed * speed / controller_max;
  if (diff == 0) {
    if (hor) {
      servo_delays_hor = servo_delays_initial;
    } else {
      servo_delays_ver = servo_delays_initial;
    }
    // Serial.printf(", Delay_initial");
    // Serial.printf(", Delta: %d\n", 0);
    return 0;
  }
  // Serial.printf("Diff: %d", diff);
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
      // Serial.printf(", Delay_hor: %d", servo_delays_hor);
    } else {
      servo_delays_ver = servo_delays_initial * (servo_speed_bump - abs_diff);
      // Serial.printf(", Delay_ver: %d", servo_delays_ver);
    }
    // Serial.printf(", Delta: %d\n", sign);
    return sign;
  }
  if (hor) {
    servo_delays_hor = servo_delays_initial;
    // Serial.printf(", Delay_hor: %d", servo_delays_hor);
  } else {
    servo_delays_ver = servo_delays_initial;
    // Serial.printf(", Delay_ver: %d", servo_delays_ver);
  }
  // Serial.printf(", Delta: %d\n", diff - sign * (servo_speed_bump - 1));
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
    // Serial.println();
    if (angle_vertical > 150)
      angle_vertical = 150;
    if (angle_vertical < 90)
      angle_vertical = 90;
  }
  // Serial.printf("Moving to %d %d  d(%d %d)\n", angle_horizontal, angle_vertical, to_servo_speed(left), to_servo_speed(up));

  Servo_1_Angle(angle_horizontal);
  Servo_2_Angle(angle_vertical);
}

void reset() {
  Motor_Move(0, 0, 0, 0);
  Servo_1_Angle(90);
  Servo_2_Angle(90);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);


  PCA9685_Setup();
  reset();
}

uint8_t i = 0;
bool dataUpdated = false;
void loop() {
  i++; // Will wrap to 0 after 255
  delay(single_delay);
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  if (i % update_delays == 0) {
    dataUpdated = BP32.update();
    // Serial.printf("Servo: %d %d\n", angle_horizontal, angle_vertical);
  }
  if (!dataUpdated) {
    return;
  }
  if (!(myController && myController->isConnected() && myController->hasData())) {
    return;
  }
  if (!myController->isGamepad()) {
    Serial.println("Unsupported controller");
    return;
  }

  if (i % update_delays == 0) {
    processGamepad(myController);
    // dumpGamepad(myController);
  }
  if (i % move_delays == 0) {
    move_frame(myController);
  }
  move_servo(myController, i);

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
}
