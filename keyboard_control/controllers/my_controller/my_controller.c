#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

#define TIME_STEP 32
#define SPEED 3.0

int main() {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("motor1");
  WbDeviceTag right_motor = wb_robot_get_device("motor2");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  wb_keyboard_enable(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();

    double left_speed = 0.0;
    double right_speed = 0.0;

    switch (key) {
      case 'W':  // Forward
        left_speed = -SPEED;
        right_speed = -SPEED;
        break;
      case 'S':  // Backward
        left_speed = SPEED;
        right_speed = SPEED;
        break;
      case 'A':  // Turn left
        left_speed = -SPEED;
        right_speed = SPEED;
        break;
      case 'D':  // Turn right
        left_speed = SPEED;
        right_speed = -SPEED;
        break;
      case ' ':  // Spacebar = stop
        left_speed = 0.0;
        right_speed = 0.0;
        break;
      default:
        // No key or unhandled key: do nothing (maintain last speed)
        break;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}

