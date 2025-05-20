/* #include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  // Get device tags
  WbDeviceTag left_motor = wb_robot_get_device("motor1");
  WbDeviceTag right_motor = wb_robot_get_device("motor2");

  // Set motor target position to infinity (velocity control mode)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Set initial velocities
  wb_motor_set_velocity(left_motor, 1.0);
  wb_motor_set_velocity(right_motor, 1.0);

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // You can add more control logic here
  }

  wb_robot_cleanup();
  return 0;
}

*/





#include <webots/robot.h>
#include <webots/motor.h>
//80, 50
#define TIME_STEP 32
#define FORWARD_DURATION 105   // Adjust based on robot speed (approx. 2.5 sec)
#define TURN_DURATION 50      // Adjust for 90 degree turn (approx. 1.25 sec)
#define MAX_SPEED 6.28       // Change if your robot has a different max speed

int main() {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("motor2");
  WbDeviceTag right_motor = wb_robot_get_device("motor1");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  int state = 0;       // 0: move forward, 1: turn
  int step_counter = 0;
  int side = 0;        // Number of sides completed

  while (wb_robot_step(TIME_STEP) != -1) {
    if (side >= 4) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break;
    }

    if (state == 0) {
      // Move forward
      wb_motor_set_velocity(left_motor, -3.0);
      wb_motor_set_velocity(right_motor, -3.0);
      step_counter++;
      if (step_counter >= FORWARD_DURATION) {
        step_counter = 0;
        state = 1;
      }
    } else if (state == 1) {
      // Turn in place (90 degrees)
      wb_motor_set_velocity(left_motor, 2.0);//-2.0
      wb_motor_set_velocity(right_motor, -2.0);
      step_counter++;
      if (step_counter >= TURN_DURATION) {
        step_counter = 0;
        state = 0;
        side++;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
