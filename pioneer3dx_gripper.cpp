/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller moving the Pioneer3DX and its gripper.
 */
 
 // Team 4: Akashleena Sarkar, Tabea Voß

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#define GRIPPER_MOTOR_MAX_SPEED 0.1

using namespace webots;

Robot *robot = new Robot();
Motor *wheel_motors[3];
Motor *gripper_motors[3];
static int time_step = 0;

static void initialize() {
  /* necessary to initialize Webots */

  time_step = robot->getBasicTimeStep();

  gripper_motors[0] = robot->getMotor("lift motor");
  gripper_motors[1] = robot->getMotor("left finger motor");
  gripper_motors[2] = robot->getMotor("right finger motor");
  wheel_motors[0] = robot->getMotor("left wheel");
  wheel_motors[1] = robot->getMotor("right wheel");
  
  // Initialize position and velocity of motors here
  gripper_motors[0] -> setVelocity(0);
  gripper_motors[0] -> setPosition(INFINITY);
  
  gripper_motors[1] -> setVelocity(0);
  gripper_motors[1] -> setPosition(INFINITY);
  
  gripper_motors[2] -> setVelocity(0);
  gripper_motors[2] -> setPosition(INFINITY);
  
  wheel_motors[0] -> setVelocity(0);
  wheel_motors[0] -> setPosition(INFINITY);
  
  wheel_motors[1] -> setVelocity(0);
  wheel_motors[1] -> setPosition(INFINITY);
}

// Function to update the motors and sensors of the robot. This should be called everytime after specifying any type of robot movement.
void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    robot->step(time_step);
    elapsed_time += time_step;
  }
}

// Function to move the gripper up/down
void lift(double position) {
  gripper_motors[0] -> setVelocity(GRIPPER_MOTOR_MAX_SPEED);
  gripper_motors[0] -> setPosition(position);
}

// Write a function here to specify how to open/close the gripper fingers
void open_close(double position) {
  gripper_motors[1] -> setVelocity(GRIPPER_MOTOR_MAX_SPEED);
  gripper_motors[1] -> setPosition(position);
  gripper_motors[2] -> setVelocity(GRIPPER_MOTOR_MAX_SPEED);
  gripper_motors[2] -> setPosition(position);
}
// Write functions here to specify how to make the robot move forwards/backwards, turn left/right and stop the robot 
void move(double speed) {
  wheel_motors[0] -> setVelocity(speed);
  wheel_motors[1] -> setVelocity(speed);
}

void back(double speed) {
  wheel_motors[0] -> setVelocity(-speed);

  wheel_motors[1] -> setVelocity(-speed);

}

void left(double speed) {
  wheel_motors[0] -> setVelocity(-speed); //left
  wheel_motors[1] -> setVelocity(speed); //right
}

void right(double speed) {
  wheel_motors[0] -> setVelocity(speed);
  wheel_motors[1] -> setVelocity(-speed);
}

int main() {
  initialize();
  
  //get blue ball
  
  open_close(.05);
  step(1.5);
  
  move(1);
  step(4);
  move(0);
  step(.5);
  
  lift(0.05);
  step(.5);
  
  open_close(.015);
  step(1.5);
  
  lift(-0.05);
  step(.5);
  
  //put blue ball into blue bin
  
  back(1);
  step(4);
  
  right(0.4);
  step(4);

  move(1);
  step(8);
  move(0);
  step(.5);
  
  lift(0.05);
  step(.5);
  open_close(.05);
  step(1.5);
  lift(-0.05);
  step(.5);

  //get green ball

  back(1);
  step(10);
  move(0);
  step(.5);
  
  left(0.4);
  step(4);
  
  move(1);
  step(5.5);
  move(0);
  step(.5);
  
  lift(0.05);
  step(.5);
  open_close(.015);
  step(1.5);
  lift(-0.05);
  step(.5);
  
  //put green ball into green bin
  
  back(1);
  step(4);
  
  left(0.4);
  step(4);

  move(1);
  step(8);
  move(0);
  step(.5);
  
  lift(0.05);
  step(.5);
  open_close(.05);
  step(1.5);
  lift(-0.05);
  step(.5);
  
  return 0;
}
