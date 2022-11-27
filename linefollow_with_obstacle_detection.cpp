#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#define TIME_STEP 64 
#define MAX_SPEED 2

using namespace webots;

/*
Concept: If left IR sensor detects a line turn left, if right IR sensor detects a line
turn right.
Author : Akashleena & Tabea

*/

int main(int argc, char **argv) {

  // Create objects of robot, wheel rotary motors, cam
 
  Robot *robot = new Robot();

  Motor *left_front_motor = robot->getMotor("left_front_wheel");
  Motor *right_front_motor = robot->getMotor("right_front_wheel");
  Motor *left_rear_motor = robot->getMotor("left_rear_wheel");
  Motor *right_rear_motor = robot->getMotor("right_rear_wheel");
  Camera *cam = robot->getCamera("cam");

  // Set wheel motors to INF position; necessary for giving velocity
  
  left_front_motor -> setPosition(INFINITY);
  right_front_motor -> setPosition(INFINITY);
  left_rear_motor -> setPosition(INFINITY);
  right_rear_motor -> setPosition(INFINITY); 
  
  // All wheels have 0 vel - so stops
  
  left_front_motor -> setVelocity(0.0);
  right_front_motor -> setVelocity(0.0);
  left_rear_motor -> setVelocity(0.0);
  right_rear_motor -> setVelocity(0.0);
  
  // Create objects for Infrared and Distance Sensors
  
  DistanceSensor *ir_right = robot->getDistanceSensor("infrared_sensor_right");
  DistanceSensor *ir_mid = robot->getDistanceSensor("infrared_sensor_mid");
  DistanceSensor *ir_left = robot->getDistanceSensor("infrared_sensor_left");
  DistanceSensor *ds_right = robot->getDistanceSensor("distance_sensor_right");
  DistanceSensor *ds_left = robot->getDistanceSensor("distance_sensor_left");
  
  // Enable sensors to read values
  
  ir_right->enable(TIME_STEP);
  ir_mid->enable(TIME_STEP);
  ir_left->enable(TIME_STEP);
  ds_right->enable(TIME_STEP);
  ds_left->enable(TIME_STEP);
  
  // Initialize variables to save sensor outputs
  
  double ir_right_val = ir_right->getValue();
  double ir_mid_val = ir_mid->getValue();
  double ir_left_val = ir_left->getValue();
    
  double ds_right_val = ds_right->getValue();
  double ds_left_val = ds_left->getValue();
  
  // Enable cam and get image shown on top left corner
  
  cam->enable(TIME_STEP);
  cam->getImage();
  
  //give both side wheels some initial value
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;
 
 // Function to move robot based on the input wheel velocities
  
  while (robot->step(TIME_STEP) != -1) {
  
    // Get updated values of sensors on each iteration
    std::cout<<"while loop "<<std::endl;
    ir_right_val = ir_right->getValue();
    ir_mid_val = ir_mid->getValue();
    ir_left_val = ir_left->getValue();
    
    std::cout<<"ir-right : "<<ir_right_val<<std::endl;
    std::cout<<"ir-mid : "<<ir_mid_val<<std::endl;
    std::cout<<"ir-left : "<<ir_left_val<<std::endl;
    
    ds_right_val = ds_right->getValue();
    ds_left_val = ds_left->getValue();
    
    std::cout<<"right distance : "<< ds_right_val <<std::endl;
    std::cout<<"left distance : "<< ds_left_val <<std::endl;
    
    //By default give both side wheels max speed
    left_speed = MAX_SPEED;
    right_speed = MAX_SPEED;
    
    //Process sensor values: detect if line is curving
    // presumtion: line values range from 600 to 800
    bool line_left = (600 < ir_left_val) && (ir_left_val < 800);
    bool line_right = (600 < ir_right_val) && (ir_right_val < 800);
   
    // in case the line curves to the left
    if((ir_left_val > ir_right_val) && line_left)
    {
      left_speed = MAX_SPEED * 0.1;
    }
    
    // in case the line curves to the right
    else if((ir_right_val > ir_left_val) && line_right)
    {
      right_speed = MAX_SPEED * 0.1;
    }
  
    //Stop the robot if line ends
    if ((ir_right_val > 12.0) && (ir_right_val < 15.0)&&
        (ir_mid_val > 12.0) && (ir_mid_val < 15.0) &&
        (ir_left_val > 12.0) && (ir_left_val < 15.0))
     {
       left_speed = 0;
       right_speed = 0;
     } 
     
    //stop infront of object
    if((ds_right_val <= 90)||(ds_left_val <= 90))
     {
       left_speed = 0;
       right_speed = 0;
     }
     
    left_front_motor -> setVelocity(left_speed);
    right_front_motor -> setVelocity(right_speed);
    left_rear_motor -> setVelocity(left_speed);
    right_rear_motor -> setVelocity(right_speed);
     
  }
  
  
  delete robot;
  return 0;

}
