#include <iostream>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <cmath>
#define TIME_STEP 32

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  DistanceSensor *ds[5];
  char dsNames[5][10] = {"ds_left","ds_right","ds_front","ds_left1","ds_right1"};
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  for (int i=0; i<5; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  
  }
  Motor *wheels[2];
  char wheels_names[2][10] = {"l_wh","r_wh"};
  
  for (int i =0;i<2 ; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  
  }
  
  
  
  //std::cout << ds[0]->getValue() <<" " << ds[1]-ds[0] << "  " << ds[3] << std::endl;

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  double Kp = 0.1;
  double Kd = 0.3;
  double maxSpeed = 5;
  double lastError = 0.0;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  
  
    double leftVal = ds[0]->getValue();
    double rightVal = ds[1]->getValue();
  //  double frontVal = ds[2]->getValue();
    double leftVal1 = ds[3]->getValue();
    double rightVal1 = ds[4]->getValue();
    
    double rightIr=(0.1594*pow(rightVal,-0.8533)-0.02916)*100;
    double rightIr1=(0.1594*pow(rightVal1,-0.8533)-0.02916)*100;
    double leftIr=(0.1594*pow(leftVal,-0.8533)-0.02916)*100;
    double leftIr1=(0.1594*pow(leftVal1,-0.8533)-0.02916)*100;
    
    
    std::cout <<"rightIr  "<< rightIr <<"  rightIr1  "<< rightIr1 <<"  leftIr   "<< leftIr <<"   leftIr1   "<< leftIr1    << std::endl;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    if (leftIr<37 && rightIr>37){
      double efferror = (leftIr-leftIr1)*10;
      double efferror1 = (leftIr-rightIr+leftIr1-rightIr1);
 
      if(leftIr<5){
      //  wheels[0]->setVelocity((maxSpeed+1));
      //  wheels[1]->setVelocity((maxSpeed-1));
      }else{
        double adjust = Kp*efferror + Kd*(efferror-lastError);
        lastError = efferror;
 
     
       // wheels[0]->setVelocity((maxSpeed-adjust));
       // wheels[1]->setVelocity((maxSpeed+adjust));
      }
    }
     else if(rightIr<37 && leftIr>37){
        double efferror = (rightIr-rightIr1)*10;
        double efferror1 = (leftIr-rightIr+leftIr1-rightIr1);
   
        if(rightIr<5){
        //  wheels[0]->setVelocity((maxSpeed-1));
         // wheels[1]->setVelocity((maxSpeed+1));
        }else{
          double adjust = Kp*efferror + Kd*(efferror-lastError);
          lastError = efferror;
     
        //  wheels[0]->setVelocity((maxSpeed-adjust));
        //  wheels[1]->setVelocity((maxSpeed+adjust));
      }}else{
      double efferror1 = (leftIr-rightIr+leftIr1-rightIr1);
      double adjust = Kp*efferror1 + Kd*(efferror1-lastError);
      lastError = efferror1;
     
     // wheels[0]->setVelocity((maxSpeed+adjust));
     // wheels[1]->setVelocity((maxSpeed-adjust));
      }
    
    }
    

  delete robot;
  return 0;
}
