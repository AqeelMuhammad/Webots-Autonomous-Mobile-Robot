#include <iostream>
#include <sstream>
#include <map>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Display.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 16
using namespace webots;

// define some global variables

static int      lastError   = 0;
static int      I           = 0;
static double   leftSpeed;
static double   rightSpeed;
static int      turnTime;
static int      lineCutoff = 500;

// ------------------------------------------------------------------------------------------------

// limit the velocity of the wheels
double limit(double &val){
  if (val > 15){
    return 15;
  }else if(val < -15){
    return -15;
  }else{
    return val;
  }
}
// ------------------------------------------------------------------------------------------------

// line following function
void lineFollow(DistanceSensor* ir[],double &kp,double &kd,double &ki,double &baseSpeed){
  int error = 0; 
  for (int i=0;i<8;i++){
    if (ir[i]->getValue()<lineCutoff){
      if (i<4){
        error += 100;
      }else{
        error -= 100;
      }
    }
  }
  int P = error;
  int D = error - lastError;
  I = I + error;
  lastError = error;
  if (error == 0){
    I = 0;
  }
  double correction = ( kd*D + kp*P + ki*I )/80.0;  
  leftSpeed = baseSpeed - correction;      leftSpeed = limit(leftSpeed);
  rightSpeed = baseSpeed + correction;     rightSpeed = limit(rightSpeed);
  return;
}
// ------------------------------------------------------------------------------------------------

// wallfollowing function
void wallFollow(int &ds,double &kp,double &kd,double &ki,double &baseSpeed, int direction){
  double error = ds - 1150;   
  int P = error;
  int D = error - lastError;
  I = I + error;
  lastError = error;
  if (error == 0){
    I = 0;
  }
  double correction = ( kd*D + kp*P + ki*I)/200.0;
  if (!direction){
    correction = -correction;
  }
  leftSpeed = baseSpeed + correction;      leftSpeed = limit(leftSpeed);
  rightSpeed = baseSpeed - correction;     rightSpeed = limit(rightSpeed);
  return;
}
// ------------------------------------------------------------------------------------------------

// left turning function
void turnLeft(){
  turnTime--;
  leftSpeed  = 0;
  rightSpeed = 15;
  return;
}
// ------------------------------------------------------------------------------------------------

// Right turning function
void turnRight(){
  turnTime--;
  leftSpeed  = 15;
  rightSpeed = 0;
  return;
}
// ------------------------------------------------------------------------------------------------

// Go forward without turning function
void noTurn(){
  turnTime   = turnTime-2;
  leftSpeed  = 8;
  rightSpeed = 8;
  return;
}
// ------------------------------------------------------------------------------------------------

// Immidiate right turning function
void turnRightI(){
  turnTime--;
  leftSpeed  = 8;
  rightSpeed = -8;
  return;
}
// ------------------------------------------------------------------------------------------------

// Immidiate left turning function
void turnLeftI(){
  turnTime--;
  leftSpeed  = -8;
  rightSpeed = 8;
  return;
}
// ------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {
  
  Robot *robot = new Robot();
  
  // linefollowing ir sensors
  DistanceSensor *ir[8];
  char irNames[8][5] = {"ir1", "ir2","ir3","ir4","ir5","ir6","ir7","ir8"};
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(irNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  // wallfollowing distance sesors
  DistanceSensor *ds[3];
  char dsNames[3][12] = {"ir_left","ultraSonic","ir_right"};
  for (int i = 0; i<3; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  // junction detecting ir sensors
  DistanceSensor *bottom[4];
  char bottomNames[4][15] = {"bottomLeft","bottomFront","bottomRight","bottomBack"};
  for (int i = 0; i<4; i++){
    bottom[i] = robot->getDistanceSensor(bottomNames[i]);
    bottom[i]->enable(TIME_STEP);
  }
  
  // position sensors of arm motors
  PositionSensor *ps_arm[4];
  char ps_arm_names[4][18] = {"psArmBase","psRightGrab","psLeftGrab","psRightGrip"};
  for (int i = 0; i < 4; i++) {
    ps_arm[i] = robot->getPositionSensor(ps_arm_names[i]);
    ps_arm[i]->enable(TIME_STEP);
  }
  
  // position sensor of a wheel
  PositionSensor *ps[2];
  char ps_names[2][18] = {"psSensorRight","psSensorLeft"};
  for (int i = 0; i < 2; i++) {
    ps[i] = robot->getPositionSensor(ps_names[i]);
  }
  
  // accelerometer for detecting the ramp
  Accelerometer *acc;
  acc = robot->getAccelerometer("accelerometer");
  
  // display to output some informations
  Display *display;
  display = robot->getDisplay("display");
  
  // camera as a colour decting sensor
  Camera *cam;
  cam = robot->getCamera("camera");
  
  // motors of the arm
  Motor *arm[4];
  char arm_names[4][18] = {"ArmBaseMotor","RightGrabMotor","LeftGrabMotor","RightGripMotor"};
  for (int i = 0; i < 4; i++) {
    arm[i] = robot->getMotor(arm_names[i]);
    arm[i]->setPosition(INFINITY);
    arm[i]->setVelocity(0.0);
  }
  
  // motors of wheels
  Motor *wheels[2];
  char wheels_names[2][18] = {"LeftMotor", "RightMotor"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  // define some variables
  
  int starting = 0;               // for the starting
   
  double kp         = 2.71;          // line following parameters
  double kd         = 0.3;
  double ki         = 0.01;
  double baseSpeed  = 8;
  
  turnTime  = 0;                  // variables for taking turns
  int turns = 0;
  
  double start  = 0;              // for calculating the diameter
  double end    = 0;
  double start1 = 0;
  double start2 = 0;
  double end1   = 0;
  double end2   = 0;
  bool   case1  = true;
  bool   case2  = true;
  
  bool case5    = false;          // for ramp detection
  int  duration = 0;
  int  ramp     = 0;
      
  int  poles = 0;                // for counting the poles
  bool case3 = true;
  bool case4 = true;
  
  std::map<std::string,int> colour_mp = {{"Red",1},{"Green",2},{"Blue",3}};             // colour code
  
  int   breakTime = 0;           // for arm movements and colour detection
  int  timeLimit  = 0;
  int  side1      = 0;
  int  side2      = 0;
  int  side3      = 0;
  int  difference = 0;
  bool case6      = false;
  bool case7      = true;
  
  bool break0  = false;         bool step0  = false;                // arm movements which have 11 steps and 11 break times
  bool break1  = false;         bool step1  = false;
  bool break2  = false;         bool step2  = false;
  bool break3  = false;         bool step3  = false;
  bool break4  = false;         bool step4  = false;
  bool break5  = false;         bool step5  = false;
  bool break6  = false;         bool step6  = false;
  bool break7  = false;         bool step7  = false;
  bool break8  = false;         bool step8  = false;
  bool break9  = false;         bool step9  = false;
  bool break10 = false;         bool step10 = false;
  
  bool case8  = true;           // for quadrant number displaying
  bool case9  = true;
  bool case10 = true;
  
  bool case11 = true;           // for gate passing
  bool case12 = false;
  
  bool case13  = true;          // after reverse situation
  bool reverse = false;
  
  bool path1 = true;            // making three paths to get shortest path
  bool path2 = false;
  bool path3 = false;
  
  while (robot->step(TIME_STEP) != -1) {
    
    // distance sensor values
    int ds_left  = ds[0]->getValue();
    int ds_front = ds[1]->getValue();
    int ds_right = ds[2]->getValue();

    // junction detecting ir sensor values
    int leftWay  = bottom[0]->getValue();
    int frontWay = bottom[1]->getValue();
    int rightWay = bottom[2]->getValue();
    int backWay  = bottom[3]->getValue();
      
    leftSpeed  = 8;
    rightSpeed = 8;
    
    // starting time
    if (starting){
      leftSpeed  = 8;
      rightSpeed = 8;
      starting--;
    }
    
    // stop & start conditions
    else if (leftWay<lineCutoff && frontWay<lineCutoff && rightWay<lineCutoff && backWay<lineCutoff){
      if (ramp){
        leftSpeed  = 0;
        rightSpeed = 0;
      }else{
        starting   = 50;
        leftSpeed  = 8;
        rightSpeed = 8;
      }
      
    }else{
      // check junctions
      if (((rightWay<lineCutoff && leftWay<lineCutoff) || (leftWay<lineCutoff && frontWay<lineCutoff) || (rightWay<lineCutoff && frontWay<lineCutoff)) && turnTime==0 && ramp<2 && !case6){
        turnTime = 26;
        turns++;
      }
      // wall at right 
      if (ds_right > 1200 && frontWay>lineCutoff && leftWay>lineCutoff && rightWay>lineCutoff){
        wallFollow(ds_right,kp,kd,ki,baseSpeed,0);
      // wall at left
      }else if (ds_left > 1200 && frontWay>lineCutoff && leftWay>lineCutoff && rightWay>lineCutoff){
        wallFollow(ds_left,kp,kd,ki,baseSpeed,1);    
      }// checked that there are no walls
      else{
        // -----------------------------------------------------------------------------------------------       
        // turns on the ramp

        if (turnTime > 0 && ramp==1 && difference%2){                                                               // junctions to turn left
          turnRight();
          if (turnTime==1){
            baseSpeed = 8;          // slow down on the ramp
          }
        }else if (turnTime > 0 && ramp==1 && !(difference%2)){                                                      // junctions to turn right
          turnLeft();
          if (turnTime==1){
            baseSpeed = 8;
          }
        }else if (turnTime > 0 && ramp == 5){
          noTurn();                                                                                                 // junctions to take no turn
          if (turnTime==1){
            baseSpeed = 8;
          }                                                                                                
        }
        // -----------------------------------------------------------------------------------------------       
        // path 1
        
        else if (turnTime > 0 && (turns==2 || turns==4 || turns==5 || turns==8) && !ramp && path1 && !case6){
          turnLeft();                                                                                               // junctions to turn left
        }else if (turnTime > 0 && (turns==1 || turns==3 || turns==6 || turns==7) && !ramp && path1 && !case6){
          turnRight();                                                                                              // junctions to turn right     
        }
        // -----------------------------------------------------------------------------------------------       
        // path 2
        
        else if (turnTime > 0 && (turns==3 || turns==5) && !ramp && path2 && !case6){
          turnLeft();                                                                                               // junctions to turn left
        }else if (turnTime > 0 && turns==4 && !ramp && path2 && !case6){
          turnRight();                                                                                              // junctions to turn right
        }
        // -----------------------------------------------------------------------------------------------        
        // path 3
        
        else if (turnTime > 0 && (turns==2 || turns==4) && !ramp && path3 && !case6){
          turnLeft();                                                                                               // junctions to turn left
        }else if (turnTime > 0 && (turns==1 || turns==3 || turns==6) && !ramp && path3 && !case6){
          turnRight();                                                                                              // junctions to turn right
        }else if (turnTime > 0 && turns==5 && !ramp && path3 && !case6){
          noTurn();                                                                                                 // junctions to take no turn
        }
        // ----------------------------------------------------------------------------------------------
        
        // junction to turn back
        else if (turnTime > 0 && poles){
          if (difference%2){
            turnLeftI();
            if (turnTime==1){
            baseSpeed = 10;             // speed up for ramp
          }
          }else{
            turnRightI();
            if (turnTime==1){
            baseSpeed = 10;
          }
          }  
        }
        // line following
        else{
          if (!case6){
            turnTime = 0;
          }
          lineFollow(ir,kp,kd,ki,baseSpeed);
        }
      }

// ------------------------------------------------------------------------------------------------------
      // box grabing and colour detection
    
   //-------------------------------------------------------------------------------------------
      // detect the box
      
      if (ds_front<375 && case7){
        // adjusting the distance
        if (ds_front<370){
          leftSpeed  = -1;
          rightSpeed = -1;
        }else{
          case6 = true;
          case7 = false;
          step0 = true;
        }     
      } 

      if (case6){
        // camera enable
        if (breakTime==1){
          cam->enable(TIME_STEP);
        }
        // colour detection
        if (breakTime==18 || breakTime==23 || breakTime==28){
          int red   = 0;
          int green = 0;
          int blue  = 0;
          const unsigned char *image = cam->getImage();
          for (int x = 0; x < 16; x++){
            for (int y = 0; y < 16; y++) {
              red   += cam->imageGetRed(image, 16, x, y);
              green += cam->imageGetGreen(image, 16, x, y);
              blue  += cam->imageGetBlue(image, 16, x, y);
              }
            }
          if (red>blue && red>green){
            if (breakTime==18){
              side1 = colour_mp["Red"];
            }
            if (breakTime==23){
              side2 = colour_mp["Red"];
            }
            if (breakTime==28){
              side3 = colour_mp["Red"];
            }
            std::cout<<"Red - 1"<<"\n";
          }else if (blue>green){
            if (breakTime==18){
              side1 = colour_mp["Blue"];
            }
            if (breakTime==23){
              side2 = colour_mp["Blue"];
            }
            if (breakTime==28){
              side3 = colour_mp["Blue"];
            }
            std::cout<<"Blue - 3"<<"\n";
          }else{
            if (breakTime==18){
              side1 = colour_mp["Green"];
            }
            if (breakTime==23){
              side2 = colour_mp["Green"];
            }
            if (breakTime==28){
              side3 = colour_mp["Green"];
            }
            std::cout<<"Green - 2"<<"\n";
          } 
        }
         // choosing the path and camera disable
        if (breakTime==31){
          cam->disable();
          if (turns == 2){
            path1 = false;
            path2 = true;
          }
          if (turns == 3){
            path1 = false;
            path3 = true;
          }
        }
        
     // ------------------------------------------------------------------------------
        // arm operation
        
        // arm wide
        if (step0 && ps_arm[1]->getValue()<0.35){
          arm[1]->setVelocity(1);
          arm[2]->setVelocity(-1);
        }else if (step0 && ps_arm[1]->getValue()>=0.35){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          step0  = false;
          break0 = true;
        }else if (break0 && breakTime<5){
          breakTime++;
        }else if (break0 && breakTime>=5){
          break0 = false;
          step1  = true;
        }
        // arm down
        else if (step1 && ps_arm[0]->getValue()<1.4){
          arm[0]->setVelocity(1);
        }else if (step1 && ps_arm[0]->getValue()>=1.4){
          arm[0]->setVelocity(0);
          step1  = false;
          break1 = true;
        }else if (break1 && breakTime<10){
          breakTime++;
        }else if (break1 && breakTime>=10){
          break1 = false;
          step2  = true;
        }
        // grab the box
        else if (step2 && timeLimit < 140){
          timeLimit++;
          arm[2]->setVelocity(0.2);
          if (ps_arm[1]->getValue()<0){
            arm[1]->setVelocity(0);
          }else{
            arm[1]->setVelocity(-0.2);
          }
        }else if (step2 && timeLimit >= 140){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break2 = true;
          step2  = false;
        }else if (break2 && breakTime<15){
          breakTime++;
        }else if (break2 && breakTime>=15){
          break2 = false;
          step3  = true;
        }
        // arm up
        else if (step3 && ps_arm[0]->getValue()>1.09){
          arm[0]->setVelocity(-0.3);
        }else if (step3 && ps_arm[0]->getValue()<=1.09){
          arm[0]->setVelocity(0);
          break3 = true;
          step3  = false;
        }else if (break3 && breakTime<20){
          breakTime++;
        }else if (break3 && breakTime>=20){
          break3 = false;
          step4  = true;
        }
        // rotate the box
        else if (step4 && ps_arm[3]->getValue()>-1.6){
          arm[3]->setVelocity(-0.3);
        }else if (step4 && ps_arm[3]->getValue()<=-1.6){
          arm[3]->setVelocity(0);
          break4 = true;
          step4  = false;
        }else if (break4 && breakTime<25){
          breakTime++;
        }else if (break4 && breakTime>=25){
          break4 = false;
          step5  = true;
        }
        // rotate the box
        else if (step5 && ps_arm[3]->getValue()>-3.2){
          arm[3]->setVelocity(-0.3);
        }else if (step5 && ps_arm[3]->getValue()<=-3.2){
          arm[3]->setVelocity(0);
          break5 = true;
          step5  = false;
        }else if (break5 && breakTime<30){
          breakTime++;
        }else if (break5 && breakTime>=30){
          break5 = false;
          step6  = true;
        }
        // turn right
        else if (step6 && turnTime==0){
          turnTime = 20;
        }else if (step6 && turnTime==1){
          turnTime = 0;
          break6 = true;
          step6  = false;
        }else if (break6 && breakTime<35){
          breakTime++;
        }else if (break6 && breakTime>=35){
          break6 = false;
          step7  = true;
        }
        // release the box
        else if (step7 && ps_arm[1]->getValue()<0.85){
          arm[1]->setVelocity(1);
          arm[2]->setVelocity(-1);
        }else if (step7 && ps_arm[1]->getValue()>=0.85){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break7 = true;
          step7  = false;
        }else if (break7 && breakTime<40){
          breakTime++;
        }else if (break7 && breakTime>=40){
          break7 = false;
          step8  =true;
        }
        // arm up
        else if (step8 && ps_arm[0]->getValue()>0){
          arm[0]->setVelocity(-1);
        }else if (step8 && ps_arm[0]->getValue()<=0){
          arm[0]->setVelocity(0);
          break8 = true;
          step8  = false;
        }else if (break8 && breakTime<45){
          breakTime++;
        }else if (break8 && breakTime>=45){
          break8 = false;
          step9  = true;
        }
        // turn Left
        else if (step9 && turnTime==0){
          turnTime = 20;
        }else if (step9 && turnTime==1){
          turnTime = 0;
          step9  = false;
          break9 = true;
        }else if (break9&& breakTime<50){
          breakTime++;
        }else if (break9 && breakTime>=50){
          break9 = false;
          step10 = true;
        }
        // arm restore
        else if (step10 && ps_arm[1]->getValue()>0){
          arm[1]->setVelocity(-1);
          arm[2]->setVelocity(1);
        }else if (step10 && ps_arm[1]->getValue()<=0){
          arm[1]->setVelocity(0);
          arm[2]->setVelocity(0);
          break10 = true;
          step10  = false;
        }else if (break10 && breakTime<55){
          breakTime++;
        }else if (break10 && breakTime>=55){
          break10  = false;
          case6    = false;

          // calculate the differnce
          if (turns==2 || turns==5){
            difference = abs(side2-side3);
            std::cout<<"Front = "<<side3<<"           "<<"Bottom = "<<side2<<"\n"; 
          }else{
            difference = abs(side2-side1);
            std::cout<<"Front = "<<side1<<"           "<<"Bottom = "<<side2<<"\n";
          }
          std::cout<<"Difference: "<<difference<<"\n";
          if (difference%2){
            std::cout<<"Should turn right on the ramp"<<"\n";
          }else{
            std::cout<<"Should turn left on the ramp"<<"\n";
          }
        }

        // stop wheels until the box activities
        leftSpeed  = 0;
        rightSpeed = 0;

        // turns during the box activities
        if (step6 && turnTime>1){
          turnRightI();                                        // turn right immediately         
        }else if (step9 && turnTime>1){
          turnLeftI();                                         // turn left immediately
        }
      }
    // --------------------------------------------------------------------------------------------------------
  
// -------------------------------------------------------------------------------------------------------------
      //get the diameter
          
      if (turns==2 && case1){
        start1 = ps[0]->getValue();
        start2 = ps[1]->getValue();
        case1  = false;
      }
      if ((turns==4) && case2){
        end1    = ps[0]->getValue();
        end2    = ps[1]->getValue();
        ps[0]   ->disable();
        ps[1]   ->disable();
        start   = (start1+start2)/2.0;
        end     = (end1+end2)/2.0;
        int dis = (end-start)*3.5 + 10;
        std::cout<<"Diameter: "<<dis<<"cm"<<"\n";
        
        // display the diameter
        std::string diam;
        std::stringstream ss;
        ss << dis;
        ss >> diam;
        diam = diam + "cm";
        display->drawText(diam,10,10);
        
        case2 = false;
      }

// ---------------------------------------------------------------------------------------------------------------
    // quadrant number display      
      
      if (turns==1 && case8){
        display->drawText("Q-01",13,25);
        std::cout<<"Quadrant No. = 01"<<"\n";
        ps[0]->enable(TIME_STEP);              // enable the position sensor of the wheel
        ps[1]->enable(TIME_STEP);
        case8 = false;
      }
      if (turns==4 && case9){
        if (!path2){
          display->drawText("Q-03",13,35);
          std::cout<<"Quadrant No. = 03"<<"\n";
          case9 = false;
        }else{
          display->drawText("Q-02",13,35);
          std::cout<<"Quadrant No. = 02"<<"\n";
          case5 = true;
          acc->enable(TIME_STEP);             // enable the accelerometer
          baseSpeed = 10;
          case9 = false;
        }
        
      }
      if (((turns==7 && path1) || (turns==5 && path3)) && case10){
        display->drawText("Q=02",13,45);
        std::cout<<"Quadrant No. = 02"<<"\n";
        case5  = true;
        acc->enable(TIME_STEP);             // enable the accelerometer
        baseSpeed = 10;
        case10 = false;
      }

// ----------------------------------------------------------------------------------------------------------------
      // recognizing the ramp
      
      if (case5){
        const double* vals = acc->getValues();
        if (vals[2]>3){
          duration++; 
        }else{
          duration = 0;
        }
        if (duration>10){
          ramp     = 1;
          turnTime = 0;
          acc->disable();
          case5 = false;
        }    
      }
      
// -----------------------------------------------------------------------------------------------------------------    
      // count the poles
      
      if (ramp){
        if (ds_right > 800 || ds_left > 800){
          if (case4){
            poles++;
            ramp++;
          }        
          case4 = false;
        }else{
          case4 = true;
        }     
      }
      // checking the path
      if (poles && (leftWay<lineCutoff || rightWay<lineCutoff) && case3){
        std::cout<<"Pillars: "<<poles<<"\n";
        if (poles == 1){
          turnTime = 52;
          std::cout<<"Path is wrong"<<"\n";
          reverse  = true;
        }else{
          std::cout<<"Path is correct"<<"\n";
        }
        ramp++;        
        case3 = false;
      }
      // after reversing, go forward on next junction
      if (reverse && (leftWay<lineCutoff || rightWay<lineCutoff) && case13  && ramp == 4){
        turnTime = 26;
        ramp++;
        case13 = false;
      }

// ------------------------------------------------------------------------------------------------------------------
      // gate passing
      if (poles && ir[0]->getValue()<lineCutoff && ir[7]->getValue()<lineCutoff && case11){
        if (ds_front<1500){
          case11 = false;
          case12 = true;
        }else{
          leftSpeed  = 0;
          rightSpeed = 0;
        }
      }

      if (poles && ir[0]->getValue()<lineCutoff && ir[7]->getValue()<lineCutoff && ds_front<1500 && case12){
        leftSpeed  = 0;
        rightSpeed = 0;
      }
    }
// -------------------------------------------------------------------------------------------------------------------
    
    // set velocities to wheels
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}