// Calling head files:
#include "encoders.h"
#include "lineSensors.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"

#define PI 3.1415926535897932384626433832795

//Pin definitions for motors:
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// Define the Pin of Buzzer:
#define BUZZER 6

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor

// Calling lineSensors.h for Calibration:
LineSensor left_sensor(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor centre_sensor(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
LineSensor right_sensor(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

// Define the calibrated reading of sensors:
float left_reading;
float centre_reading;
float right_reading;

// Using Kinematics
Kinematics pose;

// PID
float Kp = 0.65; //Proportional gain for position controller
float Ki = 0.016; //Integral gain for position controller
float Kd = 0.0; //Derivative gain for position controller
PID left_PID(Kp, Ki, Kd); //Position controller for left wheel position
PID right_PID(Kp, Ki, Kd); //Position controller for left wheel position

//PID Demand for both wheels;
float demandL;
float demandR;

// Velocity Measurement:
long prev_lcount;
long prev_rcount;
float velocity_l;
float velocity_r;

// Define state, timestamps and Flags:
int state;
bool ReJoin_Flag; 
bool DriveHome_Flag; // this flag is set for DriveHomeX/Y.
float initial_theta; // this flag is set for DriveHomeX/Y to record the theta after turning right/left.
unsigned long start_time;
unsigned long start_rejoin;
unsigned long prev_time;


//  ********************************************************************************************************** //

void setupMotorPins() 
{
  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
//  digitalWrite( L_DIR_PIN, LOW );
//  digitalWrite( R_DIR_PIN, LOW );
}

//  ********************************************************************************************************** //


void setup() 
{

  setupEncoder0();
  setupEncoder1();

  // Calibrate the readings before starting.
  left_sensor.calibrate();
  centre_sensor.calibrate();
  right_sensor.calibrate();

  // We use State and Flags to decide the behaviours:
  state = 0;
  ReJoin_Flag = 0;
  
  // We use and initialise timestamps:
  start_time = micros(); // velocity timestamp;
  prev_time = millis(); // startup timestamp;
  
  // set the previous count of wheels to be zero:
  prev_lcount = 0;
  prev_rcount = 0;

  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");

}


//  ********************************************************************************************************** //


void loop() 
{

  // Setting timestamp for current moment;
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_time;
//  unsigned long move_time = current_time - prev_move_time;
//  unsigned long elapsed_time = current_time - start_time; 


//  ********************************************************** //
  // this section calculate the velocity.
  if ( micros() - start_time > 20000 )
  {
//    velocity_l = (float)(count_left - prev_lcount) * 0.15/1000 * (micros() - start_time);
//    velocity_r = (float)(count_right - prev_rcount) * 0.15/1000 * (micros() - start_time);

    velocity_l = (float) 15050 * (count_left - prev_lcount) /(micros() - start_time);
    velocity_r = (float) 15050 * (count_right - prev_rcount) /(micros() - start_time);
    
    start_time = micros();
    prev_rcount = count_right;
    prev_lcount = count_left;
    
  }

//  ********************************************************** //



//  ********************************************************** //
//  Start to record and update the kinematics.
  if ( update_time > 5 )
  {
    prev_time = current_time; // update prev_time;
    pose.update(count_left, count_right);  // update the kinematics;

    switch (state)
    {
      case 0:
        FindLine(); // Joining Line from the starting point.
        break;
        
      case 1: // There are two approaches to follow the line:
//        Prob_Control(); 
        BangBang();
        break;

      case 2:
        ReJoin(); // Rejoin the line is lost.
        break;

      case 3:
//        FaceHome(); // via rotating.
        fir_rot();
        break;

      case 4:
        DriveHomeX(); // Drive home in X direction.
        break;

      case 5:
        sec_rot(); // anticlockwise 90 degrees.
        break;

      case 6:
        DriveHomeY(); // Drive home in Y direction.
        break;

      case 7: // Return home and Stop the engine. 
        leftMotor(0.0f);
        rightMotor(0.0f);

      default:
        Serial.print("System Error!");
        break;
    }

//    // PID control:
//    float outputL = left_PID.update(demandL, velocity_l);
//    float outputR = right_PID.update(demandR, velocity_r);
//
//    float powerL = outputL * 3;
//    float powerR = outputR * 3;
  }
  
//  ********************************************************** //



//  ********************************************************** //
// //// this section is allocated for monitoring. ////

// Empty

//  ********************************************************** //

}






///////***********************STATE DESCRIPTIONS**********************///////


////////////////////////////  State = 0  ///////////////////////////////////
///////**************************************************************///////
// Moving forward from Starting Point to Find the Line.
void FindLine()
{
  bool online = LineCheck(); // set the status of whether on the line.
  int speed = 30;

  if(! online) // if not on the line.
  {
    // Using theta_control for straight line behaviour.
    float theta = pose.get_theta();
    int PWM = 0;
    if (theta < 0)
      PWM = -2;
    else if (theta > 0)
      PWM = 2;
    else 
      PWM = 0;

    int left_demand = speed - PWM;
    int right_demand = speed + PWM;

    leftMotor(left_demand);
    rightMotor(right_demand);
  }
  
  // if on the line, stop finding.
  else 
  {
    leftMotor(0.0f);
    rightMotor(0.0f);
    state = 1;
  }
  
}

bool LineCheck() // this is important.
{
  bool online = false;
  int threshold = 100;
  left_reading = left_sensor.readCalibrated();
  centre_reading = centre_sensor.readCalibrated();
  right_reading = right_sensor.readCalibrated();

  if ( left_reading > threshold || centre_reading > threshold || right_reading > threshold ) 
    online = true;

  return online;
}

///////**************************************************************///////



////////////////////////////  State = 1  ///////////////////////////////////
///////**************************************************************///////
// Using Probability to determine whether on the line.
void Prob_Control () 
{
 // ******************************* //
}


// BangBang Control: Works fine.
void BangBang() 
{
  left_reading = left_sensor.readCalibrated();
  centre_reading = centre_sensor.readCalibrated();
  right_reading = right_sensor.readCalibrated();

  bool left_on_line = false;
  bool centre_on_line = false;
  bool right_on_line = false;

  float speed = 30.0f;
  
  if (left_reading > 90) 
    left_on_line = true;
  if (centre_reading > 110) 
    centre_on_line = true;
  if (right_reading > 90) 
    right_on_line = true;

  if (centre_on_line) 
  {
    leftMotor(speed);
    rightMotor(speed);
  }
  else if (left_on_line) 
  {
    rightMotor(speed);
    leftMotor(-speed);
    delay(20);
  }
  else if (right_on_line) 
  {
    rightMotor(-speed);
    leftMotor(speed);
    delay(20);
  }
  else 
  {
    leftMotor(0.0f);
    rightMotor(0.0f);
    
    state = 2;
  }
}

///////**************************************************************///////


////////////////////////////  State = 2  ///////////////////////////////////
///////**************************************************************///////
// Try to re-join the line when lost.
bool ReJoin()
{
  bool line_found = false;

  float speed = 20.0f;
  unsigned long current_time = millis();

  if( !ReJoin_Flag ) // ReJoin_Flag is initialise as 0 in setup();
  {
    start_rejoin = millis(); 
    ReJoin_Flag = true;
  }

  unsigned long elapsed_time = current_time - start_rejoin;

  if( elapsed_time < 1000 ) // anticlockwise rotating;
  {
    leftMotor(-speed);
    rightMotor(speed);
    line_found = LineCheck();
  }
  else if( elapsed_time < 3000 ) // clockwise rotating; ( 3000 )
  {
    leftMotor(speed);
    rightMotor(-speed);
    line_found = LineCheck();
  }
  else
  {
    leftMotor(0.0f);
    rightMotor(0.0f);
    
    analogWrite(BUZZER, 10);
    delay(500);
    analogWrite(BUZZER, 0);
    
    state = 3; // Activate FaceHome()
  }

  if( line_found ) // if line is found:
  {
    ReJoin_Flag = false;
    delay(50);
    state = 1; // continue to follow the line via Prob or BB.
  }
}

///////**************************************************************///////



///////////////////////////// State = 3 ////////////////////////////////////
///////**************************************************************///////
// Face Home, rot and anti-rot.
void FaceHome() // this part can be deleted.
{
  float angle = pose.face_home(); // unit: degree;
  fir_rot();
}

void fir_rot()
{

  float speed = 20.0f;
  initial_theta = pose.get_theta(); 
  if ( pose.get_ypos() < 0 ) // default version
  {
    if (initial_theta < 0) 
    {
      leftMotor(speed);
      rightMotor(-speed);
    }
    else
    {
      leftMotor(0.0f);
      rightMotor(0.0f);
      state = 4;
    }
  }
  
  else // mirror version
  {
    if (initial_theta > 0) 
    {
      leftMotor(-speed);
      rightMotor(speed);
    }
    else
    {
      leftMotor(0.0f);
      rightMotor(0.0f);
      state = 4;
    }
  }
}

void sec_rot()
{
  float speed = 20.0f;
  initial_theta = pose.get_theta(); 
  if ( pose.get_ypos() < 0 )
  {
    if (initial_theta > -PI/2) 
    {
      leftMotor(-speed);
      rightMotor(speed);
    }
    else
    {
      leftMotor(0.0f);
      rightMotor(0.0f);
      state = 6;
    }
  }

  else
  {
    if (initial_theta < PI/2)
    {
      leftMotor(speed);
      rightMotor(-speed);
    }
    else
    {
      leftMotor(0.0f);
      rightMotor(0.0f);
      state = 6;
    }
  }
}



///////////////////////////// State = 4 ////////////////////////////////////
///////**************************************************************///////
//Turn to face the end of the map and drive until x position = 0
void DriveHomeX() 
{
  
  if (!DriveHome_Flag) // if setup_distance = 0/false
  {
    initial_theta = pose.get_theta(); //
    DriveHome_Flag = true;
  }

  float theta_delta = initial_theta - pose.get_theta();
  int speed = 30;

  // theta_control starts:
  int PWM = 0;

  if (theta_delta > 0)
    PWM = -2;
  else if (theta_delta < 0) 
    PWM = 2;
  else PWM = 0;

  int left_demand = -speed - PWM;
  int right_demand = -speed + PWM;

  leftMotor(left_demand);
  rightMotor(right_demand);

  if (abs(pose.get_xpos()) < 5) // 10
  {
    DriveHome_Flag = false;
    state = 5;
  }
  
}

///////**************************************************************///////


//////////////////////// State = 6 /////////////////////////////////////////
///////**************************************************************///////
void DriveHomeY() 
{
  
  if (!DriveHome_Flag) 
  {
    initial_theta = pose.get_theta();
    DriveHome_Flag = true;
  }

  float theta_delta = initial_theta - pose.get_theta();
  int speed = 30;

  // theta_control starts:
  int PWM = 0;

  if (theta_delta > 0)
    PWM = -2;
  else if (theta_delta < 0) 
    PWM = 2;
  else PWM = 0;

  int left_demand = -speed - PWM;
  int right_demand = -speed + PWM;

  leftMotor(left_demand);
  rightMotor(right_demand);
  
  if (abs(pose.get_ypos()) < 5) // 60 
  {
    DriveHome_Flag = false;
    state = 7;
  }

}

///////*****************************END
