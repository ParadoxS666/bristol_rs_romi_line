// Calling head files:
#include "encoders.h"
#include "lineSensors.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"

#define PI 3.1415

//Pin definitions for motors:
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// Define the Pin of Buzzer:
#define BUZZER 6

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A0 //Pin for the left line sensor
#define LINE_CENTRE_PIN A2 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A3 //Pin for the right line sensor


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
float Kp = 0.63; 
float Ki = 0.014; 
float Kd = 0.0; 
PID left_PID(Kp, Ki, Kd); 
PID right_PID(Kp, Ki, Kd); 


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

void setupMotorPins()
{
  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );


}

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


void loop()
{

  // Setting timestamp for current moment;
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_time;


  //  ---------------------------------------------- //
  // this section calculates the velocity.
  if ( micros() - start_time > 20000 )
  {

    velocity_l = (float) 15050 * (count_left - prev_lcount) / (micros() - start_time);
    velocity_r = (float) 15050 * (count_right - prev_rcount) / (micros() - start_time);

    start_time = micros();
    prev_rcount = count_right;
    prev_lcount = count_left;

  }

  //  ---------------------------------------------- //



  //  ---------------------------------------------- //
  //  Start to record and updates the kinematics.
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
        BangBang();
        break;

      case 2:
        ReJoin(); // Rejoin the line is lost.
        break;

      case 3:
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

    // PID control:
    float outputL = left_PID.update(demandL, velocity_l);
    float outputR = right_PID.update(demandR, velocity_r);
    float powerL = outputL * 3;
    float powerR = outputR * 3;
  }

}



////////////////////////////  State = 0  ///////////////////////////////////
// Moving forward from Starting Point to Find the Line.
void FindLine()
{
  Serial.println("Find");
  bool online = LineCheck(); // set the status of whether on the line.
  int speed = 30;

  if (! online) // if not on the line.
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

///////----------------------------------------------///////



////////////////////////////  State = 1  ///////////////////////////////////

// BangBang Control: Works fine.
void BangBang()
{ Serial.println("Bang");
  left_reading = left_sensor.readCalibrated();
  centre_reading = centre_sensor.readCalibrated();
  right_reading = right_sensor.readCalibrated();

  bool left_on_line = false;
  bool centre_on_line = false;
  bool right_on_line = false;

  float speed = 60.0f; //Safe speed is upto 80

  if (left_reading > 100)
    left_on_line = true;
  if (centre_reading > 100)
    centre_on_line = true;
  if (right_reading > 100)
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

////////////////////////////  State = 2  ///////////////////////////////////

// Try to re-join the line when lost.
bool ReJoin()
{ Serial.println("Rejoin");
  bool line_found = false;

  float speed = 20.0f;
  unsigned long current_time = millis();

  if ( !ReJoin_Flag ) // ReJoin_Flag is initialise as 0 in setup();
  {
    start_rejoin = millis();
    ReJoin_Flag = true;
  }
  unsigned long elapsed_time = current_time - start_rejoin;
  if ( elapsed_time < 2000 ) // anticlockwise rotating;
  {
    leftMotor(speed);
    rightMotor(-speed);
    line_found = LineCheck();
  }
  else if ( elapsed_time < 6000 ) // clockwise rotating;
  {
    leftMotor(-speed);
    rightMotor(speed);
    line_found = LineCheck();
  }
  else if ( elapsed_time < 8000 ) // anti-clockwise rotating;
  {
    leftMotor(speed);
    rightMotor(-speed);
    line_found = LineCheck();
  }
  else if ( elapsed_time < 10000 ) // forward move to search;
  {
    leftMotor(speed);
    rightMotor(speed);
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

  if ( line_found ) // if line is found:
  {
    ReJoin_Flag = false;
    delay(50);
    state = 1; // continue to follow the line via Prob or BB.
  }
}

///////----------------------------------------------///////



///////////////////////////// State = 3 ////////////////////////////////////

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
    if (initial_theta > -PI / 2)
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
    if (initial_theta < PI / 2)
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
//Turn to face the end of the map and drive until x position = 0
void DriveHomeX()
{

  if (!DriveHome_Flag) // if setup_distance = 0/false
  {
    initial_theta = pose.get_theta(); //
    DriveHome_Flag = true;
  }

  float theta_delta = initial_theta - pose.get_theta();
  int speed = 80;

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

//////////////////////// State = 6 /////////////////////////////////////////
void DriveHomeY()
{

  if (!DriveHome_Flag)
  {
    initial_theta = pose.get_theta();
    DriveHome_Flag = true;
  }

  float theta_delta = initial_theta - pose.get_theta();
  int speed = 80;

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
