
#ifndef _PID_h
#define _PID_h
#include <stdint.h>

class PID
{

  public:

    PID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float I, float D );      // This function updates the values of the gains
    void reset();                                   // This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement);  // This function calculates the PID control signal. It should be called in a loop
    void printComponents();                        // This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax);                     // This function sets the maximum output the controller can ask for
    void setDebug(bool state);                      // This function sets the debug flag;
    

  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative

    //We can use this to limit the output to a certain value
    float max_output; 

    //Output components
    //These are used for debugging purposes
    float Kp_output; 
    float Ki_output;
    float Kd_output;
    float output_signal;

    //Values to store between updates().
    float last_demand;      //For storing the previous input
    float last_measurement; //For storing the last measurement
    float last_error;       //For calculating the derivative term
    float error_integral;   //For storing the integral of the error
    long last_millis;       //To track elapsed_time
    bool debug;             //This flag controls whether we print the contributions of each component when update is called

    long integral_millis;
    
};


 PID::PID(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);
  
  // Initialise key variables.
  Kp_output     = 0;
  Ki_output     = 0;
  Kd_output     = 0;
  output_signal = 0;

  max_output        = 255;
  last_demand       = 0;
  last_measurement  = 0;
  last_error        = 0;
  error_integral    = 0;
  debug             = false;
  last_millis       = millis();
  integral_millis = millis();
}

void PID::printComponents() 
{
  Serial.print(Kp_output);
  Serial.print(",");
  Serial.print(Kd_output);
  Serial.print(",");
  Serial.print(Ki_output);
  Serial.print(",");
  Serial.print(output_signal);
  Serial.print("\n");
}


void PID::setGains(float P, float I, float D) 
{
  Kp = P;
  Ki = I;
  Kd = D;
}


float PID::update(float demand, float measurement)
{
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;


  //This represents the error term
  // Decide what your error signal is (demand vs measurement)
  float error;
  error = demand - measurement;   
  
  //This represents the error derivative
  // Calculate the change in your error between update()
  float error_delta;
  error_delta = 0;

  // This represents the error integral.
  // Integrate error over time.
  if(millis()-integral_millis>2)
  {
    error_integral = error_integral + error;
    integral_millis = millis();
  }

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output = Ki * error_integral;
  Kd_output = Kd * error_delta;

    output_signal = Kp_output + Ki_output + Kd_output;



   
  //Update persistent variables.
  last_demand = demand;
  last_measurement = measurement;

     
//    printComponents();
//  }
  
  return output_signal;
}

void PID::setMax(float newMax)
{
  if (newMax > 0)
    max_output = newMax;
  else
    Serial.println("Max output must be positive");
}

void PID::setDebug(bool state) 
{
  debug = state;
}

void PID::reset() 
{
  
  last_error = 0;
  error_integral = 0;
  last_millis = millis();
  
}



#endif
