// This part works fine.
#ifndef _Line_follow_h
#define _Line_follow_h

#define BUZZER 6

//Number of readings to take for calibration
const int NUM_CALIBRATIONS = 100;


class LineSensor
{
  public:

    LineSensor(int line_pin) 
    {
      pin = line_pin;
      pinMode(pin, INPUT);
    }

    void calibrate();      //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

    
  private:
  
    int pin;
    int average;
    
};


// Returns unmodified reading.
int LineSensor::readRaw()
{
  return analogRead(pin);
}

// Write the calibrate function.
void LineSensor::calibrate()
{
  long sum = 0;

  for (int i = 0; i < NUM_CALIBRATIONS; i++ )
  {
    sum += analogRead(pin);
  }

  average = sum / NUM_CALIBRATIONS;
  analogWrite(BUZZER, 10);
  delay(50);
//  analogWrite(BUZZER, 5);
//  delay(50);
  analogWrite(BUZZER, 0);
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated()
{
  int reading = analogRead(pin) - average;
  reading = constrain(reading, 0, 1023);
  return reading;
}


#endif
// end LineSensors
