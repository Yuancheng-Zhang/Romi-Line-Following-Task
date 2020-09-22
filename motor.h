
#ifndef _motor
#define _motor_h

//Pin definitions for motor:
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

void leftMotor(float speed) 
{
  if (speed < -255.0f || speed > 255.0f) 
    Serial.println("Invalid Left Motor Speed.");
  else 
  {
    if (speed >= 0) 
      digitalWrite( L_DIR_PIN, LOW );
    else 
      digitalWrite( L_DIR_PIN, HIGH );
    speed = abs(speed);
    analogWrite( L_PWM_PIN, speed );
  }
}

void rightMotor(float speed) 
{
  if (speed < -255.0f || speed > 255.0f) 
    Serial.println("Invalid Right Motor Speed.");
  else 
  {
    if (speed >= 0) 
      digitalWrite( R_DIR_PIN, LOW );
    else 
      digitalWrite( R_DIR_PIN, HIGH );
    speed = abs(speed);
    analogWrite( R_PWM_PIN, speed );
  }
}

#endif
