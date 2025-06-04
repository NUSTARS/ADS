// add any other libraries needed for SPARC

double desiredAreaPercent; //double from 0 to 1 given by funciton f from SPARC
int servoAngle;  
Servo actuationServo;  // will be used for testing servo until SPARC function complete


void ServoSetup() {
  actuationServo.attach(SERVO_PIN, 500, 2500); // replace with actual pin
  actuationServo.write(SERVO_MIN_ANGLE); 

}

void SetDesiredAreaPercent(double desiredAreaPercent)
{
  servoAngle = map(desiredAreaPercent, 0, 100, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  Serial.println("servo angle");
  Serial.println(servoAngle);
  actuationServo.write(servoAngle);
  return;
}

