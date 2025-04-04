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
  double flapAreaPercent = desiredAreaPercent;
  servoAngle = PercentToAngle(PercentConversion(flapAreaPercent));
  actuationServo.write(servoAngle);
  return;
}

/**
 * Converts the desired percent of flap area actuated to the percent the servo rotates of its total rotation
 * @param flapAreaPercent is the desired percent of flap area actuated
 * @return  the percent of its total rotation that the servo should rotate to actuate the amount of flap area desired
*/
double PercentConversion(double flapAreaPercent){
    return flapAreaPercent*(1.0032);
}

/**
 * Converts the desired servo rotation percent to an actual servo positions
 * @param servoActuationPercent is the desired servo rotation percent
 * @return the servo angle as an integer
*/
int PercentToAngle(double servoActuationPercent){
    return round((SERVO_MIN_ANGLE + servoActuationPercent*((SERVO_MAX_ANGLE)-(SERVO_MIN_ANGLE))));
}

//void setup() {
//    Serial.begin(115200);
//    actuationServo.attach(4); //replace with actual pin
//}

//void loop() {
//    desiredAreaPercent = i //will be a function that outputs percent of area needed to cover (use incrementing function for testing)
//    servoAngle = PercentToAngle(PercentConversion(i));
//    actuationServo.write(servoAngle); 
//}
