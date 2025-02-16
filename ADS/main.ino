#include <MatrixMath.h>
#include <math.h> 

void computeRotationMatrix(float phi, float theta, float psi, float R[9]) {
  // Fill the 3x3 rotation matrix R in row-major order
  R[0] = 1;
  R[1] = sin(phi) * sin(theta);
  R[2] = cos(phi) * tan(theta);
  
  R[3] = 0;
  R[4] = cos(phi);
  R[5] = -sin(phi);
  
  R[6] = 0;
  R[7] = sin(phi) / cos(theta);
  R[8] = cos(phi) / cos(theta);
}

void integrate(sensors_event_t* orientationData, sensors_event_t* linearAccelData, unsigned long* previousTime, float* v_world[3], float* v_body[3]){
  unsigned long currentTime = millis();

  float deltaTime = (currentTime - previousTime) / 1000.0; // Delta time in seconds
  
  //we have euler x euler y euler z 
  double phi = orientationData.orientation.x * M_PI / 180.0; //x
  double theta = orientationData.orientation.y;  // y
  double psi = (90 - orientationData.orientation.z) * M_PI / 180.0; //z, **NEED TO CHECK THIS

  // Initialize the rotation matrix R (3x3)
  float R[9];
  computeRotationMatrix(phi, theta, psi, R);

  float R_inv[9];
  Matrix.inv(R, 3, R_inv);

  float a_body[3] = {linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z};

  // Resultant acceleration in the world frame
  float a_world[3];
  Matrix.Multiply(R_inv, a_body, a_world, 3, 3, 1);

  
  // fwd integrate in world frame frame
  v_world[0] += a_world[0] * deltaTime;
  v_world[1] += a_world[1] * deltaTime;
  v_world[2] += a_world[2] * deltaTime;

  Matrix.Multiply(R, v_world, v_body, 3, 3, 1);

  previousTime = currentTime;

}


void loop() {

  sensors_event_t orientationData, angVelocityData, linearAccelData;
  barometerData baro;
  float magnitude;
  unsigned long previousTime = 0;
  float altitude_offset = 0.0;
  float v_body[3] = {0.0,0.0,0.0};
  float v_world[3] = {0.0,0.0,0.0};
  float dt;
  double u = 0.0;

  getBarometerData(&baro, altitude_offset);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  getBarometerData(&baro, altitude_offset);
  Serial.print(altitude_offset);
  altitude_offset = (&baro)->alt; 
  Serial.print(altitude_offset);
  printBarometerData(&baro);


  // STARTUP WAITING
  do {
    getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    getBarometerData(&baro, altitude_offset);

    double phi = orientationData.orientation.x * M_PI / 180.0; //x
    double theta = orientationData.orientation.y;  // y
    double psi = (90 - orientationData.orientation.z) * M_PI / 180.0; //z, **NEED TO CHECK THIS


    magnitude = sqrt(pow(linearAccelData.acceleration.x, 2) + pow(linearAccelData.acceleration.y, 2) + pow(linearAccelData.acceleration.z, 2));

    integrate(&orientationData, &linearAccelData, &previousTime, &v_world, &v_body)
    


    double wx, wy, wz = angVelocityData.gyro.x, angVelocityData.gyro.z, angVelocityData.gyro.z;
    double vx, vy, vz = v_body[0], v_body[1], v_body[2]
    double thetax, thetay, thetaz = phi, theta, psi
    double initial_h = baro.alt
    u = SPARC_main_loop(vx, vy, vz, wx, wy, wz, theta_x, theta_y, theta_z, initial_h, u);

    //calibration_setup(bno, sys, gyro, accel, mag);
    
    // Serial.print(F("Calibration: "));
    // Serial.print(sys, DEC);
    // Serial.print(F(", "));
    // Serial.print(gyro, DEC);
    // Serial.print(F(", "));
    // Serial.print(accel, DEC);
    // Serial.print(F(", "));
    // Serial.print(mag, DEC);
    // Serial.println(F(""));
    // Serial.println(calibrated);

    // printEvent(&orientationData);
    // printEvent(&angVelocityData);
    // printEvent(&linearAccelData);
    // printBarometerData(&baro);
    
    delay(BNO055_SAMPLERATE_DELAY_MS);


    Serial.print("\t\tNOT LAUNCHED YET\t\t");
    Serial.println(magnitude);
    printEvent(&linearAccelData);
    printBarometerData(&baro);
  } while (magnitude < THRESH_ACCEL);
  Serial.println("LAUNCHED!");
  tone(BUZZER, 1500);


  // LOGGING
  data dataArr[2*LOG_TIME * LOG_FREQ];
  int currentPoint = 0;
  unsigned long loggingStartTime = millis();  // Capture start time



  while (!openFlaps(&linearAccelData, &baro)) {
    unsigned long timeStarted = millis();
    Serial.println("Burnout not reached.");
    getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    getBarometerData(&baro, altitude_offset);
    printEvent(&linearAccelData);
    // printEvent(&orientationData);
    // printEvent(&angVelocityData);
    printBarometerData(&baro);


    dataArr[currentPoint].pressure = baro.press;
    dataArr[currentPoint].temp = baro.temp;
    dataArr[currentPoint].altitude = baro.alt;

    dataArr[currentPoint].euler_x = orientationData.orientation.x;
    dataArr[currentPoint].euler_y = orientationData.orientation.y;
    dataArr[currentPoint].euler_z = orientationData.orientation.z;

    dataArr[currentPoint].accel_x = linearAccelData.acceleration.x;
    dataArr[currentPoint].accel_y = linearAccelData.acceleration.y;
    dataArr[currentPoint].accel_z = linearAccelData.acceleration.z;

    dataArr[currentPoint].ang_x = angVelocityData.gyro.x;
    dataArr[currentPoint].ang_y = angVelocityData.gyro.y;
    dataArr[currentPoint].ang_z = angVelocityData.gyro.z;


    while (millis() - timeStarted < 1000.0 / LOG_FREQ) {}

    dataArr[currentPoint].time = millis();

    logData2(dataArr);
  
  }

  Serial.println("Burnout reached!!.");
  tone(BUZZER, 600);

  for (int i = 0; i < LOG_TIME * LOG_FREQ; i++) {  // 6000 originally, making it less for testing
    unsigned long timeStarted = millis();

    getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    getBarometerData(&baro, altitude_offset);

    // CHANGING CURERNT POINT TO i
    dataArr[currentPoint].pressure = baro.press;
    dataArr[currentPoint].temp = baro.temp;
    dataArr[currentPoint].altitude = baro.alt;

    dataArr[currentPoint].euler_x = orientationData.orientation.x;
    dataArr[currentPoint].euler_y = orientationData.orientation.y;
    dataArr[currentPoint].euler_z = orientationData.orientation.z;

    dataArr[currentPoint].accel_x = linearAccelData.acceleration.x;
    dataArr[currentPoint].accel_y = linearAccelData.acceleration.y;
    dataArr[currentPoint].accel_z = linearAccelData.acceleration.z;

    dataArr[currentPoint].ang_x = angVelocityData.gyro.x;
    dataArr[currentPoint].ang_y = angVelocityData.gyro.y;
    dataArr[currentPoint].ang_z = angVelocityData.gyro.z;

    // printEvent(&orientationData);
    // printEvent(&angVelocityData);
    // printEvent(&linearAccelData);
    // printBarometerData(&baro);

    while (millis() - timeStarted < 1000.0 / LOG_FREQ) {}

    dataArr[currentPoint].time = millis();

    //unsigned long loggingStartTime = millis();  // Capture start time
    logData2(dataArr);
    //Serial.println(millis() - loggingStartTime);


    //Serial.println(dataArr[currentPoint].time); //TESTING
  }

  //SD WRITE

  Serial.println("DONE LOGGING");
  actuationServo.write(SERVO_MIN_ANGLE);


  // END

  while (1) {
    for (int i = 0; i < 3; i++) {
      noTone(BUZZER);
      delay(50);
      tone(BUZZER, random(300, 2000));
      delay(50);
    }
  }
}





//Serial.println(timeToLog);
//logData();
