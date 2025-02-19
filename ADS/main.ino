double calcAvg(double* vals){
  double sum = 0.0;
  for(int i = 0; i < 10; i++){
    sum += vals[i];
  }
  return sum/10.0;
}

void addValue(double* oldVals, double newVal){
  for(int i = 0; i < 10 - 1; i++){
    oldVals[i] = oldVals[i+1];
  }
  oldVals[10-1] = newVal;
}


void loop() {

    static sensors_event_t orientationData, angVelocityData, linearAccelData;
    static double v_body[3] = {0.0,0.0,0.0};
    static double v_world[3] = {0.0,0.0,0.0};
    static unsigned long previousTime = millis();
    double oldValues[10];

    //getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    //delay(10);

    //integrate(&orientationData, &linearAccelData, previousTime, v_world, v_body);
    //previousTime = millis();

/*
    printEvent(&linearAccelData);
    Serial.println();
    printEvent(&orientationData);
    Serial.println();

    Serial.print(" Velocity:");
    Serial.print("\tx = ");
    Serial.print(v_body[0]);
    Serial.print(" \ty = ");
    Serial.print(v_body[1]);
    Serial.print(" \tz = ");
    Serial.println(v_body[2]);
    Serial.println();
*/

    double avg = 0.0;

    for(int i = 0; i < 5; i++){
      getIMUData(&orientationData, &angVelocityData, &linearAccelData);
      avg += linearAccelData.acceleration.x;
      delay(3);
    }

    avg /= 5.0;

    //double avg = calcAvg(oldValues);
    //addValue(oldValues, linearAccelData.acceleration.x);

    /*

  sensors_event_t orientationData, angVelocityData, linearAccelData;
  barometerData baro;
  float magnitude;
  unsigned long previousTime = 0;
  float altitude_offset = 0.0;
  double v_body[3] = {0.0,0.0,0.0};
  double v_world[3] = {0.0,0.0,0.0};
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

  
    // u = SPARC_main_loop(vx, vy, vz, wx, wy, wz, theta_x, theta_y, theta_z, initial_h, u);
    
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
    magnitude = sqrt(pow(linearAccelData.acceleration.x, 2) + pow(linearAccelData.acceleration.y, 2) + pow(linearAccelData.acceleration.z, 2));



    Serial.print("\t\tNOT LAUNCHED YET\t\t");
    Serial.println(magnitude);
    printEvent(&linearAccelData);
    printBarometerData(&baro);
  } while (magnitude < THRESH_ACCEL);
  Serial.println("LAUNCHED!");
  // tone(BUZZER, 1500);



  // LOGGING
  data dataArr[2*LOG_TIME * LOG_FREQ];
  int currentPoint = 0;
  unsigned long loggingStartTime = millis();  // Capture start time



  //while (!openFlaps(&linearAccelData, &baro)) {
    while (1) {
    unsigned long timeStarted = millis();
    // Serial.println("Burnout not reached.");
    getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    getBarometerData(&baro, altitude_offset);
    // printEvent(&linearAccelData);
    // printEvent(&orientationData);
    // printEvent(&angVelocityData);
    // printBarometerData(&baro);


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

    //integration code
    double phi = orientationData.orientation.x * M_PI / 180.0; //x
    double theta = orientationData.orientation.y;  // y
    double psi = (90 - orientationData.orientation.z) * M_PI / 180.0; //z, **NEED TO CHECK THIS

    // Serial.println("before integrate.");
    integrate(&orientationData, &linearAccelData, &previousTime, v_world, v_body);
    // Serial.println("after integrate.");
    double wx = angVelocityData.gyro.x;
    double wy = angVelocityData.gyro.y;  
    double wz = angVelocityData.gyro.z;

    double vx = v_body[0];
    double vy = v_body[1];
    double vz = v_body[2];

    double thetax = phi;
    double thetay = theta;
    double thetaz = psi;
    double initial_h = baro.alt;

    Serial.print("Vx: ");
    Serial.print(vx);
    Serial.print(" Vy: ");
    Serial.print(vy);
    Serial.print(" Vz: ");
    Serial.println(vz);
  
  }

  // Serial.println("Burnout reached!!.");
  // tone(BUZZER, 600);

  for (int i = 0; i < LOG_TIME * LOG_FREQ; i++) {  // 6000 originally, making it less for testing
    unsigned long timeStarted = millis();

    getIMUData(&orientationData, &angVelocityData, &linearAccelData);
    getBarometerData(&baro, altitude_offset);

    double phi = orientationData.orientation.x * M_PI / 180.0; //x
    double theta = orientationData.orientation.y;  // y
    double psi = (90 - orientationData.orientation.z) * M_PI / 180.0; //z, **NEED TO CHECK THIS

    integrate(&orientationData, &linearAccelData, &previousTime, v_world, v_body);
    
    double wx = angVelocityData.gyro.x;
    double wy = angVelocityData.gyro.y;  
    double wz = angVelocityData.gyro.z;

    double vx = v_body[0];
    double vy = v_body[1];
    double vz = v_body[2];

    double thetax = phi;
    double thetay = theta;
    double thetaz = psi;
    double initial_h = baro.alt;

    Serial.print("Vx: ");
    Serial.print(vx);
    Serial.print(" Vy: ");
    Serial.print(vy);
    Serial.print(" Vz: ");
    Serial.println(vz);

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
  */
}





//Serial.println(timeToLog);
//logData();
