void loop() {
  
  float gyro_vals[3];
  float orient_vals[3];
  float accel_vals[3];
  float vel_vals[3];
  double u = 0.0;
  float magnitude = 0;
  int currentPoint = 0;
  data dataArr[int(1.5*LOG_TIME * LOG_FREQ)];

  //WAITING TO LAUNCH
  do {

    sensing.tare();

    sensing.getGyro(gyro_vals);
    sensing.getOrient(orient_vals);
    sensing.getAccel(accel_vals);
    sensing.getVel(vel_vals);
  
    magnitude = sqrt(pow(accel_vals[0], 2) + pow(accel_vals[1], 2) + pow(accel_vals[2], 2));

    Serial.print("\t\tNOT LAUNCHED YET\t\t");
    Serial.println(magnitude);
    Serial.println(sensing.getHeight());


    //delay(BNO055_SAMPLERATE_DELAY_MS);

  } while (magnitude < THRESH_ACCEL);
  Serial.println("LAUNCHED!");
  tone(BUZZER, 1500);

  //LAUNCHED

  //Checking for burnout
  while (!burnoutReached(accel_vals, sensing.getHeight())) {
    sensing.getGyro(gyro_vals);
    sensing.getOrient(orient_vals);
    sensing.getAccel(accel_vals);
    sensing.getVel(vel_vals);
    Serial.println(accel_vals[2]);
    Serial.println(sensing.getHeight());
    // Serial.print("\t x:");
    // Serial.println(vel_vals[0]);
    // Serial.print("\t y:");
    // Serial.println(vel_vals[1]);
    // Serial.print("\t z:");
    // Serial.println(vel_vals[2]);

    
    unsigned long timeStarted = millis();
    // Serial.println("Burnout not reached.");

    dataArr[currentPoint].altitude = sensing.getHeight();

    dataArr[currentPoint].euler_x = orient_vals[0];
    dataArr[currentPoint].euler_y = orient_vals[1]; 
    dataArr[currentPoint].euler_z = orient_vals[2];

    dataArr[currentPoint].accel_x = accel_vals[0];
    dataArr[currentPoint].accel_y = accel_vals[1];
    dataArr[currentPoint].accel_z = accel_vals[2];

    dataArr[currentPoint].ang_x = gyro_vals[0];
    dataArr[currentPoint].ang_y = gyro_vals[1];
    dataArr[currentPoint].ang_z = gyro_vals[2];

    dataArr[currentPoint].vel_x = vel_vals[0];
    dataArr[currentPoint].vel_y = vel_vals[1]; 
    dataArr[currentPoint].vel_z = vel_vals[2];

    dataArr[currentPoint].u = u;

    while (millis() - timeStarted < 1000.0 / LOG_FREQ) {}

    dataArr[currentPoint].time = millis();

    logData2(dataArr);



  }

    Serial.println("Burnout reached!!.");
    tone(BUZZER, 600);

  //BURNOUT REACHED, NOW CONTROLS CAN START
  for (int i = 0; i < LOG_TIME * LOG_FREQ; i++) {  // 6000 originally, making it less for testing

    unsigned long timeStarted = millis();

    sensing.getGyro(gyro_vals);
    sensing.getOrient(orient_vals);
    sensing.getAccel(accel_vals);
    sensing.getVel(vel_vals);


    u = main_loop_dt(vel_vals[0], vel_vals[1], vel_vals[2], gyro_vals[0], gyro_vals[1], gyro_vals[2], orient_vals[0], orient_vals[1], orient_vals[2], sensing.getHeight(), u);

    Serial.println(u);

    SetDesiredAreaPercent(100*u);

    dataArr[currentPoint].altitude = sensing.getHeight();

    dataArr[currentPoint].euler_x = orient_vals[0];
    dataArr[currentPoint].euler_y = orient_vals[1];
    dataArr[currentPoint].euler_z = orient_vals[2];

    dataArr[currentPoint].accel_x = accel_vals[0];
    dataArr[currentPoint].accel_y = accel_vals[1];
    dataArr[currentPoint].accel_z = accel_vals[2];

    dataArr[currentPoint].ang_x = gyro_vals[0];
    dataArr[currentPoint].ang_y = gyro_vals[1];
    dataArr[currentPoint].ang_z = gyro_vals[2];

    dataArr[currentPoint].vel_x = vel_vals[0];
    dataArr[currentPoint].vel_y = vel_vals[1];
    dataArr[currentPoint].vel_z = vel_vals[2];

    dataArr[currentPoint].u = u;

    while (millis() - timeStarted < 1000.0 / LOG_FREQ) {}

    dataArr[currentPoint].time = millis();

    logData2(dataArr);
  }

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
