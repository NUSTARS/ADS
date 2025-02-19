void setup() {
  Serial.println("STARTING");
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  calibrated = false;

  delay(1000);

  for(int i = 200; i < 1500; i++){
    tone(BUZZER, i);
    delay(1);
  }
   tone(BUZZER, 0);

  if(!bno.begin(OPERATION_MODE_NDOF)){
    Serial.println("BNO Failed");
    // tone(BUZZER, 200);
    while(1);
  }
  // if(setupBarometer() != 0){
  //   Serial.println("BMP Failed");
  //   tone(BUZZER, 200);
  //   while(1);
  // }

  // if (setupSD() != 0) {
  //   Serial.println("SD not inserted");
  //   tone(BUZZER, 200);
  //   while(1);
  // };

  Wire.setClock(400000UL);

  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  if(!cal_setup()){
    tone(BUZZER, 200);
    Serial.println("Cal setup stuck");
    delay(2000);
    tone(BUZZER,0);
  }

  ServoSetup();


  // int eeAddress = 0;
  // long bnoID = 0;
  // EEPROM.get(eeAddress, bnoID);
  // adafruit_bno055_offsets_t calibrationData;
  // sensor_t sensor;
  // bno.getSensor(&sensor);
  // bool foundCalibration = false;

  // if (bnoID == sensor.sensor_id) {
  //   EEPROM.get(eeAddress + sizeof(long), calibrationData);
  //   displaySensorOffsets(calibrationData);
  //   bno.setMode(OPERATION_MODE_CONFIG); // Ensure in config mode
  //   delay(10);
  //   bno.setSensorOffsets(calibrationData);
  //   bno.setMode(OPERATION_MODE_NDOF);
  //   Serial.println("Calibration data loaded from EEPROM.");
  //   bno.getSensorOffsets(calibrationData);
  //   foundCalibration = true;
  //   delay(5000);
  // } else {
  //   Serial.println("No calibration data found in EEPROM.");
  //   delay(500);
  // }

  
  // if (foundCalibration) {
  //   Serial.println("Calibration Restored");
  //   displayCalStatus();
  //   displaySensorStatus();
  // }
  // else {
  //    Serial.println("Please Calibrate Sensor: ");
  //         while (!bno.isFullyCalibrated())
  //         {
  //           displayCalStatus();
  //           Serial.println("");
          
  //           /* Wait the specified delay before requesting new data */
  //           delay(100);
  //         }
  // }
  // Serial.println("\nFully calibrated!");
  // Serial.println("--------------------------------");
  // Serial.println("Calibration Results: ");
  // adafruit_bno055_offsets_t newCalib;
  // bno.getSensorOffsets(newCalib);
  // displaySensorOffsets(newCalib);

  // Serial.println("\n\nStoring calibration data to EEPROM...");

  // eeAddress = 0;
  // bno.getSensor(&sensor);
  // bnoID = sensor.sensor_id;

  // EEPROM.put(eeAddress, bnoID);

  // eeAddress += sizeof(long);
  // EEPROM.put(eeAddress, newCalib);
  // Serial.println("Data stored to EEPROM.");

  
  for(int i = 200; i < 1500; i++){
    tone(BUZZER, i);
    delay(1);
  }
  for(int i = 1500; i > 200; i--){
    tone(BUZZER, i);
    delay(1);
  }
  tone(BUZZER, 0); // CHANGING THIS FROM 1000 DOWN TO 450 BC ANNOYING WHEN TESTING

  Serial.println("Taring, do not move...");

  tare = 0.0;
  static sensors_event_t orientationData, angVelocityData, linearAccelData;

  for(int i = 0; i < 100; i++){
      getIMUData(&orientationData, &angVelocityData, &linearAccelData);
      tare += linearAccelData.acceleration.x;
      delay(10);
  }

  tare /= 100.0;

  Serial.println("Setup Complete");

}