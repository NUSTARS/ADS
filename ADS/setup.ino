// void displaySensorStatus(void)
// {
//     /* Get the system status values (mostly for debugging purposes) */
//     uint8_t system_status, self_test_results, system_error;
//     system_status = self_test_results = system_error = 0;
//     bno.getSystemStatus(&system_status, &self_test_results, &system_error);

//     /* Display the results in the Serial Monitor */
//     Serial.println("");
//     Serial.print("System Status: 0x");
//     Serial.println(system_status, HEX);
//     Serial.print("Self Test:     0x");
//     Serial.println(self_test_results, HEX);
//     Serial.print("System Error:  0x");
//     Serial.println(system_error, HEX);
//     Serial.println("");
//     delay(500);
// }

// void displayCalStatus(void)
// {
//     /* Get the four calibration values (0..3) */
//     /* Any sensor data reporting 0 should be ignored, */
//     /* 3 means 'fully calibrated" */
//     uint8_t system, gyro, accel, mag;
//     system = gyro = accel = mag = 0;
//     bno.getCalibration(&system, &gyro, &accel, &mag);

//     /* The data should be ignored until the system calibration is > 0 */
//     Serial.print("\t");
//     if (!system)
//     {
//         Serial.print("! ");
//     }

//     /* Display the individual values */
//     Serial.print("Sys:");
//     Serial.print(system, DEC);
//     Serial.print(" G:");
//     Serial.print(gyro, DEC);
//     Serial.print(" A:");
//     Serial.print(accel, DEC);
//     Serial.print(" M:");
//     Serial.print(mag, DEC);
// }

// void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
// {
//     Serial.print("Accelerometer: ");
//     Serial.print(calibData.accel_offset_x); Serial.print(" ");
//     Serial.print(calibData.accel_offset_y); Serial.print(" ");
//     Serial.print(calibData.accel_offset_z); Serial.print(" ");

//     Serial.print("\nGyro: ");
//     Serial.print(calibData.gyro_offset_x); Serial.print(" ");
//     Serial.print(calibData.gyro_offset_y); Serial.print(" ");
//     Serial.print(calibData.gyro_offset_z); Serial.print(" ");

//     Serial.print("\nMag: ");
//     Serial.print(calibData.mag_offset_x); Serial.print(" ");
//     Serial.print(calibData.mag_offset_y); Serial.print(" ");
//     Serial.print(calibData.mag_offset_z); Serial.print(" ");

//     Serial.print("\nAccel Radius: ");
//     Serial.print(calibData.accel_radius);

//     Serial.print("\nMag Radius: ");
//     Serial.println(calibData.mag_radius);
// }

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
    tone(BUZZER, 200);
    while(1);
  }
  if(setupBarometer() != 0){
    Serial.println("BMP Failed");
    tone(BUZZER, 200);
    while(1);
  }

  if (setupSD() != 0) {
    Serial.println("SD not inserted");
    tone(BUZZER, 200);
    while(1);
  };

  Wire.setClock(400000UL);

  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  if(!cal_setup()){
    tone(BUZZER, 200);
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
  tone(BUZZER, 450); // CHANGING THIS FROM 1000 DOWN TO 450 BC ANNOYING WHEN TESTING

  // TESTING DOWN HERE FOR PRINT STATEMENTS
  linspace = 0;

}