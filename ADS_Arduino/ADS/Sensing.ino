#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoEigenDense.h>
#include <EEPROM.h>

void Sensing::updateReadings(Sensing* instance){

  instance->updateReadingsHelper();

}

void Sensing::updateReadingsHelper(void){

  sensors_event_t data;
  static int vel_window_index = 0;
  static int accel_window_index = 0;
  static int height_window_index = 0;
  static long last_time_imu = millis();
  static long last_time_baro = millis();
  static float last_height = 0.0;

  bno.getEvent(&data, Adafruit_BNO055::VECTOR_LINEARACCEL);
  accel[accel_window_index][0] = data.acceleration.x;
  accel[accel_window_index][1] = data.acceleration.y;
  accel[accel_window_index][2] = data.acceleration.z;

  baro_height[height_window_index] = bmp.readAltitude(1013.25) * 3.28084;
  // baro_height[height_window_index] = 5;

  if(accel_window_index == ACCEL_AVG_WINDOW-1){

    // get gyro data
    bno.getEvent(&data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro[0] = data.gyro.x;
    gyro[1] = data.gyro.y;
    gyro[2] = data.gyro.z;

    // get orientation data
    bno.getEvent(&data, Adafruit_BNO055::VECTOR_EULER);
    orient[0] = data.orientation.x;
    orient[1] = data.orientation.y;
    orient[2] = data.orientation.z;

    // compute V from imu
    float accel[3];
    getAccel(accel);
    Eigen::Vector3f a_body(accel[0], accel[1], accel[2]); 
    long current_time = millis();
    Eigen::Matrix3f R = getR();
    Eigen::Vector3f a_world = R*(a_body - const_cast<Eigen::Vector3f&>(accel_tare));
    Eigen::Vector3f v_world_imu = ((current_time - last_time_imu)/1000.0)*(a_world) + const_cast<Eigen::Vector3f&>(v_world);

    // filter the two Vs
    const_cast<Eigen::Vector3f&>(v_world) = (getVel_baro()/v_world_imu(2))*v_world_imu;
  }
  if(height_window_index == HEIGHT_AVG_WINDOW-1){
    // baro temp
    baro_temp = bmp.temperature * 9/5 + 32; // to Fahrenheit

    // compute V from baro
    float curr_height = getHeight();
    long current_time = millis();
    v_world_baro[vel_window_index] = (curr_height-last_height)/((current_time - last_time_baro)/1000.0);
    last_height = curr_height;
    last_time_baro = current_time;

    vel_window_index++;
    vel_window_index %= VEL_AVG_WINDOW;  
  }
  

  accel_window_index++;
  height_window_index++;

  accel_window_index %= ACCEL_AVG_WINDOW;
  height_window_index %= HEIGHT_AVG_WINDOW;
}

void Sensing::displaySensorDetails(void){
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void Sensing::displaySensorStatus(void){
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void Sensing::displayCalStatus(void){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
      Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void Sensing::displaySensorOffsets(const adafruit_bno055_offsets_t &calibData){
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

bool Sensing::calibrate(void){
  bool zero = true;
  bool calibrate = true;
  double degToRad = 57.295779513;
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }

  if(calibrate) {
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

  //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        long long time = millis();
        while (!bno.isFullyCalibrated())
        {
            if((millis()-time)/1000 >= 60) {//after 2 min fails
              return false;
            }
            bno.getEvent(&event);

            imu::Vector<3> euler = bno.getQuat().toEuler();
            
            double x = euler.y() * degToRad;
            double y = euler.z() * degToRad;
            double z = euler.x() * degToRad;
            
            Serial.print("X: ");
            Serial.print(x, 4);
            Serial.print(" Y: ");
            Serial.print(y, 4);
            Serial.print(" Z: ");
            Serial.print(z, 4);
            Serial.print("\t\t");

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");
  }
  else {
    bno.setExtCrystalUse(true);
  }
  
  if(zero) {
    Serial.println("Zeroing... Please do not move the device");
    delay(1000);
  }
  
  bno.setMode(0x0C);
  Serial.println("Calibration Complete");
  return true;
}

void Sensing::printEvent(sensors_event_t* event) {

  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print(" Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print(" Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print(" Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print(" Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print(" Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print(" Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print(" Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print(" Unk:");
  }

  Serial.print("\tx = ");
  Serial.print(x);
  Serial.print(" \ty = ");
  Serial.print(y);
  Serial.print(" \tz = ");
  Serial.print(z);
}

// gets body to earth rotation based on current orientation
Eigen::Matrix3f Sensing::getR(){
  noInterrupts();
  // Bosch: heading z, pitch x, roll y 
  // heading roll pitch
  // Max: Heading x, Pitch y, Roll z?
  // Alr this should be good 
  float phi = -orient[2] * 0.01745329; //x 
  float theta = -orient[1]* 0.01745329; //y
  float psi = orient[0] * 0.01745329; //z MUST BE
  interrupts();

  float cos_psi = cos(psi);
  float sin_psi = sin(psi);
  float cos_phi = cos(phi);
  float sin_phi = sin(phi);
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  /*
  Eigen::Matrix3f R; // yxz
    R << cos_theta * cos_psi - sin_theta * sin_phi * sin_psi, -cos_phi * sin_psi, cos_psi * sin_theta + cos_theta * sin_phi * sin_psi,
         cos_theta * sin_psi + sin_theta * sin_phi * cos_psi,  cos_phi * cos_psi, sin_psi * sin_theta - cos_theta * sin_phi * cos_psi,
        -sin_theta * cos_phi, sin_phi, cos_theta * cos_phi;
  */

    Eigen::Matrix3f Rx {{1,         0,         0},
                 {0,  cos(phi),  -sin(phi)},
                 {0, sin(phi), cos(phi)}};

    Eigen::Matrix3f Ry {{cos(theta), 0, sin(theta)},
                 {0         , 1,           0},
                 {-sin(theta), 0,  cos(theta)}};
    Eigen::Matrix3f Rz {{cos(psi) , -sin(psi), 0},
                 {sin(psi), cos(psi), 0},
                 {        0,        0, 1}};
    Eigen::Matrix3f R = Rz*Ry*Rx; // Rz MUST come last

  return R;
}

// gets earth to body rotation based on current orientation
Eigen::Matrix3f Sensing::getRinv(){
  /*
  noInterrupts();
  float phi = orient[0] * 0.01745329; // pi / 180.0
  float theta = orient[1]* 0.01745329;
  float psi = (orient[2]) * 0.01745329; //need to check
  interrupts();

  float cos_psi = cos(psi);
  float sin_psi = sin(psi);
  float cos_phi = cos(phi);
  float sin_phi = sin(phi);
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

 // Hardcoded inverse (transpose of R)
  Eigen::Matrix3f R_inv {{cos_theta * cos_psi - sin_theta * sin_phi * sin_psi, cos_theta * sin_psi + sin_theta * sin_phi * cos_psi, -sin_theta * cos_phi},
                         {-cos_phi * sin_psi, cos_phi * cos_psi, sin_phi},
                         {cos_psi * sin_theta + cos_theta * sin_phi * sin_psi, sin_psi * sin_theta - cos_theta * sin_phi * cos_psi, cos_theta * cos_phi}};   

  return R_inv;
  */
  return getR().transpose();
}

// turns on all stuff
bool Sensing::begin(int freq){

  bool status = true;

  if(!bno.begin(OPERATION_MODE_NDOF)){
    status = false;
  }
  else if (!calibrate()){
    status = false;
  }

  memset(accel, 0, sizeof(accel));

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x08); // accel config register
  Wire.write(0x0F); // 16G
  Wire.endTransmission(true); 

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    status = false;
  }
  else{
  // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Can change these oversampling rates by powers of 2 from none to 16 (higher means more precise but less speed)
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15); // Smooths sensor outputs with the options: BMP3_IIR_FILTER_DISABLE, BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15, BMP3_IIR_FILTER_COEFF_31.
    bmp.setOutputDataRate(BMP3_ODR_100_HZ); // determines how fast new data is made available: Options: BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, etc.
  }

  timer.begin([this]() {updateReadings(this); },  1000000l/(freq));
  delay(1000);
  Serial.println("Sensing Setup Complete");

  return status;
}

// tares accelrometer and height based on current readings
void Sensing::tare(){
  float currAccel[3];
  getAccel(currAccel);
  Eigen::Vector3f a_body {currAccel[0], currAccel[1], currAccel[2]};
  
  noInterrupts();
  const_cast<Eigen::Vector3f&>(accel_tare) = a_body;
  v_world.setZero();
  baro_tare += getHeight();
  interrupts();
}

// gets gyro from the imu, unfiltered
void Sensing::getGyro(float* gyro_vals){
  noInterrupts();
  gyro_vals[0] = gyro[0];
  gyro_vals[1] = gyro[1];
  gyro_vals[2] = gyro[2];
  interrupts();
}

// gets orientation from the imu, unfiltered
void Sensing::getOrient(float* orient_vals){
  noInterrupts();
  orient_vals[0] = orient[0];
  orient_vals[1] = orient[1];
  orient_vals[2] = orient[2];
  interrupts();
}

// gets tared acceleration from the imu, filtered
void Sensing::getAccel(float* accel_vals){

  float sums[3] = {0,0,0};

  noInterrupts();
  for(int i = 0; i < ACCEL_AVG_WINDOW; i++){
    sums[0] += accel[i][0];
    sums[1] += accel[i][1];
    sums[2] += accel[i][2];
  }
  interrupts();

  accel_vals[0] = sums[0]/ACCEL_AVG_WINDOW;
  accel_vals[1] = sums[1]/ACCEL_AVG_WINDOW;
  accel_vals[2] = sums[2]/ACCEL_AVG_WINDOW;

}

// gets velocity filtered from the two sensors 
void Sensing::getVel(float* vel_vals){
  noInterrupts();
  Eigen::Vector3f v_body = getRinv()*const_cast<Eigen::Vector3f&>(v_world);
  vel_vals[0] = v_body(0);
  vel_vals[1] = v_body(1);
  vel_vals[2] = v_body(2);
  interrupts();
}

// gets tared height from the barometer, unfiltered
float Sensing::getHeight(){
  float sum = 0.0;

  noInterrupts();
  for(int i = 0; i < HEIGHT_AVG_WINDOW; i++){
    sum += baro_height[i];
  }
  interrupts();
  return sum/HEIGHT_AVG_WINDOW - baro_tare;
}

float Sensing::getVel_baro(){

  float sum = 0.0;

  noInterrupts();
  for(int i = 0; i < VEL_AVG_WINDOW; i++){
    sum += v_world_baro[i];
  }
  interrupts();

  return sum/VEL_AVG_WINDOW;
}
