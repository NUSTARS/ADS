// return -1 if failed, 0 if success
int getIMUData(sensors_event_t* orientationData, sensors_event_t* angVelocityData, sensors_event_t* linearAccelData) {

  bno.getEvent(orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  

  return 0;
}

void printEvent(sensors_event_t* event) {

  if (linspace > 2) {
    Serial.println();
    linspace = 0;
  }

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

  linspace++;
}
