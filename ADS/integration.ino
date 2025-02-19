void computeRotationMatrix(double phi, double theta, double psi, double R[9]) {
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

void transposeMatrix(double R[9], double R_inv[9]) {
    R_inv[0] = R[0]; R_inv[1] = R[3]; R_inv[2] = R[6];
    R_inv[3] = R[1]; R_inv[4] = R[4]; R_inv[5] = R[7];
    R_inv[6] = R[2]; R_inv[7] = R[5]; R_inv[8] = R[8];
}

void multiplyMatrix(double A[9], double B[3], double result[3]) {
    // Multiply A (3x3) with B (3x1)
    result[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    result[1] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
    result[2] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
}

void integrate(sensors_event_t* orientationData, sensors_event_t* linearAccelData, unsigned long previousTime, double v_world[3], double v_body[3]){
  unsigned long currentTime = millis();

  float deltaTime = (currentTime - previousTime) / 1000.0; // Delta time in seconds
  
  //we have euler x euler y euler z 
  double phi = orientationData->orientation.x * M_PI / 180.0;
  double theta = orientationData->orientation.y;
  double psi = (90 - orientationData->orientation.z) * M_PI / 180.0; //need to check

  // Serial.println("after angles."); 
  // Initialize the rotation matrix R (3x3)
  double R[9];
  computeRotationMatrix(phi, theta, psi, R);

  // Serial.println("computer R."); 

  double R_inv[9];
  transposeMatrix(R, R_inv);
  // Matrix.inv(R, 3, R_inv);

  // Serial.println("compute R_inv."); 

  double a_body[3] = {linearAccelData->acceleration.x, linearAccelData->acceleration.y, linearAccelData->acceleration.z};

  // Serial.println("a body."); 

  // Resultant acceleration in the world frame
  double a_world[3];
  multiplyMatrix(R_inv, a_body, a_world);

  // Serial.println("m mutliply ."); 

  
  // fwd integrate in world frame frame
  v_world[0] += a_world[0] * deltaTime;
  v_world[1] += a_world[1] * deltaTime;
  v_world[2] += a_world[2] * deltaTime;

  // Serial.println("m mutliply x2."); 

  multiplyMatrix(R, v_world, v_body);
}