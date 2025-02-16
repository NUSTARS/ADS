
// returns -1 if failed, 0 if success
int setupBarometer() {
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    return -1;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Can change these oversampling rates by powers of 2 from none to 16 (higher means more precise but less speed)
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff( BMP3_IIR_FILTER_COEFF_15); // Smooths sensor outputs with the options: BMP3_IIR_FILTER_DISABLE, BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15, BMP3_IIR_FILTER_COEFF_31.
  bmp.setOutputDataRate(BMP3_ODR_200_HZ); // determines how fast new data is made available: Options: BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, etc.

  return 0;
}


// -1 if failed, 0 if success
int getBarometerData(barometerData* baro, float altitude_offset) {

  if(!bmp.performReading()) {
    return -1;
  }

  baro->temp = bmp.temperature * 9/5 + 32; // to Fahrenheit
  baro->press = bmp.pressure / 100;        // to mbar
  baro->alt = bmp.readAltitude(1013.25) * 3.28084 - altitude_offset; // to feet

  return 0;
}

void printBarometerData(barometerData* baro) {
  // Print data
  Serial.print("Temperature = ");
  Serial.print(baro->temp);

  Serial.print(" Pressure = ");
  Serial.print(baro->press); 

  Serial.print(" Altitude = ");
  Serial.println(baro->alt); 
}
