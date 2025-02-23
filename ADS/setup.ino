void setup() {
  Serial.begin(115200);
  Serial.println("STARTING");
  pinMode(BUZZER, OUTPUT);

  delay(1000);

  for(int i = 200; i < 1500; i++){
    tone(BUZZER, i);
    delay(1);
  }
   tone(BUZZER, 0);

  if(!imu_var.begin(90)){
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

  ServoSetup();
  
  for(int i = 200; i < 1500; i++){
    tone(BUZZER, i);
    delay(1);
  }
  for(int i = 1500; i > 200; i--){
    tone(BUZZER, i);
    delay(1);
  }
  tone(BUZZER, 0); // CHANGING THIS FROM 1000 DOWN TO 450 BC ANNOYING WHEN TESTING

  imu_var.tare();

  Serial.println("Setup Complete");

}