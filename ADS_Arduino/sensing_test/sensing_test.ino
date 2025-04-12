#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <math.h> 
#include <ArduinoEigenDense.h>

// Barometer
#define SEALEVELPRESSURE_HPA (1013.25)

#define ACCEL_AVG_WINDOW 5
#define HEIGHT_AVG_WINDOW 10
#define VEL_AVG_WINDOW 2
#define BNO055_SAMPLERATE_DELAY_MS 10

long last_tare = millis();

float gyro[3];
float orient[3];
float accel[3];
float velocity[3];

class Sensing{

  private:

    const uint8_t IMU_ADDR = BNO055_ADDRESS_A;
    Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDR);

    Adafruit_BMP3XX bmp; // default adress set to 0x77 (I2C address)

    volatile float accel[ACCEL_AVG_WINDOW][3];
    volatile float orient[3] = {0.0, 0.0, 0.0};
    volatile float gyro[3] = {0.0, 0.0, 0.0};

    volatile float baro_height[HEIGHT_AVG_WINDOW];
    volatile float v_world_baro[VEL_AVG_WINDOW];
    volatile float baro_temp;

    volatile Eigen::Vector3f accel_tare;
    volatile float baro_tare;
    volatile Eigen::Vector3f v_world;
  
    IntervalTimer timer;

    static void updateReadings(Sensing* instance);
    void updateReadingsHelper(void);
    void displaySensorDetails(void);
    void displaySensorStatus(void);
    void displayCalStatus(void);
    void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
    bool calibrate(void);
    void printEvent(sensors_event_t* event);

    Eigen::Matrix3f getR();
    Eigen::Matrix3f getRinv();

  public:
    bool begin(int freq);
    void tare();
    void getGyro(float* gyro_vals);
    void getOrient(float* orient_vals);
    void getAccel(float* accel_vals);
    void getVel(float* vel_vals);
    float getHeight();
    float getVel_baro();

};

Sensing sensing;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.setClock(400000UL);

  delay(5000);

  Serial.println("BEGIN");

  if(!sensing.begin(50)){
   Serial.println("Sensing Failed");
   while(1);
  }
}

void loop() {
  if(millis() - last_tare > 20000){
    sensing.tare();
    last_tare = millis();
  }

  sensing.getOrient(orient);
  sensing.getAccel(accel);
  sensing.getGyro(gyro);
  sensing.getVel(velocity);

  // Serial.print("O: ");
  // Serial.print(orient[0]); Serial.print(", "); Serial.print(orient[1]); Serial.print(", "); Serial.print(orient[2]);
  // Serial.print("G: ");
  // Serial.print(gyro[0]); Serial.print(", "); Serial.print(gyro[1]); Serial.print(", "); Serial.print(gyro[2]);
  // Serial.print("A: ");
  // Serial.print(accel[0]); Serial.print(", "); Serial.print(accel[1]); Serial.print(", "); Serial.print(accel[2]);
  // Serial.print("V: ");
  // Serial.print(velocity[0]); Serial.print(", "); Serial.print(velocity[1]); Serial.print(", "); Serial.print(velocity[2]);
  // Serial.println();

  Serial.println(accel[2]);

  delay(2);

  

}
