/* Notes
  - Will be using a Teensy 4.1 to log data, that will be the primary purpose 
  - Using a BNO085 to get IMU data
    - Want angles, velocity, apogee reached, max velocity, orientation at landing
  - Using a BMP390 for alitutde calculation 
    - Want maximum altitude (apogee) (have both BNO integrate and BMP for comparisson -- 3 apogee comparissons)
  - Time of landing 
    - Will use on board clock on MCU to keep track of time 
  - Will need to log data
    - Log data in CSV format 


  Notes
    - Download the Teensyduino library 
      - https://www.pjrc.com/teensy/tutorial.html
    - https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/uart-rvc-for-arduino
    - How to store data the fastest

  Things to do
    - Check why we are getting random data into SD card
    - Make sure to validate which axis is up
    - Timer for getting all data points at a constant rate
    - Look into calibration

  if weird error, denied ability to access key strokes (this is for my mac specifically don't worry abt it)
*/

// ---------- Include ---------- //
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <SdFat.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h> 
#include <ArduinoEigenDense.h>
#include <main_sensors.h>


#define BUZZER 5 // actually 5 
#define SERVO_PIN 24

#define LOG_FREQ 50 // in Hz
#define LOG_TIME 40 // in s (CHANGE THIS BACK) to 60
//#define FILE_NAME "data.csv"
#define THRESH_ACCEL 50 // in ft/s^2  (PUT TO 30)
#define BURNOUT_HEIGHT 600 //ft

// Barometer
#define SEALEVELPRESSURE_HPA (1013.25)

#define ACCEL_AVG_WINDOW 5
#define HEIGHT_AVG_WINDOW 10
#define VEL_AVG_WINDOW 2
#define BNO055_SAMPLERATE_DELAY_MS 10

// Servo
#define SERVO_MIN_ANGLE 74 //min servo angle corresponding to 0% actuation 70
#define SERVO_MAX_ANGLE 20 //max servo angle corresponding to 100% actuation

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

struct data {
  float time;
  // float temp;
  // float pressure;
  float altitude;
  float euler_x;
  float euler_y;
  float euler_z;
  float ang_x;
  float ang_y;
  float ang_z;
  float accel_x;
  float accel_y;
  float accel_z;
  float vel_x;
  float vel_y;
  float vel_z;
  float u;
};

// SD Functions
int setupSD();
void logData(data* dataArr, int arrLen);
void logData2(data* dataArr);

// Flap Functions
bool openFlapsAccel(float* accel_vals);
bool openFlapsHeight(float height);
bool burnoutReached(float* accel_vals, float height);

// IMU vars  ---------------------------------
Sensing sensing;

// SD Stuff ---------------------------------------------------
SdFat SD;
FsFile dataFile;
const String FILE_NAME  = "data.csv";













