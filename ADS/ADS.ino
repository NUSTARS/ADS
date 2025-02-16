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

#define BUZZER 5
#define SERVO_PIN 24


#define LOG_FREQ 50 // in Hz
#define LOG_TIME 60 // in s (CHANGE THIS BACK) to 60
#define THRESH_ACCEL 30 // in ft/s^2  (PUT TO 50)
#define FILE_NAME "data.csv" // CHANGING THIS TO A TEXT FILE BC GETTING REALLY GOOFY NUMBERS IN CSV
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BURNOUT_HEIGHT 400 //ft

// Barometer
#define SEALEVELPRESSURE_HPA (1013.25)

#define RECALIB 1 // 1 = recalibrate, 0 = dont


// IMU vars  ---------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)
bool calibrated;
int altitude_offset;

// Barometer vars ---------------------------------
Adafruit_BMP3XX bmp; // default adress set to 0x77 (I2C address)

// SD Stuff ---------------------------------------------------
SdFat SD;
FsFile dataFile;
int linspace; // fixes printing for imu to make it easier to see 

// Structs -----------------------------------------------------
struct barometerData {
  float temp;
  float press;
  float alt;
};

struct data {
  float time;
  float temp;
  float pressure;
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
};

// Functions -------------------------------------------------------------------------
// Declaring IMU functions
int getIMUData(sensors_event_t* orientationData, sensors_event_t* angVelocityData, sensors_event_t* linearAccelData);
void printEvent(sensors_event_t* event);

// Barometer Functions
int setupBarometer();
int getBarometerData(barometerData* baro, float altitude_offset);
void printBarometerData(barometerData* baro);

// SD Functions
int setupSD();
void logData(data* dataArr, int arrLen);
void logData2(data* dataArr);

void displayCalStatus(void);
bool cal_setup(void);

void calibration_setup(Adafruit_BNO055& bno, uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);

bool openFlapsAccel(sensors_event_t* event);
bool openFlapsHeight(barometerData* baro);
bool openFlaps(sensors_event_t* event, barometerData* baro);


void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);














