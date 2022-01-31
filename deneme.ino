#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PWMServo.h>
#include <Adafruit_Sensor.h>

#include <utility/imaumaths.h>
#include <Wire.h>
#include <SPI.h>

//BNO055 header
#include <Adafruit_BNO055.h>
//bme280 header
#include <Adafruit_BME280.h>
//gps header
#include <NeoGPS_cfg.h>
#include <ublox/ubxGPS.h>
#include <GPSport.h>
#include <Streamers.h>




// BME 280 tanımları
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;//I2C tanımı
//BME 280 sonu

//GPS
#ifndef NMEAGPS_DERIVED_TYPES
  #error You must "#define NMEAGPS_DERIVED_TYPES" in NMEAGPS_cfg.h!
#endif

#if !defined(UBLOX_PARSE_STATUS)  & !defined(UBLOX_PARSE_TIMEGPS) & \
    !defined(UBLOX_PARSE_TIMEUTC) & !defined(UBLOX_PARSE_POSLLH)  & \
    !defined(UBLOX_PARSE_DOP)     & !defined(UBLOX_PARSE_PVT)     & \
    !defined(UBLOX_PARSE_VELNED)  & !defined(UBLOX_PARSE_SVINFO)  & \
    !defined(UBLOX_PARSE_HNR_PVT)

  #error No UBX binary messages enabled: no fix data available.

#endif

#ifndef NMEAGPS_RECOGNIZE_ALL
  //  Resetting the messages with ublox::configNMEA requires that
  //    all message types are recognized (i.e., the enum has all
  //    values).
  #error You must "#define NMEAGPS_RECOGNIZE_ALL" in NMEAGPS_cfg.h!
#endif
//gps son


float pid_p_roll = 1.3;
float pid_i_roll = 1;
float pid_d_roll = 1;
int pid_max_roll = 1;

float pid_p_pitch = 1;
float pid_i_pitch = 1;
float pid_d_pitch = 1;
int pid_max_pitch = pid_max_roll;

float pid_p_yaw = 1;
float pid_i_yaw = 1;
float pid_d_yaw = 1;
int pid_max_yaw = pid_max_roll;

float pid_p_altitude = 1;
float pid_i_altitude = 1;
float pid_d_altitude = 1;
int pid_max_altitude = pid_max_roll;

float pid_p_gps = 1;
float pid_d_gps = 1;

float pusula_sapma = 0;

int16_t manual_kalkis_gaz = 0;
int16_t motor_rolanti_hiz = 0;// Degisecek

float max_pil_voltaj= 0.0;
float min_pil_voltaj = 10.5;


void setup() {
  // put your setup code here, to run once:
  
  
  
  Serial.begin(9600);

  //BNO055 sensor  
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  // BNO055 başlatma
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
  //BME280 sensor

  while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

  if (!status) {
        Serial.println("Gecerli bir BME280 sensor bulanamadi, kablolamayi, addressi, sensor ID'i kontrol edin");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

  int beginning_Pressure = bme.readPressure() / 100.0F; // Umarım calisir

  //gps baslatma

  //gps son


}

void loop() {
  // put your main code here, to run repeatedly:
  //Kalibrasyon BNO055
  int8_t system, gyro, accel, mag = 0;
  while (system==3 && gyro==3 && accel==3 && mag==3)
  {
    no.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
    //delay(BNO055_SAMPLERATE_DELAY_MS);BNO055 delay
    }




  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);// Roll pitch yaw acilarini sensorden okuma

  /* Display the floating point data */
  Serial.print("Euler Angles X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  imu::Vector<3> mag = bno.getVector (Adafruit_BNO005::VECTOR_EULER);

  Serial.print("Magnetic X: ");
  Serial.print(mag.x());
  Serial.print(" Y: ");
  Serial.print(mag.y());
  Serial.print(" Z: ");
  Serial.print(mag.z());
  Serial.print("\t\t");

  printValues();//basinc yazdirma



  delay(1000);


}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(beginning_Pressure));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}