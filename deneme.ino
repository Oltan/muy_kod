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
/*
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

class MyGPS : public ubloxGPS
{
public:

    enum
      {
        GETTING_STATUS, 
        GETTING_LEAP_SECONDS, 
        GETTING_UTC, 
        RUNNING
      }
        state NEOGPS_BF(8);

    MyGPS( Stream *device ) : ubloxGPS( device )
    {
      state = GETTING_STATUS;
    }

    //--------------------------

    void get_status()
    {
      static bool acquiring = false;

      if (fix().status == gps_fix::STATUS_NONE) {
        static uint32_t dotPrint;
        bool            requestNavStatus = false;

        if (!acquiring) {
          acquiring = true;
          dotPrint = millis();
          DEBUG_PORT.print( F("Acquiring...") );
          requestNavStatus = true;

        } else if (millis() - dotPrint > 1000UL) {
          dotPrint = millis();
          DEBUG_PORT << '.';

          static uint8_t requestPeriod;
          if ((++requestPeriod & 0x07) == 0)
            requestNavStatus = true;
        }

        if (requestNavStatus)
          // Turn on the UBX status message
          enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_STATUS );

      } else {
        if (acquiring)
          DEBUG_PORT << '\n';
        DEBUG_PORT << F("Acquired status: ") << (uint8_t) fix().status << '\n';

        #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
            defined(UBLOX_PARSE_TIMEGPS)

          if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
            DEBUG_PORT.println( F("enable TIMEGPS failed!") );

          state = GETTING_LEAP_SECONDS;
        #else
          start_running();
          state = RUNNING;
        #endif
      }
    } // get_status

    //--------------------------

    void get_leap_seconds()
    {
      #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
          defined(UBLOX_PARSE_TIMEGPS)

        if (GPSTime::leap_seconds != 0) {
          DEBUG_PORT << F("Acquired leap seconds: ") << GPSTime::leap_seconds << '\n';

          if (!disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
            DEBUG_PORT.println( F("disable TIMEGPS failed!") );

          #if defined(UBLOX_PARSE_TIMEUTC)
            if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
              DEBUG_PORT.println( F("enable TIMEUTC failed!") );
            state = GETTING_UTC;
          #else
            start_running();
          #endif
        }
      #endif

    } // get_leap_seconds

    //--------------------------

    void get_utc()
    {
      #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
          defined(UBLOX_PARSE_TIMEUTC)

        lock();
          bool            safe = is_safe();
          NeoGPS::clock_t sow  = GPSTime::start_of_week();
          NeoGPS::time_t  utc  = fix().dateTime;
        unlock();

        if (safe && (sow != 0)) {
          DEBUG_PORT << F("Acquired UTC: ") << utc << '\n';
          DEBUG_PORT << F("Acquired Start-of-Week: ") << sow << '\n';

          start_running();
        }
      #endif

    } // get_utc

    //--------------------------

    void start_running()
    {
      bool enabled_msg_with_time = false;

      #if defined(UBLOX_PARSE_POSLLH)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH ))
          DEBUG_PORT.println( F("enable POSLLH failed!") );

        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_PVT)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_PVT ))
          DEBUG_PORT.println( F("enable PVT failed!") );

        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_VELNED)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED ))
          DEBUG_PORT.println( F("enable VELNED failed!") );

        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_DOP)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_DOP ))
          DEBUG_PORT.println( F("enable DOP failed!") );
        else
          DEBUG_PORT.println( F("enabled DOP.") );

        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_SVINFO)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_SVINFO ))
          DEBUG_PORT.println( F("enable SVINFO failed!") );
        
        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_TIMEUTC)

        #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
          if (enabled_msg_with_time &&
              !disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
            DEBUG_PORT.println( F("disable TIMEUTC failed!") );

        #elif defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
          // If both aren't defined, we can't convert TOW to UTC,
          // so ask for the separate UTC message.
          if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
            DEBUG_PORT.println( F("enable TIMEUTC failed!") );
        #endif

      #endif

      state = RUNNING;
      trace_header( DEBUG_PORT );

    } // start_running

    //--------------------------

    bool running()
    {
      switch (state) {
        case GETTING_STATUS      : get_status      (); break;
        case GETTING_LEAP_SECONDS: get_leap_seconds(); break;
        case GETTING_UTC         : get_utc         (); break;
      }

      return (state == RUNNING);

    } // running

} NEOGPS_PACKED;

// Construct the GPS object and hook it to the appropriate serial device
static MyGPS gps( &gpsPort );

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  static void GPSisr( uint8_t c )
  {
    gps.handle( c );
  }
#endif

//--------------------------

static void configNMEA( uint8_t rate )
{
  for (uint8_t i=NMEAGPS::NMEA_FIRST_MSG; i<=NMEAGPS::NMEA_LAST_MSG; i++) {
    ublox::configNMEA( gps, (NMEAGPS::nmea_msg_t) i, rate );
  }
}

//--------------------------

static void disableUBX()
{
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_DOP );
}

*/
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

float roll_ref = 0;
float pitch_ref = 0;
float yaw_ref = 0;


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
