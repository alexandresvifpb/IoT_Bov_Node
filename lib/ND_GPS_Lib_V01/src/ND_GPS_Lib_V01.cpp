#include "ND_GPS_Lib_V01.h"

TinyGPSPlus gpsModule;
data_gps_t lastPosition;
data_gps_t currentPosition;
boolean datasUpdate = false;
String gps_device_id;

GPSLib::GPSLib() {}

// Inicializa a porta Serial do GPS e verifica se o modulo gps esta conectado 
boolean GPSLib::begin(String id) {
  boolean result = false;

  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);

  uint8_t cont = 1;
  while (cont > 0) {
    if (Serial1) {
      result = true;
      break;
    }
        
    Serial.print(".");
    cont++;
    delay(10);
   }

  (result) ? Serial.println("GPS sensor initialized with SUCCESS") : Serial.println("GPS sensor initialized with FAULT");

  gps_device_id = id;

  return result;
}

//
void GPSLib::run(void) {
  while ( Serial1.available() ) {

    currentPosition.age = gpsModule.location.age();

    gpsModule.encode( Serial1.read() );
    if ( gpsModule.location.isUpdated() ) {

        // Serial.println(__LINE__);
        
      lastPosition = currentPosition;

      currentPosition.latitude = gpsModule.location.lat();
      currentPosition.longitude = gpsModule.location.lng();
      currentPosition.altitude = gpsModule.altitude.meters();
      currentPosition.speed = gpsModule.speed.kmph();
      currentPosition.course = gpsModule.course.deg();
      currentPosition.distanceBetweenTwoPoints = gpsModule.distanceBetween(lastPosition.latitude, lastPosition.longitude, currentPosition.latitude, currentPosition.longitude);
      currentPosition.age = gpsModule.location.age();
      currentPosition.satellites = gpsModule.satellites.value();
      currentPosition.hdop = gpsModule.hdop.value();

      tmElements_t t;
      t.Year = gpsModule.date.year() - 1970;
      t.Month = gpsModule.date.month();
      t.Day = gpsModule.date.day();
      t.Hour = gpsModule.time.hour();
      t.Minute = gpsModule.time.minute();
      t.Second = gpsModule.time.second();
      currentPosition.epochTime = makeTime(t) + GPS_UTC_OFFSET * 3600;

      datasUpdate = true;
    }
  }
}

//
boolean GPSLib::new_record(void) {
  boolean ret = datasUpdate;
  datasUpdate = false;
  return ret;
}

// 
String GPSLib::get_record(long nowtime) {
  String result = "[";
  result += 1;
  result += ",";
  result += String((unsigned long)currentPosition.epochTime);
  result += ",";
  result += String((unsigned long)nowtime);
  result += ",";
  result += String(currentPosition.latitude, 6);
  result += ",";
  result += String(currentPosition.longitude, 6);
  result += ",";
  result += currentPosition.altitude;
  result += ",";
  result += currentPosition.speed;
  result += ",";
  result += currentPosition.course;
  result += ",";
  result += currentPosition.distanceBetweenTwoPoints;
  result += ",";
  result += currentPosition.age;
  result += ",";
  result += currentPosition.satellites;
  result += ",";
  result += currentPosition.hdop;
  result += "]";
  return result;
}

//
unsigned long GPSLib::epochtime(void) {
  return currentPosition.epochTime;
}