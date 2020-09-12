#include "ESP32UtilLibV01.h"

esp_adc_cal_characteristics_t adc_cal;
const int vBatPin = VBAT_PIN;
uint8_t counterToEnterModeDeepSleep = 0;
float moving_average_yaw_array[MOVING_AVERAGE_ARRAY_SIZE];
float moving_average_yaw_value = 0;
float moving_average_pitch_array[MOVING_AVERAGE_ARRAY_SIZE];
float moving_average_pitch_value = 0;
float moving_average_roll_array[MOVING_AVERAGE_ARRAY_SIZE];
float moving_average_roll_value = 0;

// function constructor of the class ESP32Lib
ESP32UtilLib::ESP32UtilLib() {}

// uses the MAC of the ESP32 for generate the device ID
String ESP32UtilLib::get_MAC(void) {
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));
    Serial.printf("%08X\n",(uint32_t)chipID);

    uint8_t id8 = (uint8_t)(chipID>>24);
    (id8 > 9) ? sprintf(chipID_0, "%0X", id8) : sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    (id16 > 9) ? sprintf(chipID_1, "%0X", id16) : sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    (id32 > 9) ? sprintf(chipID_2, "%0X", id32) : sprintf(chipID_2, "0%0X", id32);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// Hello SHA 256 from ESP32learning
uint8_t ESP32UtilLib::hash(String str) {
  char payload[32];
  str.substring(1,32).toCharArray(payload, 32);

  uint8_t _hash = 0;
  for (uint8_t i = 0; i < 32; i++) {
    _hash += int(payload[i]);
  }
  
  return _hash;
}

// Read from flash memory
uint16_t ESP32UtilLib::getFlashUInt16(uint8_t address) {
    return word(EEPROM.read(address), EEPROM.read(address + 1));
}

// Writes to flash memory
void ESP32UtilLib::setFlashUInt16(uint8_t address, uint16_t value) {
    EEPROM.write(address,highByte(value));
    EEPROM.write(address + 1,lowByte(value));
}

// get next value of the sequence boot and update in the EEPROM
uint16_t ESP32UtilLib::get_boot_sequence(void) {
    // reset the bootsequence variable
    if ( RESET_CONT_BOOT ) {
        setFlashUInt16(0, 0);
        EEPROM.commit();
        Serial.print("EEPROM.read(): ");
    } else {
        uint16_t _bootseq = getFlashUInt16(0);
        setFlashUInt16(0, _bootseq + 1);
        EEPROM.commit();
        return _bootseq;
    }
}

//
float ESP32UtilLib::getVBat(void) {
  // return ( float(analogRead(vBatPin)) / 4095.0 ) * 20.0;
  return ( float(analogRead(vBatPin)) / 1820.0 ) * 20.0;
}

//
boolean ESP32UtilLib::checkBattery(void) {
  counterToEnterModeDeepSleep++;
  ( getVBat() < MINIMUM_BATTERY_VOLTAGE) ? counterToEnterModeDeepSleep++: counterToEnterModeDeepSleep = 0;

  if ( counterToEnterModeDeepSleep > 20 ) {
    Serial.print("esp_deep_sleep_start()");
    esp_deep_sleep_start();
    return true;
  } else {
    Serial.print("counterToEnterModeDeepSleep: ");
    Serial.println(counterToEnterModeDeepSleep);
    return false;
  }
}

//
float ESP32UtilLib::moving_average_yaw(float value) {
  float _accumulator = 0;
  for (int8_t i = MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    moving_average_yaw_array[i] = moving_average_yaw_array[i-1];
    _accumulator += moving_average_yaw_array[i];
  }

  moving_average_yaw_array[0] = value;
  _accumulator += value;

  moving_average_yaw_value = _accumulator/MOVING_AVERAGE_ARRAY_SIZE;
  return moving_average_yaw_value;
}

//
float ESP32UtilLib::moving_average_pitch(float value) {
  float _accumulator = 0;
  for (int8_t i = MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    moving_average_pitch_array[i] = moving_average_pitch_array[i-1];
    _accumulator += moving_average_pitch_array[i];
  }

  moving_average_pitch_array[0] = value;
  _accumulator += value;

  moving_average_pitch_value = _accumulator/MOVING_AVERAGE_ARRAY_SIZE;
  return moving_average_pitch_value;
}

//
float ESP32UtilLib::moving_average_roll(float value) {
  float _accumulator = 0;
  for (int8_t i = MOVING_AVERAGE_ARRAY_SIZE-1; i > 0; i--) {
    moving_average_roll_array[i] = moving_average_roll_array[i-1];
    _accumulator += moving_average_roll_array[i];
  }

  moving_average_roll_array[0] = value;
  _accumulator += value;

  moving_average_roll_value = _accumulator/MOVING_AVERAGE_ARRAY_SIZE;
  return moving_average_roll_value;
}

//
uint8_t ESP32UtilLib::classify_behavior(void) {
  uint8_t rest = 0;
  if ( moving_average_pitch_value > -20 && moving_average_pitch_value < 30) rest = 1;
  if ( moving_average_pitch_value > 35 && moving_average_pitch_value < 70) rest = 2;
  if ( moving_average_pitch_value > -60 && moving_average_pitch_value < -35) rest = 3;
  return rest;
}