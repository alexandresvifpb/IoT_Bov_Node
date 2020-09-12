#include "ND_SDCard_Lib_V01.h"

SPIClass spi_sdcard;
bool SDCardPresent = false;
LinkedList<SDCard_record_t> listSaveRecord = LinkedList<SDCard_record_t>();

SDCardLib::SDCardLib() {}

// SDCard module initialization
bool SDCardLib::begin(void) {

  pinMode(SD_PIN_SS, OUTPUT);    

  spi_sdcard.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_SS);
  spi_sdcard.setFrequency(4000000);

  uint8_t tryConnect = 30;

  do {
    SDCardPresent = SD.begin(SD_PIN_SS, spi_sdcard);
    tryConnect--;
    delay(100);
  } while (!SDCardPresent && tryConnect);

  uint8_t cardType = SD.cardType();
    
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    SDCardPresent = false;
  } else {
    SDCardPresent = true;
  }

  if (SDCardPresent) {
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
      Serial.println("MMC");
    } else if(cardType == CARD_SD){
      Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }
  }

  return SDCardPresent;
}

//
void SDCardLib::run(void) {

  // Serial.print(__FUNCTION__);
  // Serial.println(__LINE__);

  while ( listSaveRecord.size() > 0 ) {

    // Serial.println(__LINE__);

    String filename = "";
    SDCard_record_t record = listSaveRecord.remove(0);
    String strID = record.id.substring(4);

    // Serial.println(__LINE__);

    switch (record.type)
    {
      case TYPE_ALL:
        // Serial.println(__LINE__);

        filename = "/ALL" + strID + String(record.bootSequence) + ".dat";
        break;
        
      case TYPE_IMU:
        // Serial.println(__LINE__);

        filename = "/IMU" + strID + String(record.bootSequence) + ".dat";
        break;
        
      case TYPE_GPS:
        // Serial.println(__LINE__);

        filename = "/GPS" + strID + String(record.bootSequence) + ".dat";
        break;
        
      default:
        break;
    }

    // Serial.println(__LINE__);

    append_file(SD, filename.c_str(), record.payload.c_str());
  }
}

// check if the SDCard is present
bool SDCardLib::is_SDCard_present(void) {
  return SDCardPresent;
}

//
bool SDCardLib::add_record(SDCard_record_t value) {

  // Serial.print(__FUNCTION__);
  // Serial.println(__LINE__);

  return listSaveRecord.add(value);
}

// open a file for reading and / or writing at the end of the file
bool SDCardLib::append_file(fs::FS &fs, const char * filename, const char * payload) {

  // Serial.println(__LINE__);

  File myFile = fs.open(filename, FILE_APPEND);
  bool ret = false;

  // Serial.println(__LINE__);

  if (myFile) {

    // Serial.println(__LINE__);

    if (myFile.println(payload)) {

      // Serial.println(__LINE__);

      ret = true;
    } else {

      // Serial.println(__LINE__);

      Serial.println("SDCard Erro: not write in file");
      ret = false;
    }

    // Serial.println(__LINE__);

    myFile.close();
  } else {

    // Serial.println(__LINE__);

    Serial.println("SDCard Erro: file not open");
    ret = false;
  }

  // Serial.println(__LINE__);
  return ret;
}
