#include "ND_LoRa_Lib_V01.h"

LinkedList<lora_send_t> list_lora_send = LinkedList<lora_send_t>();
LinkedList<lora_recv_t> list_lora_recv = LinkedList<lora_recv_t>();

String lora_device_id;
lora_temp_t message_temp;
boolean new_message_temp = false;
boolean device_connected = false;
int16_t rssi;
float snr;

LoRaNodeLib::LoRaNodeLib() {}

// LoRa module initialization function
boolean LoRaNodeLib::begin(String id) {

  SPI.begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_SS);
  // LoRa.setPins(SS, LORA_PIN_RST, LORA_PIN_DIO0);
  LoRa.setPins(LORA_PIN_SS, LORA_PIN_RST, LORA_PIN_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed. Check your connections.");
    return false;
  }

  if (synWord) {

    // Serial.println(__LINE__);

    LoRa.setSyncWord(synWord);                                          // ranges from 0-0xFF, default 0x34, see API docs
  }

  LoRa.enableCrc();
  LoRa.onReceive(onReceive);
  SetRxMode();

  lora_device_id = id;

  return true;
}

//
void LoRaNodeLib::run(void) {

  if ( new_message_temp ) {

    new_message_temp = false;

    // Serial.println(__LINE__);

    lora_recv_t _message_recv = decodeJSON_recv(message_temp.payload);
    _message_recv.RSSI = message_temp.RSSI;
    _message_recv.SNR = message_temp.SNR;

    if ( _message_recv.target_device_id == lora_device_id || ( _message_recv.target_device_id == "ffffff") || ( _message_recv.target_device_id == "FFFFFF") ) {

      // Serial.println(__LINE__);

      list_lora_recv.add(_message_recv);
    }

  }


    // para implementar uma rede mesh, completar o codigo para salvar mensagem recebidas que não seja para o node
    // em uma lista de mensagem a ser enviada para o gatway

}

//
void LoRaNodeLib::add_send_message(lora_send_t msg) {

  // Serial.println("ADD_LORA_SEND_MSG");

  list_lora_send.add(msg);
}

//
void LoRaNodeLib::add_recv_message(lora_recv_t msg) {

  // Serial.println("ADD_LORA_RECV_MSG");
  
  list_lora_recv.add(msg);
}

//
String LoRaNodeLib::get_next_send_message(void) {
  lora_send_t _send_msg = list_lora_send.remove(0);

  return encodeJSON_send(_send_msg);
}

//
lora_recv_t LoRaNodeLib::get_next_recv_message(void) {
  return list_lora_recv.remove(0);
}

// is the sending list empty
boolean LoRaNodeLib::is_send_list_empty(void) {
  return list_lora_send.size();
}

// is the receiving list empty
boolean LoRaNodeLib::is_recv_list_empty(void) {
  return list_lora_recv.size();
}

// is the receiving list empty
boolean LoRaNodeLib::is_connected(void) {
  return device_connected;
}

// 
boolean LoRaNodeLib::send_message(String message) {

  bool result = false;

  Serial.print("LORA_SEND: ");
  Serial.println(message);

  SetTxMode();                        // set tx mode
  LoRa.beginPacket();                 // start packet
  LoRa.print(message);                // add payload
  LoRa.endPacket();                   // finish packet and send it
  SetRxMode();                        // set rx mode

  result = true;
  return result;
}

//
void LoRaNodeLib::set_connected(bool status) {
  device_connected = status;
}

// Automatically executed every time the LoRa module receives a new message
void LoRaNodeLib::onReceive(int packetSize) {
  if (packetSize == 0) return;

  String _packetRecv = "";

  while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa

    _packetRecv += (char)LoRa.read();

  }

  if (_packetRecv != "") {

    // Serial.println(__LINE__);
    Serial.print("LORA_RECV: ");
    Serial.println(_packetRecv);

    message_temp.SNR = LoRa.packetSnr();
    message_temp.RSSI = LoRa.packetRssi();
    message_temp.payload = _packetRecv;
    new_message_temp = true;

  }

}

// Lora sets the module to work in RX mode
void LoRaNodeLib::SetRxMode(void) {
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

// Lora sets the module to work in TX mode
void LoRaNodeLib::SetTxMode(void) {
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

// Convert a messageLoRa_t to a String structure
String LoRaNodeLib::encodeJSON_recv(lora_recv_t msg) {

  String _strJSON;

  StaticJsonDocument<256> _doc;

  JsonObject _root = _doc.to<JsonObject>();

  _root["sd"] = msg.sourcer_device_id;
  _root["td"] = msg.target_device_id;
  _root["md"] = msg.message_id;
  _root["tp"] = msg.message_type;
  _root["nb"] = msg.neighbor_id_list;
  _root["hp"] = msg.hops;
  _root["sn"] = msg.SNR;
  _root["rs"] = msg.RSSI;
  _root["pl"] = msg.payload;

  if (serializeJson(_doc, _strJSON) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  return _strJSON;
}

// Convert a messageLoRa_t to a String structure
String LoRaNodeLib::encodeJSON_send(lora_send_t msg) {

  String _strJSON;

  StaticJsonDocument<256> _doc;

  JsonObject _root = _doc.to<JsonObject>();

  _root["sd"] = msg.sourcer_device_id;
  _root["td"] = msg.target_device_id;
  _root["md"] = msg.message_id;
  _root["tp"] = msg.message_type;
  _root["nb"] = msg.neighbor_id_list;
  _root["hp"] = msg.hops;
  _root["pl"] = msg.payload;

  if (serializeJson(_doc, _strJSON) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  return _strJSON;
}

//
lora_recv_t LoRaNodeLib::decodeJSON_recv(String strJSON) {
  lora_recv_t _message;

  StaticJsonDocument<200> _doc;

  DeserializationError error = deserializeJson(_doc, strJSON);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return _message;
  }

  JsonObject _root = _doc.as<JsonObject>();

  String _sourcer_device_id = _root["sd"];
  String _target_device_id = _root["td"];
  String _message_id = _root["md"];
  uint8_t _message_type = _root["tp"];
  String _neighbor_id_list = _root["nb"];
  uint8_t _hops = _root["hp"];
  float _snr = _root["sn"];
  float _rssi = _root["rs"];
  String _payload = _root["pl"];

  _message.sourcer_device_id = _sourcer_device_id;
  _message.target_device_id = _target_device_id;
  _message.message_id = _message_id;
  _message.message_type = _message_type;
  _message.neighbor_id_list = _neighbor_id_list;
  _message.hops = _hops;
  _message.SNR = _snr;
  _message.RSSI = _rssi;
  _message.payload = _payload;

  return _message;
}

//
lora_send_t LoRaNodeLib::decodeJSON_send(String strJSON) {
  lora_send_t _message;

  StaticJsonDocument<200> _doc;

  DeserializationError error = deserializeJson(_doc, strJSON);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return _message;
  }

  JsonObject _root = _doc.as<JsonObject>();

  String _sourcer_device_id = _root["sd"];
  String _target_device_id = _root["td"];
  String _message_id = _root["md"];
  uint8_t _message_type = _root["tp"];
  String _neighbor_id_list = _root["nb"];
  uint8_t _hops = _root["hp"];
  String _payload = _root["pl"];

  _message.sourcer_device_id = _sourcer_device_id;
  _message.target_device_id = _target_device_id;
  _message.message_id = _message_id;
  _message.message_type = _message_type;
  _message.neighbor_id_list = _neighbor_id_list;
  _message.hops = _hops;
  _message.payload = _payload;

  return _message;
}