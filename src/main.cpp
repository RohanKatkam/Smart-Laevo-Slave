/*
  TODO : update the description comments.
  Testing sketch that publishes a message on a MQTT broker, for storing it on a database afterwards.

  The circuit:
  - MKR NB 1500 board
  - Antenna
  - SIM card with a data plan

  Largely inspired by :

  https://create.arduino.cc/projecthub/Arduino_Genuino/securely-connecting-an-arduino-nb-1500-to-azure-iot-hub-af6470
  
*/

#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <MKRNB.h>

#include "arduino_secrets.h"
#include "slaveSPI.h"

/////// Enter your sensitive data in arduino_secrets.h
const char pinnumber[]   = SECRET_PINNUMBER;
const char broker[]      = SECRET_BROKER;
const int  deviceId      = SECRET_DEVICE_ID;

/*Data buffer, storing the data segments.
The stored data takes the following form :
index 0 : state tag;
index 1 : duration of the state.*/
int _data[BUF_SIZE][2];
const int slavePin = 4;
byte buf[1] = {0};

/*should be equal to the column count of the data buffer, i.e. the total
amount of data segments. Note that in most languages, arrays start
at 0, meaning the last data segment in the data buffer is located
at data[_dataCount-1].*/
size_t _dataCount;

NB nbAccess;
GPRS gprs;

NBClient      nbClient;            // Used for the TCP socket connection
BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

unsigned long lastMillis = 0;

void connectNB();
void connectMQTT();
void publishData(uint8_t buf_dat);
void clearDataBuffer();
unsigned long getTime();


void setup() 
{
  Serial.begin(9600);
  while (!Serial);

  if (!ECCX08.begin()) 
  {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  // reconstruct the self signed cert
  ECCX08SelfSignedCert.beginReconstruction(0, 8);
  ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
  ECCX08SelfSignedCert.endReconstruction();

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  // Set the client id used for MQTT as the device id
  char string_deviceId[BUF_SIZE];
  itoa(deviceId, string_deviceId, 10); //set the string to the value of deviceId in base 10
  mqttClient.setId(string_deviceId);

  mqttClient.setUsernamePassword(SECRET_USERNAME, SECRET_PASSWORD);
  
  connectNB();
  connectMQTT();

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();
  _dataCount = 0;

  pinMode(slavePin, INPUT_PULLUP);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);

  Serial.begin(115200);
  Serial.println("Slave Initialistion underway!");
  Sercom1init();
  Serial.println("Slave Sercom 1 is initialised");
    
  // Good information on pin modes: https://www.arduino.cc/en/Tutorial/Foundations/DigitalPins
  attachInterrupt(digitalPinToInterrupt(slavePin), SERCOM1_Handler, FALLING);
}

void loop() 
{
  /*TODO : Poll for master's input*/
  /*TODO : check for connection only for sending messages ?
  Send messages while we are connected ?*/
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) 
    connectNB();

  if (!mqttClient.connected()) 
    connectMQTT();

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();

  // publish a message roughly every minute.
  if (millis() - lastMillis > 60000)
  {
    Serial.println("Sending buffer data!");
    publishData(buf[0]);
    clearDataBuffer();
    lastMillis = millis();
  }
}


void SERCOM1_Handler(){
  uint8_t data = 0;
  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; //Read SPI interrupt register
  // buf[0] = data;
  #ifdef DEBUG
    Serial.println("In SPI Interrupt");
    Serial.print("Interrupt Flag: "); 
    Serial.println(interrupts);
  #endif

  if (SERCOM1->SPI.INTFLAG.bit.SSL && SERCOM1->SPI.INTENSET.bit.SSL) {
    // FLAG IS SET WHEN nSS IS DETECTED GOING LOW
    // you can initialize the variables here, probably start you connection here
    SERCOM1->SPI.DATA.reg = 0xAB;											// preload shift register with first outgoing data			
		SERCOM1->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_SSL;
  }

  if (SERCOM1->SPI.INTFLAG.bit.DRE && SERCOM1->SPI.INTENSET.bit.DRE){
    // no need to send any data to the master for now
    SERCOM1->SPI.DATA.reg = 0xAC; // this is dummy write
  }

  if (SERCOM1->SPI.INTFLAG.bit.TXC && SERCOM1->SPI.INTFLAG.bit.TXC) {
    // this bit is set when SS is pulled high
    SERCOM1->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_TXC;
  }


  if (SERCOM1->SPI.INTFLAG.bit.RXC && SERCOM1->SPI.INTENSET.bit.RXC) {
    data = SERCOM1->SPI.DATA.bit.DATA; //Read data register
    Serial.print("DATA value is:   ");
    Serial.println(data);
    buf[0] = data; // copy data to buffer
    #ifdef DEBUG
      Serial.println("SPI Data Received Complete Interrupt");
      Serial.print("DATA in DEBUG: ");
      Serial.println(data);
    #endif
  }

  if (SERCOM1->SPI.INTFLAG.bit.ERROR && SERCOM1->SPI.INTENSET.bit.ERROR){
    SERCOM1->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_ERROR;
  }
  
  #ifdef DEBUG
    Serial.println("-----------------------------------");
  #endif
}

unsigned long getTime() 
{
  // get the current time from the cellular module
  return nbAccess.getTime();
}

void connectNB() 
{
  Serial.println("Attempting to connect to the cellular network");

  while ((nbAccess.begin(pinnumber) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY)) 
  {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }

  Serial.println("You're connected to the cellular network");
  Serial.println();
}

void connectMQTT() 
{
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) 
  {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();
}

void publishMessage(char message_buffer[BUF_SIZE]) 
{
  Serial.println("Publishing message");
  
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("messages"); //choose the topic
  
  mqttClient.print(message_buffer);
  
  mqttClient.endMessage();
}

void setMessageBuffer(int state, int duration, char message_buffer[])
{
  Serial.println("Setting up message...");
  Serial.println("Creating Json document...");
  StaticJsonDocument<BUF_SIZE> doc;
  Serial.println("Done.");

  doc["DeviceID"] = SECRET_DEVICE_ID;
  doc["State"] = state;
  doc["Duration"] = duration;

  Serial.println("Serializing the Json with the message buffer...");
  serializeJson(doc, message_buffer, BUF_SIZE);
  Serial.println("Done.");
}

void newDataReceived(int state_id)
{
  // First data received.
  if (_dataCount == 0)
  {
    _data[_dataCount][0] = state_id;
    _data[_dataCount][1] = millis();
    _dataCount++;
    return;
  }

  // Data lost.
  if (_dataCount >= BUF_SIZE)
  {
    Serial.println("Data lost!");
    _dataCount++; //still incrementing, for debugging matters.
    return;
  }

  // Closing previous data record by computing the duration of the state.
  _data[_dataCount-1][1] = millis() - _data[_dataCount-1][1];

  // Opening new data record.
  _data[_dataCount][0] = state_id;
  _data[_dataCount][1] = millis();
  
  _dataCount++;
}

void clearDataBuffer()
{
  Serial.println("Data being deleted...\n");
  for (int i = _dataCount; i >= 0; --i)
  {
    if (i <= BUF_SIZE - 1)
    {
      _data[i][0] = -1;
      _data[i][1] = -1;
    }
  }
  _dataCount = 0;
  Serial.println("Data deleted.\n");
}

void publishData(uint8_t buf_dat)
{
  int duration_for_publishing = millis();
  if(_dataCount > BUF_SIZE - 1)
    _dataCount = BUF_SIZE -1;

  // Closing last segment.
  //TODO : BUF_SIZE reached case.
  _data[_dataCount][1] = millis() - _data[_dataCount][1];

  char message_buffer[BUF_SIZE];

//   for (int i = 0; i < _dataCount; i++)
//   {
    int state = 5;
    int duration = buf_dat;
    setMessageBuffer(state, duration, message_buffer);
    publishMessage(message_buffer);
    memset(message_buffer, 0, BUF_SIZE);
//   }

  duration_for_publishing = millis() - duration_for_publishing;
  Serial.println("Duration for publishing the whole message :");
  Serial.println(duration_for_publishing);
}