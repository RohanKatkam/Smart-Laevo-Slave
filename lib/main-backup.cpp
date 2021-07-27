/**
 * Slave MKR code for SPI read from the Master and send data using MQTT and NB-IoT
 */ 
#include <Arduino.h>
#include <ML_IMU_inferencing.h>
#include <cstdarg>
#include <SPI.h>
#include <SerialFlash.h>
#include <string>

#include "main.h"
#include "arduino_secrets.h"
#include "headCrypto.h"
#include "slaveSPI.h"
#include "headMQTT_NB.h"

// initialize variables
byte buf[1];

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):

#define DEBUG

const int slavePin = 4;

void setup() {
  // put your setup code here, to run once:
  setupECCX08(&sslClient);

  setupMQTT_NB(&mqttClient, &nbAccess, &gprs);

  _currentTime = millis()/1000;
  _lastMessagePublished = _currentTime;

  initDataBuffer();


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

uint8_t circ_shift_left(uint8_t data, uint8_t n){
  return (data << n) | (data >> (8 - n));
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
    Serial.println(circ_shift_left(data, 1));
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


    // Get the SPI data

  // Send using MQTT over the NB network\
  // poll for new MQTT messages and send keep alives if still connected (useless otherwise)
  if (nbAccess.status() == NB_READY && gprs.status() == GPRS_READY &&
      mqttClient.connected())
    mqttClient.poll();

  _currentTime = millis()/1000;

  if (_currentTime - _lastMessagePublished > T_BETWEEN_MESSAGES)
  {
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) 
    {
      endCurrentDataSegment();
      createNewDataSegment(NOT_MEASURING);
      connectNB(&nbAccess, &gprs);
      endCurrentDataSegment();
    }

    if (!mqttClient.connected())
    {
      endCurrentDataSegment();
      createNewDataSegment(NOT_MEASURING);
      // connectMQTT();
      endCurrentDataSegment();
    } 

    if (nbAccess.status() == NB_READY && gprs.status() == GPRS_READY &&
      mqttClient.connected())
    {
      endCurrentDataSegment();
      // String message = makeMessageFromData();
      String message = String(buf[0]);
      Serial.println(message.c_str());
      Serial.println("\n");
      publishMessage(message);
      clearDataBuffer();
      initDataBuffer();
    }
    /*Even if the board isn't connected, we set up the time of _lastMessagePublished
    so that it retries connecting in T_BETWEEN_MESSAGES, and still measures in between.*/
    _lastMessagePublished = _currentTime;
    _lastDetectedState = 20;
  }


}

void loop() {
  // put your main code here, to run repeatedly
  Serial.print("Buffer data: ");
  Serial.println(buf[0]);
  delay(1000);
}