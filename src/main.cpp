/**
 * Slave MKR code for SPI read from the Master and send data using MQTT and NB-IoT
 */ 
#include <Arduino.h>
#include <ML_IMU_inferencing.h>
#include <cstdarg>
#include <SPI.h>
#include <SerialFlash.h>

#include "main.h"
#include "arduino_secrets.h"
#include "headCrypto.h"
#include "slaveSPI.h"

// initialize variables
byte buf[1];

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):

#define DEBUG

const int slavePin = 4;

void setup() {

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

void SERCOM1_Handler(){
  uint8_t data = 0;
  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; //Read SPI interrupt register
  buf[0] = data;
  #ifdef DEBUG
    Serial.println("In SPI Interrupt");
    Serial.print("Interrupt: "); 
    Serial.println(interrupts);
  #endif

  if(interrupts & (1<<3)) // 8 = 1000 = SSL
  {
    #ifdef DEBUG
      Serial.println("SPI SSL Interupt");
    #endif
    SERCOM1->SPI.INTFLAG.bit.SSL = 1; //clear slave select interrupt
    data = SERCOM1->SPI.DATA.reg; //Read data register
    #ifdef DEBUG
      Serial.print("DATA: "); Serial.println(data);
    #endif
    // SERCOM1->SPI.INTFLAG.bit.RXC = 1; //clear receive complete interrupt
  }
  
  // This is where data is received, and is written to a buffer, which is used in the main loop
  if(interrupts & (1<<2)) // 4 = 0100 = RXC
  {
    SERCOM1->SPI.INTFLAG.bit.RXC = 1; //clear receive complete interrupt
    data = SERCOM1->SPI.DATA.bit.DATA; //Read data register
    Serial.print("DATA value is:   ");
    Serial.println(data << 1);
    buf[0] = data; // copy data to buffer
    #ifdef DEBUG
      Serial.println("SPI Data Received Complete Interrupt");
      Serial.print("DATA in DEBUG: ");
      Serial.println(data);
    #endif
    
  }
  
  if(interrupts & (1<<1)) // 2 = 0010 = TXC
  {
    #ifdef DEBUG
      Serial.println("SPI Data Transmit Complete Interrupt");
    #endif
    SERCOM1->SPI.INTFLAG.bit.TXC = 1; //clear receive complete interrupt
  }
  
  if(interrupts & (1<<0)) // 1 = 0001 = DRE
  {
    #ifdef DEBUG
      Serial.println("SPI Data Register Empty Interrupt");
    #endif
    // SERCOM1->SPI.DATA.reg = 0xAA;
    // SERCOM1->SPI.INTFLAG.bit.DRE = 1;
  }
  
  #ifdef DEBUG
    Serial.println("----------");
  #endif
}

// For handler use SERCOM1 handler 

void loop() {
  // put your main code here, to run repeatedly
  Serial.print("Buffer data: ");
  Serial.println(buf[0]);
  delay(1000);
}