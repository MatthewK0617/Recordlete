/*
* Arduino Tracking System
*/
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include "./betracked.h"
#include <TimeLib.h>
#include "SSD1306.h" 

unsigned int counter = 0;

SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet ;

// GPS
static NMEAGPS gps;
static gps_fix fix;
int totSat=0, traSat=0;
SoftwareSerial nss(GPS_TX_PIN, GPS_RX_PIN);
//HardwareSerial nss = Serial1;
//Adafruit_GPS GPS(&nss);

int gyear , gmonth , gdate, ghour , gminute , gsecond, pm;
float flat;
float flng;
float eflat;
float eflng;

String LAT, LNG;
struct gpsdata{
  String date_str;
  String time_str;
  String lat_str;
  String lng_str;
};
gpsdata sgps;

void setup() { 
  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT);
  
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
      
  Serial.begin(115200);  // Debugging
  //GPS.begin(9600);
  //GPS.sendCommand(PMTK_SET_BAUD_57600);  
  //GPS.sendCommand(PMTK_SET_BAUD_38400);
  //GPS.sendCommand("$PMTK251,38400*27");
  nss.begin(9600);
  delay(2000);
  //nss.println(PMTK_SET_BAUD_57600);
  //delay(50);
  //nss.end();
  //nss.begin(57600);
  //GPS.begin(38400);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  nss.println(PMTK_SET_NMEA_UPDATE_10HZ);
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
   
  delay(1500);  
}

void loop() { 
  int totSat=0, traSat=0;
  bool newdata = false;

  while (!gps.available( nss )) {
    //Serial.print("XXXXX\n");
    delay(0);
  }
  while (gps.available( nss )) {
  //if (nss.available()) {
    fix = gps.read();
    //Serial.println(gps.statistics.errors);
    totSat = gps.sat_count;
    
//    for (int i = 0; i < totSat; i++) {
//      if (gps.satellites[i].tracked) {
//        traSat++;        
//      }
//      delay(0);
//    }
//    Serial.print("Total Satellites: "); Serial.println(totSat);
//    Serial.print("Tracked Satellites: "); Serial.println(traSat);

    if (fix.valid.location) {
      //Serial.print("Fix: "); Serial.println(fix.satellites);
      flat = fix.latitude();
      flng = fix.longitude();
      newdata = true;
  

 //       Serial.print("Time - "); Serial.println(sgps.time_str);
 //       Serial.print("AP IP address: ");
//        Serial.println(myIP);
        //WiFiClient client;
        //server.handleClient();
        /*
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_10);
  
        display.drawString(0, 0, "Sending packet: ");
        display.drawString(90, 0, String(counter));
        display.drawString(0, 15, "flat");
        display.drawString(30, 15, String(flat,17));
        display.drawString(0, 30, "flng");
        display.drawString(30, 30, String(flng,17));
        Serial.println(String(counter));
        display.display();
*/
        //eflat = flat*exp(17);
        //eflng = flng*exp(17);
        String sflat = String(flat,9);
        String sflng = String(flng,9);
        // send packet
        LoRa.beginPacket();
        LoRa.print("cgnt71/");
        LoRa.print(counter);
        LoRa.print("/");
        LoRa.print(sflat);
        LoRa.print("/");
        LoRa.print(sflng);
        LoRa.endPacket();

        counter++;
        //digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
        //delay(1000);                       // wait for a second
        //digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
        //delay(1000);  
      }
    }
  
}
