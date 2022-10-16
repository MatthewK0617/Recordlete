/*
* Arduino Tracking System
*/
#include <SPI.h>
#include <LoRa.h>
#include <NMEAGPS.h>
#include "SSD1306.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "./TC_HMC5883L.h"
#include <TimeLib.h>
#include <Stepper.h>
//#include "SoftwareI2C.h"
#include "SoftWire.h"

//SoftwareI2C softwarei2c;
//SoftWire sw(23, 25);
//Wire sw(23, 25);
//SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);

SSD1306 display(0x3c, 21, 22);

// Compass
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// GPS
static NMEAGPS  gps;
static gps_fix  fix;
SoftwareSerial nss(GPS_TX_PIN, GPS_RX_PIN);
//SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
int gyear , gmonth , gdate, ghour , gminute , gsecond, pm, date_adj;
String date_str , time_str , lat_str , lng_str;
// To change update period and baud rate
//Adafruit_GPS GPS(&nss);

// Stepper Motor
//Define number of steps per rotation:
const int stepsPerRevolution = 2038;
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 13, 2, 15, 4);
//Stepper myStepper = Stepper(stepsPerRevolution, D5, D6, D7, D8);

unsigned long previousMillis = 0;
const long interval = 100; 
unsigned long start;

void CompassInfo(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void InitCompass() {
   /* Initialise the compass */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("no HMC5883 detected ... Check your wiring!");
    while(1) yield();
  }
  
  /* Display some basic information on this sensor */
  //CompassInfo();
}

// Get My Current GPS Location
GeoLoc GetMyLoc() {
  float flat, flng;
  bool newdata = false;
  GeoLoc myLoc;
  int totSat=0, traSat=0;
  
  while (!gps.available( nss )) {
    Serial.print("XXXXX\n");
    delay(0);
  }
  while (gps.available( nss )) {
    fix = gps.read();
    //Serial.println(gps.statistics.errors);
    totSat = gps.sat_count;
    for (int i = 0; i < totSat; i++) {
      if (gps.satellites[i].tracked) {
        traSat++;        
      }
      delay(0);
    }
//    Serial.print("Total Satellites: "); Serial.println(totSat);
//    Serial.print("Tracked Satellites: "); Serial.println(traSat);

    if (fix.valid.location) {
      //Serial.print("Fix: "); Serial.println(fix.satellites);
      flat = fix.latitude();
      flng = fix.longitude();
      newdata = true;
    }
    
//    if (fix.valid.time) {
//      ghour = fix.dateTime.hours - 7;
//      gminute = fix.dateTime.minutes;
//      gsecond = fix.dateTime.seconds;
//      
//    }
  }
    
  if (newdata) {
    myLoc.lat = flat;
    myLoc.lng = flng;
  }
  else {
    myLoc.lat = 0.0;
    myLoc.lng = 0.0;
  }
  return myLoc;
}

// Get AP's Current GPS Location
GeoLoc GetApLoc() {
  String apLat;
  String apLng;
  float apLatF;
  float apLngF;
  String apTime;
  String packet;
  int apLatI;
  int apLngI;
  String secur;
  int packetNum;
  int dl[4];

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      //Serial.print((char)LoRa.read());
      packet += (char)LoRa.read();
    }
    Serial.print(packet);
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    int j=0;
    for (int i = 0; i < packet.length(); i++) {
      if (packet.substring(i, i+1) == "/") {   
        dl[j] = i+1;
        j++;
      }
    }

    secur = packet.substring(0, dl[0]-1);
    packetNum = packet.substring(dl[0],dl[1]-1).toInt();
    apLatF = packet.substring(dl[1],dl[2]-1).toFloat()/exp(17);
    apLngF = packet.substring(dl[2],dl[3]-1).toFloat()/exp(17);
    Serial.print("secur: ");Serial.println(secur);
    Serial.print("packetNum: ");Serial.println(packetNum);
    Serial.print("apLat: ");Serial.println(apLatF,17);
    Serial.print("apLng: ");Serial.println(apLngF,17);
    
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
  
    display.drawString(0, 0, "Received packet: ");
    display.drawString(90, 0, String(packetNum));
    display.drawString(0, 12, "flat");
    display.drawString(30, 12, String(apLatF,17));
    display.drawString(0, 24, "flng");
    display.drawString(30, 24, String(apLngF,17));
    display.display();
  }
  
  GeoLoc apLoc;
  apLoc.lat = apLatF;
  apLoc.lng = apLngF;
  apLoc.secur = secur;
  return apLoc;
}

//float IRAM_ATTR CalHeading() {
float CalHeading() {
  sensors_event_t event;  
  mag.getEvent(&event);
  //Serial.print("X axis: "); Serial.print(event.magnetic.x); Serial.print(" uT   ");
  //Serial.print("Y axis: "); Serial.print(event.magnetic.y); Serial.print(" uT   ");
  //Serial.print("Z axis: "); Serial.print(event.magnetic.z); Serial.println(" uT");
  //float heading = atan2(event.magnetic.y, event.magnetic.x);
  float heading = atan2(event.magnetic.x, event.magnetic.y);
  heading += DEGTORAD;// * MAGNETIC_DECLINATION;
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  float headingDegree = heading * 180/M_PI;
  return headingDegree;
}

bool IRAM_ATTR DirectToNorth() {
  float hd = CalHeading();  
  while(1) {
    Serial.print("hd");Serial.println(hd);
    if (hd <= 2 || hd >= 358) {
      Serial.println("Facing North");
      //delay(500);
      return true;
    }
//    else if (hd > 2 && hd <= 180) {
//      Serial.println("5<PD<180");
//      myStepper.step(-1);  
//    }
//    else if (hd > 180 && hd < 358) {
//      Serial.println("180<PD<355");
//      myStepper.step(1);  
//    }
    myStepper.step(-1); 
    hd = CalHeading();
    yield();
  }
}

float CalDistance(struct GeoLoc &a, struct GeoLoc &b) {
  float delta_lat = (b.lat - a.lat) * DEGTORAD;
  float delta_lng = (b.lng - a.lng) * DEGTORAD;
  float ralat = a.lat * DEGTORAD;
  float rblat = b.lat * DEGTORAD;
  float x = sin(delta_lat/2) * sin(delta_lat/2) + cos(ralat) * cos(rblat) * sin(delta_lng/2) * sin(delta_lng/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return (EARTHRAD * y);
}

float CalBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float xx, yy, bb;
  
  //Calculate Bearing from Client(a) to Server(b)
  xx = cos(b.lat*DEGTORAD)*sin((b.lng-a.lng)*DEGTORAD);  
  yy = cos(a.lat*DEGTORAD)*sin(b.lat*DEGTORAD) - sin(a.lat*DEGTORAD)*cos(b.lat*DEGTORAD)*cos((b.lng-a.lng)*DEGTORAD);
  bb = atan2(xx,yy) * RADTODEG;
  return bb;
}

void setup() {   
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Receiver Callback");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);  
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.onReceive(cbk);
  LoRa.receive();
  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);  

  delay(1500);
  pinMode(GPS_TX_PIN, INPUT);
  pinMode(GPS_RX_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  // Stepper Motor
  // Set the speed to 5 rpm:
  myStepper.setSpeed(10);
  
  // Debugging
  Serial.begin(115200);
  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);

  nss.begin(9600);  
  //nss.begin(115200);
  
  // Compass
  InitCompass();  
  
  // Facing North
  DirectToNorth();
}

void loop() { 
  float bearing, heading, bearing_360;  
  float dist;
  float rot;
  int rot_step, pos, pos_ex;
  GeoLoc aLoc, bLoc;
  String secur;
  bLoc = GetApLoc();
  if (bLoc.secur == "cgnt71") {
    secur = "";
    aLoc = GetMyLoc();//(gps);

    if (aLoc.lat != 0 && aLoc.lng != 0 && bLoc.lat != 0 && bLoc.lng != 0) {
      Serial.print("Ap: "); Serial.print(bLoc.lat,7); Serial.println(bLoc.lng,7);
      Serial.print("My: "); Serial.print(aLoc.lat,7); Serial.println(aLoc.lng,7);
            
      dist = CalDistance(aLoc, bLoc);
      Serial.print("Distance: "); Serial.println(dist,7);
      bearing = CalBearing(aLoc, bLoc);
      Serial.print("Bearing: "); Serial.println(bearing,7);
      myStepper.step(1*10);//*pos_ex);
      heading = CalHeading();
      yield();
      Serial.print("Heading: "); Serial.println(heading,7);

      if (bearing < 0) {
        //rot = heading - bearing;
        //pos_ex = -1;
        bearing_360 = bearing + 360;
      }
      else {
        bearing_360 = bearing;
      }
      Serial.print("bearing_360: "); Serial.println(bearing_360,7);
      rot = bearing_360 - heading;
      // obtuse angle to acute angle to reduce travel.
      if (rot >= 0.0 && rot <= 180.0) {
        rot_step = abs(rot)*stepsPerRevolution/360;
        Serial.print("No. 1: "); Serial.println(rot);
        pos = -1;
      }
      else if (rot > 180.0 && rot < 360.0) {
        rot_step = abs(rot - 360)*stepsPerRevolution/360;
        Serial.print("No. 2: "); Serial.println(rot-360);
        pos = 1;
      }      
      else if (rot <= -0.0 && rot >= -180.0) {
        rot_step = abs(rot)*stepsPerRevolution/360;
        Serial.print("No. 3: "); Serial.println(rot);
        pos = 1;
      }
      else if (rot > -360.0 && rot < -180.0) {
        rot_step = abs(rot + 360)*stepsPerRevolution/360;
        Serial.print("No. 4: "); Serial.println(rot+360);
        pos = -1;
      }   

      Serial.print("rot: "); Serial.println(rot,7);
      Serial.print("pos: "); Serial.println(pos);
      Serial.print("rot_step: "); Serial.println(rot_step);
          display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
  
    display.drawString(0, 36, "D: ");
    display.drawString(20, 36, String(dist));
    display.drawString(60, 36, "B: ");
    display.drawString(80, 36, String(bearing));
    display.drawString(0, 48, "H: ");
    display.drawString(20, 48, String(heading));
    display.display();
      for (int i = 0; i < rot_step; i++) {
        myStepper.step(1*pos);//*pos_ex);
        yield();
      }
    }
  }
}
