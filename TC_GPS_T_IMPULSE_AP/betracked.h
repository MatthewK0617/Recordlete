// GPS PIN
#define GPS_RX              PC11
#define GPS_TX              PC10
#define GPS_EN              PC6
#define GPS_RST             PB2
#define GPS_BAUD_RATE       115200
// LoRa
#define LORA_SCK            PB13
#define LORA_MISO           PB14
#define LORA_MOSI           PB15
#define LORA_NSS            PB12
#define LORA_RST            PB10
#define LORA_DIO0           PB11
#define LORA_BAND           915E6
#define RADIO_ANT_SWITCH_RXTX PA1
// SSD1306
#define MySDA               PB7      // ICM20948
#define MySCL               PB6      // ICM20948
#define OLED_RESET          PA8 // Reset pin # (or -1 if sharing Arduino reset pin)
// TOUCH
#define TTP223_VDD_PIN      PA2
#define TouchPad            PA0
// ICM20948
#define ICM20948_ADDR       0x69

#define PwrSwitchGPS        PA3
#define PwrSwitch1_8V       PB0
#define BatteryVol          PC4

#define DEBUG(x) Serial.print(x);
#define DEBUGLN(x) Serial.println(x);
#define DEBUG1 0
#define DISPLAY 0
#define METHOD1 0
#define METHOD2 0
#define METHOD3 1

#define DECLINATION_ANGLE   0.20f

//#define ssid ABCD
//#define password "12345678"

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

struct GeoLoc {
  float lat;
  float lon;
};


#define DEBUG(x) Serial.print(x);
#define DEBUGLN(x) Serial.println(x);

// GPS update period
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ "$PMTK220,10000*2F"  // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ "$PMTK220,5000*1B"   // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
//GPS Baud Rate
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_38400 "$PMTK251,38400*27"
#define PMTK_SET_BAUD_19200 "$PMTK251,19200*22"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
