// GPS PIN
#define GPS_TX_PIN 34
#define GPS_RX_PIN 35

#define MOTOR_A_IN_1_PIN 13
#define MOTOR_A_IN_2_PIN 15
#define MOTOR_B_IN_1_PIN 2
#define MOTOR_B_IN_2_PIN 4

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  915E6

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define EARTHRAD 6371000 // km
// Find the magnetic declination at your locationto.
// http://www.magnetic-declination.com/
// Add it to the compass to adjust the magnetic field.
#define MAGNETIC_DECLINATION 11.38f

#define GPS_UPDATE_INTERVAL 2000

struct GeoLoc {
  float lat;
  float lng;
  String secur;
};

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
