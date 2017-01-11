#include <RTClib.h>
#include <Adafruit_MMA8451.h>
#include <SD.h>
#include <Wire.h>

#include <NeoSWSerial.h>
//#include <SoftwareSerial.h>
NeoSWSerial mySerial( 3, 2 );  // Change this to NeoSWSerial!

#define LOG_INTERVAL  1000 //millis between entries

#define REDLED 7
#define GREENLED 6
#define CHIPSELECT 10
#define SWITCHIN 5

//onboard LEDs
//const int redLED = 7;
//const int greenLED = 6;

//switch for inputs
//const int switchIn = 5;
//const int chipSelect = 10;

//------------------------------------
//  NeoGPS section
#include "NMEAGPS.h"

static NMEAGPS  gps;
static gps_fix fused;

static void GPSloop( uint8_t c )
{
  if (gps.decode( c ) == NMEAGPS::DECODE_COMPLETED) {

    // All enabled sentence types will be merged into one fix.
    //   This 'fused' data can be safely used anywhere in your program.

    fused |= gps.fix();
  }
} // GPSloop

//------------------------------------
// Declare stuff
RTC_DS1307       RTC;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Data variables
uint32_t logMillis = 0;
uint32_t greenOn;
uint32_t redOn;
enum state_t { WAITING_TO_LOG, GREEN_ON, BOTH_OFF }; // the FSM states
state_t state = WAITING_TO_LOG;
bool logfileOpen = false;


File newLog() {

  char filename[] = "LOG00.TXT";
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = i / 10 + '0';
    filename[4] = i % 10 + '0';

    if (! SD.exists(filename)) {
      break;
    }
  }

  File logfile =  SD.open(filename, FILE_WRITE);

  Serial.print("Logging to: ");
  Serial.println(filename);

  DateTime now;
  logMillis = millis();
  now = RTC.now();

  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.println(now.second(), DEC);

  logfile.print("G Range: ");
  logfile.println(2 << mma.getRange());

  logfile.println("millis,speed,lat,lon,alt,accx,accy,accz, orientation");

  return logfile;

}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println();

  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(SWITCHIN, INPUT_PULLUP);
  pinMode(CHIPSELECT, OUTPUT);

  digitalWrite(REDLED, HIGH);

  mySerial.begin(9600);
  mySerial.println( F("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28") );
  mySerial.println( F("PMTK220,200*2C") );
  
  mySerial.attachInterrupt( GPSloop );

  mma.begin();
  mma.setRange(MMA8451_RANGE_4_G);

  Serial.println("Initializing SD card");

  SD.begin(CHIPSELECT);

  Serial.println("card initialized");

  Wire.begin();
  if (!RTC.begin()) {
    Serial.println("RTC failed");
  }
}

  /*
  SIGNAL(TIMER0_COMPA_vect) {
  if(mySerial.available()) {
    GPSloop(mySerial.read());
  }
  }

  void useInterrupt() {
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  }
//  */

void loop()
{
  // This goes away when you switch to the interrupt-driven version
  //while (mySerial.available())
  //  GPSloop( mySerial.read() );

  //  No delays, just a Finite-state Machine like "blink without delay"

  if (!digitalRead(SWITCHIN)) {
    File logfile = newLog();
    digitalWrite(REDLED, LOW);

    while (!digitalRead(SWITCHIN)) {

      digitalWrite(GREENLED, HIGH);
      mma.read();

      logfile.print(millis() - logMillis);
      logfile.print(',');

      if (fused.valid.speed)
        logfile.print( fused.speed() );
      logfile.print(',');

      if (fused.valid.location) {
        logfile.print( fused.latitude(), 6 );
        logfile.print(',');
        logfile.print( fused.longitude(), 6 );
      } else {
        logfile.print(',');
      }
      logfile.print(',');

      if (fused.valid.altitude) {
        logfile.print( fused.altitude() );
        logfile.print(',');
      }

      logfile.print(mma.x);
      logfile.print(",");
      logfile.print(mma.y);
      logfile.print(",");
      logfile.print(mma.z);
      logfile.print(",");
      logfile.print(mma.getOrientation());

      logfile.println(",");

      delay(50);

      digitalWrite(GREENLED, LOW);

      delay(LOG_INTERVAL - 50);
    }

    logfile.close();
    digitalWrite(REDLED, HIGH);

  } //end if
}
