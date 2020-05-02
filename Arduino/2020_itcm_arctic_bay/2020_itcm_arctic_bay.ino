/*
  Title:    Cryologger - Ice-Tethered Current Meter
  Date:     May 2, 2020
  Author:   Adam Garbo

  Description:
  - An ice tethered current meter intended for deployment in Arctic Bay, Nunavut.
  - Current measurements are recorded every 30 minutes for 2 minutes at 1-second intervals.
  - Data is transmitted via the Iridium satellite network every 8 hours.

  Components:
  - Adafruit Feather M0 Proto
  - Adafruit Ultimate GPS Featherwing
  - Rock Seven RockBLOCK 9603
  - Maxtena M1621HCT-P-SMA Iridium antenna
  - Adafruit Precision NXP 9-DOF Breakout Board
  - Adafruit Lithium Ion Battery Pack - 3.7V 6600mAh

  Comments:
  - Code is ready for testing. 
  - Currently set to transmit every 5 minutes. 
*/

// Libraries
#include <Adafruit_Sensor.h>      // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_FXAS21002C.h>  // https://github.com/adafruit/Adafruit_FXAS21002C
#include <Adafruit_FXOS8700.h>    // https://github.com/adafruit/Adafruit_FXOS8700
#include <Arduino.h>              // https://github.com/arduino/ArduinoCore-samd (required before wiring_private.h)
#include <ArduinoLowPower.h>      // https://github.com/arduino-libraries/ArduinoLowPower
#include <IridiumSBD.h>           // https://github.com/PaulZC/IridiumSBD
#include <math.h>                 // https://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
#include <RTCZero.h>              // https://github.com/arduino-libraries/RTCZero
#include <LSM303.h>               // https://github.com/pololu/lsm303-arduino
#include <Statistic.h>            // https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic
#include <TimeLib.h>              // https://github.com/PaulStoffregen/Time
#include <TinyGPS++.h>            // https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>                 // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>       // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/wiring_private.h (required for pinPeripheral() function)

// Pin definitons
#define GPS_EN_PIN            A5  // GPS enable pin
#define ROCKBLOCK_RX_PIN      10  // RockBLOCK 9603 RX pin
#define ROCKBLOCK_TX_PIN      11  // RockBLOCK 9603 TX pin
#define ROCKBLOCK_SLEEP_PIN   12  // RockBLOCK 9603 sleep pin
#define VBAT_PIN              A7  // Battery voltage measurement pin

// Debugging constants
#define DEBUG         false   // Output debug messages to Serial Monitor
#define DEBUG_GPS     false   // Echo NMEA sentences to Serial Monitor
#define DIAGNOSTICS   false   // Output Iridium diagnostic messages to Serial Monitor
#define DEPLOY        true    // Disable debugging messages for deployment

// Create a new Serial/UART instance, assigning it to pins 10 and 11
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN, SERCOM_RX_PAD_2, UART_TX_PAD_0);
#define IridiumSerial Serial2
#define GpsSerial     Serial1

// Attach the interrupt handler to the SERCOM
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// Object instantiations
Adafruit_FXOS8700   accelmag  = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro      = Adafruit_FXAS21002C(0x0021002C);
RTCZero             rtc;
IridiumSBD          modem(IridiumSerial, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus         gps;

// User defined global variable declarations
unsigned long alarmInterval           = 1800;   // Alarm sleep duration (Default: 1800 seconds)
unsigned int  sampleInterval          = 120;    // Sampling duration of current tilt measurements (Default: 120 seconds)
byte          sampleFrequency         = 1;      // Sampling frequency of current tilt measurements (Default: 1 second)
byte          transmitInterval        = 2;      // Number of messages sent in each Iridium transmission (340-byte limit)
byte          maxRetransmitCounter    = 2;      // Number of failed messages to reattempt in each Iridium transmission (340-byte limit)
byte          maxFixCounter           = 10;     // Minimum number of acquired GPS fixes

// Global variable and constant declarations
volatile bool alarmFlag               = false;  // Flag for alarm interrupt service routine
volatile byte watchdogCounter         = 0;      // Watchdog Timer trigger counter
bool          ledState                = LOW;    // Flag to toggle LED in blinkLed() function
bool          rtcFlag                 = false;  // Flag to indicate if RTC has been set
byte          gyroFlag                = 0;      // Gyroscope flag
byte          resetFlag               = 0;      // Flag to force Watchdog Timer system reset
byte          transmitBuffer[340]     = {};     // RockBLOCK 9603 transmission buffer
unsigned int  messageCounter          = 0;      // RockBLOCK 9603 transmitted message counter
unsigned int  retransmitCounter       = 0;      // RockBLOCK 9603 failed data transmission counter
unsigned int  transmitCounter         = 0;      // RockBLOCK 9603 transmission interval counter
int           errorValue              = 5;      // Gyroscope dps error threshold
int           warningValue            = 2;      // Gyroscope dps warning threshold
unsigned long previousMillis          = 0;      // Global millis() timer variable
unsigned long unixtime                = 0;      // UNIX Epoch time variable
unsigned long alarmTime               = 0;      // Epoch alarm time variable
tmElements_t  tm;
sensors_event_t aEvent, mEvent, gEvent;         // Internal measurement unit variables

// Statistics objects
Statistic batteryStats;
Statistic axStats;
Statistic ayStats;
Statistic azStats;
Statistic mxStats;
Statistic myStats;
Statistic mzStats;
Statistic gxStats;
Statistic gyStats;
Statistic gzStats;

// Structure and union to store and send data byte-by-byte via RockBLOCK
typedef union {
  struct {
    uint32_t  unixtime;           // Unix epoch time                (4 bytes)
    int16_t   axMean;             // Accelerometer x                (2 bytes)
    int16_t   ayMean;             // Accelerometer y                (2 bytes)
    int16_t   azMean;             // Accelerometer z                (2 bytes)
    int16_t   axStdev;            // Accelerometer x                (2 bytes)
    int16_t   ayStdev;            // Accelerometer y                (2 bytes)
    int16_t   azStdev;            // Accelerometer z                (2 bytes)
    int16_t   mxMean;             // Magnetometer x                 (2 bytes)
    int16_t   myMean;             // Magnetometer y                 (2 bytes)
    int16_t   mzMean;             // Magnetometer z                 (2 bytes)
    uint8_t   gyroFlag;           // Gyroscope flag                 (1 byte)
    int32_t   latitude;           // Latitude                       (4 bytes)
    int32_t   longitude;          // Longitude                      (4 bytes)
    uint16_t  voltage;            // Battery voltage (mV)           (2 bytes)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                        // Total: 37 bytes
  uint8_t bytes[37];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message); // Size (in bytes) of data message to be transmitted

//*****************************************************************************
//
// Setup
//
//*****************************************************************************
void setup() {

  // Pin assignments
  pinMode(GPS_EN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VBAT_PIN, INPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  // Start Serial at 115200 baud
  Serial.begin(115200);
  //while (!Serial); // Wait for user to open Serial Monitor
  delay(5000); // Delay to allow user to open Serial Monitor

  // Watchdog Timer Configuration
  configureWatchdog();

  Serial.println(F("---------------------------------------"));
  Serial.println(F("Cryologger - Ice-Tethered Current Meter"));
  Serial.println(F("---------------------------------------"));

  // I2C Configuration
  Wire.begin();           // Initialize I2C bus
  Wire.setClock(100000);  // Set I2C clock speed to 100kHz

  // Analog-to-digital converter (ADC) Configuration
  analogReadResolution(12); // Change the ADC resolution to 12 bits

  // Inertial Measurement Unit (IMU) Configuration
  if (accelmag.begin(ACCEL_RANGE_2G)) {
    Serial.println("FXOS8700 initialized.");
  }
  else {
    Serial.println("Warning: FXOS8700 not detected. Please check wiring.");
  }

  if (gyro.begin()) {
    Serial.println("FXAS21002C initialized.");
  }
  else {
    Serial.println("Warning: FXAS21002C not detected. Please check wiring.");
  }

  // RockBLOCK 9603 Configuration
  if (modem.isConnected()) {
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
    modem.adjustSendReceiveTimeout(120); // Default = 300 seconds
    modem.adjustATTimeout(20);
    Serial.println(F("RockBLOCK 9603 intialized."));
  }
  else {
    Serial.println(F("Warning: RockBLOCK 9603N not detected. Please check wiring."));
  }

  // Real-time clock (RTC) Configuration
  /*
    Alarm matches:
    MATCH_OFF          = Never
    MATCH_SS           = Every Minute
    MATCH_MMSS         = Every Hour
    MATCH_HHMMSS       = Every Day
    MATCH_DHHMMSS      = Every Month
    MATCH_MMDDHHMMSS   = Every Year
    MATCH_YYMMDDHHMMSS = Once, on a specific date and time
  */
  rtc.begin();                      // Initialize the RTC
  Serial.println(F("RTC initialized."));
  Serial.println(F("Syncing RTC date and time with GPS..."));
  readGps();                        // Set the RTC's date and time from GPS
  rtc.setAlarmTime(0, 0, 0);        // Set alarm
#if DEBUG
  rtc.enableAlarm(rtc.MATCH_SS);    // Set alarm to seconds match
  //rtc.enableAlarm(rtc.MATCH_MMSS);  // Set alarm to seconds and minutes match
#else if DEPLOY
  rtc.enableAlarm(rtc.MATCH_MMSS);  // Set alarm to seconds and minutes match
#endif
  rtc.attachInterrupt(alarmMatch);  // Attach alarm interrupt

  // Print RTC's alarm date and time
  Serial.print(F("Next alarm: ")); printAlarm();

  // Print operating mode
  Serial.print(F("Mode: "));
#if DEBUG
  Serial.println(F("DEBUG"));
#else if DEPLOY
  Serial.println(F("DEPLOY"));
#endif

  // Print current date and time
  Serial.print(F("Datetime: ")); printDateTime();

  // Blink LED to indicate setup has completed
  blinkLed(20, 100);
}

//*****************************************************************************
//
// Loop
//
//*****************************************************************************
void loop() {

  // Check if alarm interrupt service routine was triggered
  if (alarmFlag) {

    // Read the RTC
    readRtc();

    // Pet the Watchdog Timer
    petDog();

    // Read the battery voltage
    readBattery();

    // Print date and time
    Serial.print("Alarm trigger: "); printDateTime();

    // Perform current measurements
    for (int i = 0; i < sampleInterval; i++) {
      petDog(); // Pet the Watchdog Timer
      readImu(); // Read IMU
#if DEBUG
      blinkLed(1, 500);
#else if DEPLOY
      // Go to sleep to reduce power consumption
      // To do: Investigate why the sleep duration appears to be double
      // (e.g. 1000 ms sleep = 1500 ms on the oscilloscope)
      LowPower.sleep(500);
#endif
    }

    // Read GPS
    readGps();

    // Perform statistics on measurements
    printStatistics();
    calculateStatistics();

    // Write data to buffer
    writeBuffer();

    // Check if data should be transmitted
    if (transmitCounter == transmitInterval) {
      transmitData();       // Transmit data
      transmitCounter = 0;  // Reset transmit counter
    }

    // Set the RTC alarm
    alarmTime = unixtime + alarmInterval; // Calculate next alarm

    // Check if alarm was set in the past
    if (alarmTime <= rtc.getEpoch()) {
      Serial.println(F("Warning! Alarm set in the past."));
      alarmTime = rtc.getEpoch() + alarmInterval; // Calculate new alarm
      rtc.setAlarmTime(0, 0, 0);  // Set alarm to next hour rollover
      rtc.enableAlarm(rtc.MATCH_MMSS); // Set alarm to seconds and minutes match
    }
    else {
      rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0);
      rtc.setAlarmDate(day(alarmTime), month(alarmTime), year(alarmTime) - 2000);
      rtc.enableAlarm(rtc.MATCH_MMDDHHMMSS); // Set alarm to seconds, minutes, hours, day, and month match
    }
    rtc.attachInterrupt(alarmMatch);

    Serial.print(F("Next alarm: ")); printAlarm();
  }
  alarmFlag = false; // Clear alarm interrupt service routine flag

#if DEBUG
  blinkLed(1, 500);
#endif

#if DEPLOY
  blinkLed(1, 10);
  LowPower.deepSleep(); // Enter deep sleep
#endif
}

//*****************************************************************************
//
// Measure battery voltage from 100/100 kOhm voltage divider
//
//*****************************************************************************
void readBattery() {

  // Start loop timer
  unsigned long loopStartTime = millis();
  float voltage = 0.0;

  voltage = analogRead(VBAT_PIN); // Read voltage
  voltage *= 2;     // Divided by 2, so multiply back
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 4096;  // Convert to voltage

  // Add to statistics object
  batteryStats.add(voltage);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
#if DEBUG
  //Serial.print("readBattery() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));
#endif
}

//*****************************************************************************
//
// Read real-time clock
//
//*****************************************************************************
void readRtc() {

  // Start loop timer
  unsigned long loopStartTime = micros();

  // Get UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  message.unixtime = unixtime;
  Serial.print(F("Epoch time: ")); Serial.println(unixtime);


  // Stop loop timer
  unsigned long loopEndTime = micros() - loopStartTime;
#if DEBUG
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" μs"));
#endif
}

//*****************************************************************************
//
// RTC alarm interrupt service routine
//
//*****************************************************************************
void alarmMatch() {
  alarmFlag = true; // Set alarm flag
}

//*****************************************************************************
//
// Print RTC date and time
//
//*****************************************************************************
void printDateTime() {
  char datetimeBuffer[20];
  snprintf(datetimeBuffer, sizeof(datetimeBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  Serial.println(datetimeBuffer);
}

//*****************************************************************************
//
// Print RTC alarm
//
//*****************************************************************************
void printAlarm() {
  char alarmBuffer[20];
  snprintf(alarmBuffer, sizeof(alarmBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           rtc.getAlarmYear() + 2000, rtc.getAlarmMonth(), rtc.getAlarmDay(),
           rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  Serial.println(alarmBuffer);
}

//*****************************************************************************
//
// Read IMU
//
//*****************************************************************************
void readImu() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  // Bring accelerometer and gyroscope out of low-power (standby) mode
  accelmag.standby(0);
  gyro.standby(0);

  // Get a new sensor event
  accelmag.getEvent(&aEvent, &mEvent);  // Measured in m/s^2 and uTesla
  gyro.getEvent(&gEvent);               // Measured in rad/s

  // Add to statistics object
  axStats.add(aEvent.acceleration.x);
  ayStats.add(aEvent.acceleration.y);
  azStats.add(aEvent.acceleration.z);
  mxStats.add(mEvent.magnetic.x);
  myStats.add(mEvent.magnetic.y);
  mzStats.add(mEvent.magnetic.z);
  gxStats.add(gEvent.gyro.x);
  gyStats.add(gEvent.gyro.y);
  gzStats.add(gEvent.gyro.z);

  // Place accelerometer and gyroscope in low-power (standby) mode
  accelmag.standby(1);
  gyro.standby(1);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
#if DEBUG
  Serial.print(F("readImu() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
#endif
}

//*****************************************************************************
//
// Read GPS
//
//*****************************************************************************
void readGps() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  bool fixFound = false;
  bool charsSeen = false;
  byte fixCounter = 0;

  // Enable GPS
  digitalWrite(GPS_EN_PIN, LOW);

  Serial.println("Beginning to listen for GPS traffic...");
  GpsSerial.begin(9600);
  blinkLed(2, 500);

  // Configure GPS
  GpsSerial.println("$PMTK220,1000*1F"); // Set NMEA port update rate to 1Hz
  blinkLed(1, 100);
  GpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Set NMEA sentence output frequencies to GGA and RMC
  blinkLed(1, 100);
  //GpsSerial.println("$PGCMD,33,1*6C"); // Enable antenna updates
  GpsSerial.println("$PGCMD,33,0*6D"); // Disable antenna updates

  // Look for GPS signal for up to 2 minutes
  while (!fixFound && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
    if (GpsSerial.available()) {
      charsSeen = true;
      char c = GpsSerial.read();
#if DEBUG_GPS
      Serial.write(c); // Echo NMEA sentences to serial
#endif
      if (gps.encode(c)) {
        if ((gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) &&
            (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated())) {
          fixCounter++;
#if DEBUG
          char gpsDateTime[25];
          snprintf(gpsDateTime, sizeof(gpsDateTime), "%04u-%02d-%02dT%02d:%02d:%02d,",
                   gps.date.year(), gps.date.month(), gps.date.day(),
                   gps.time.hour(), gps.time.minute(), gps.time.second());
          Serial.print(gpsDateTime);
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(","));
          Serial.print(gps.location.lng(), 6);
          Serial.print(F(","));
          Serial.print(gps.satellites.value());
          Serial.print(F(","));
          Serial.println(gps.hdop.hdop(), 2);
#endif
          if (fixCounter >= maxFixCounter) {
            fixFound = true;

            // Sync RTC with GPS time
            rtc.setTime(gps.time.hour(), gps.time.minute(), gps.time.second());
            rtc.setDate(gps.date.day(), gps.date.month(), gps.date.year() - 2000);
            rtcFlag = true;
            Serial.print("RTC set: "); printDateTime();

            message.latitude = gps.location.lat() * 1000000;
            message.longitude = gps.location.lng() * 1000000;
          }
        }
      }
    }

    ISBDCallback();

    if ((millis() - loopStartTime) > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      break;
    }
  }
  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  // Disable GPS
  digitalWrite(GPS_EN_PIN, HIGH);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
#if DEBUG
  Serial.print(F("readGps() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
#endif
}

//*****************************************************************************
//
// Calculate statistics and clear objects
//
//*****************************************************************************
void calculateStatistics() {

  // Write data to union
  message.voltage = batteryStats.average() * 1000;
  message.axMean = axStats.average() * 100;
  message.ayMean = ayStats.average() * 100;
  message.azMean = azStats.average() * 100;
  message.axStdev = axStats.pop_stdev() * 100;
  message.ayStdev = ayStats.pop_stdev() * 100;
  message.azStdev = azStats.pop_stdev() * 100;
  message.mxMean = mxStats.average() * 100;
  message.myMean = myStats.average() * 100;
  message.mzMean = mzStats.average() * 100;
  //message.mxStdev = mxStats.pop_stdev() * 100;
  //message.myStdev = myStats.pop_stdev() * 100;
  //message.mzStdev = mzStats.pop_stdev() * 100;
  //message.gxMean = gxStats.average() * 100;
  //message.gyMean = gyStats.average() * 100;
  //message.gzMean = gzStats.average() * 100;
  //message.gxStdev = gxStats.pop_stdev() * 100;
  //message.gyStdev = gyStats.pop_stdev() * 100;
  //message.gzStdev = gzStats.pop_stdev() * 100;

  // Gyro measurement standard deviations
  int gxStdev, gyStdev, gzStdev;
  gxStdev = gxStats.pop_stdev() * 180 / PI;
  gyStdev = gyStats.pop_stdev() * 180 / PI;
  gzStdev = gzStats.pop_stdev() * 180 / PI;

  // Bitfield
  bool gyroBitfield[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  if (abs(gxStdev) > warningValue) {
    gyroBitfield[7] = 1;
  }
  if (abs(gyStdev) > warningValue) {
    gyroBitfield[6] = 1;
  }
  if (abs(gzStdev) > warningValue) {
    gyroBitfield[5] = 1;
  }
  if (abs(gxStdev) > errorValue) {
    gyroBitfield[4] = 1;
  }
  if (abs(gyStdev) > errorValue) {
    gyroBitfield[3] = 1;
  }
  if (abs(gzStdev) > errorValue) {
    gyroBitfield[2] = 1;
  }

  /*
    // Extra bits available
    if (abs(gxStdev) > errorValue) {
      bit_gy[1] = 1;
    }
    if (abs(gxStdev) > errorValue) {
      bit_gy[0] = 1;
    }
  */
  byte gyroFlag = 0; // Clear all bits

  // Write bits to flag
  for (int i = 0; i < 8; i++) {
    gyroFlag |= gyroBitfield[i] << i;
  }

  // Write data to union
  message.gyroFlag = gyroFlag;

  // Clear statistics objects
  axStats.clear();
  ayStats.clear();
  azStats.clear();
  mxStats.clear();
  myStats.clear();
  mzStats.clear();
  gxStats.clear();
  gyStats.clear();
  gzStats.clear();
  batteryStats.clear();
}

//*****************************************************************************
//
// Write union data to transmit buffer in preparation of data transmission
//
//*****************************************************************************
void writeBuffer() {
  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)),
         message.bytes, sizeof(message)); // Copy message to transmit buffer

#if DEBUG
  printUnion();
  //printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}

//*****************************************************************************
//
// Transmit data via the RockBLOCK 9603 transceiver
//
//*****************************************************************************
void transmitData() {

  // Start loop timer
  unsigned long loopStartTime = millis();
  unsigned int err;

  // Start the serial power connected to the RockBLOCK modem
  IridiumSerial.begin(19200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(ROCKBLOCK_RX_PIN, PIO_SERCOM);
  pinPeripheral(ROCKBLOCK_TX_PIN, PIO_SERCOM);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err == ISBD_SUCCESS) {
    byte inBuffer[240];  // Buffer to store incoming transmission (240 byte limit)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer)); // Clear inBuffer array

    /*
        // Test the signal quality
        int signalQuality = -1;
        err = modem.getSignalQuality(signalQuality);
        if (err != ISBD_SUCCESS) {
          Serial.print(F("SignalQuality failed: error "));
          Serial.println(err);
          return;
        }
      Serial.print(F("On a scale of 0 to 5, signal quality is currently: "));
      Serial.println(signalQuality);
    */

    // Transmit and receieve data in binary format
    Serial.println(F("Attempting to transmit data..."));
    err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

    // Check if transmission was successful
    if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

      // Check for incoming message. If no inbound message is available, inBufferSize will be zero
      if (inBufferSize > 0) {

        // Print inBuffer size and values of each incoming byte of data
        Serial.print(F("Inbound buffer size is: ")); Serial.println(inBufferSize);
        for (byte i = 0; i < inBufferSize; i++) {
          Serial.print(F("Address: "));
          Serial.print(i);
          Serial.print(F("\tValue: "));
          Serial.println(inBuffer[i], HEX);
        }

        // Recompose bits using bitshift
        uint8_t resetFlagBuffer             = (((uint8_t)inBuffer[8] << 0) & 0xFF);
        uint16_t maxRetransmitCounterBuffer = (((uint16_t)inBuffer[7] << 0) & 0xFF) +  (((uint16_t)inBuffer[6] << 8) & 0xFFFF);
        uint16_t transmitIntervalBuffer     = (((uint16_t)inBuffer[5] << 0) & 0xFF) + (((uint16_t)inBuffer[4] << 8) & 0xFFFF);
        uint32_t alarmIntervalBuffer        = (((uint32_t)inBuffer[3] << 0) & 0xFF) + (((uint32_t)inBuffer[2] << 8) & 0xFFFF) +
                                              (((uint32_t)inBuffer[1] << 16) & 0xFFFFFF) + (((uint32_t)inBuffer[0] << 24) & 0xFFFFFFFF);

        // Check if incoming data is valid
        if ((alarmIntervalBuffer >= 300 && alarmIntervalBuffer <= 1209600) &&
            (transmitIntervalBuffer >= 1 && transmitIntervalBuffer <= 24) &&
            (maxRetransmitCounterBuffer >= 0 && maxRetransmitCounterBuffer <= 24) &&
            (resetFlagBuffer == 0  || resetFlagBuffer == 255)) {

          // Update global variables
          alarmInterval = alarmIntervalBuffer;                // Update alarm interval
          transmitInterval = transmitIntervalBuffer;          // Update transmit interval
          maxRetransmitCounter = maxRetransmitCounterBuffer;  // Update max retransmit counter
          resetFlag = resetFlagBuffer;                        // Update force reset flag

          Serial.print(F("alarmInterval: ")); Serial.println(alarmInterval);
          Serial.print(F("transmitInterval: ")); Serial.println(transmitInterval);
          Serial.print(F("maxRetransmitCounter: ")); Serial.println(maxRetransmitCounter);
          Serial.print(F("resetFlag: ")); Serial.println(resetFlag);
        }
      }

    }
    else {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
    }
  }
  else {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      Serial.println(F("Warning: No modem detected. Please check wiring."));
    }
    return;
  }

  // If transmission or modem begin fails
  if (err != ISBD_SUCCESS) {
    retransmitCounter++;

    // Reset counter if retransmit counter is exceeded
    if (retransmitCounter >= maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Power down the modem
  Serial.println(F("Putting the RockBLOCK 9603 to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Close the serial port connected to the RockBLOCK modem
  IridiumSerial.end();

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
#if DEBUG
  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);
#endif

  // Write data to union
  message.transmitDuration = loopEndTime / 1000;

  // Check if reset flag was transmitted
  if (resetFlag == 255) {
    while (1); // Wait for Watchdog Timer to reset system
  }
}

//*****************************************************************************
//
// Non-blocking LED blink
//
//*****************************************************************************
void blinkLed(byte flashes, unsigned long interval) {
  byte i = 0;
  while (i < flashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

//*****************************************************************************
//
// Blink LED
//
//*****************************************************************************
void blinkLED() {
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
}

// RockBLOCK callback function
bool ISBDCallback() {
  petDog(); // Pet the Watchdog
  readBattery();
  blinkLED();
  return true;
}

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
}

//*****************************************************************************
//
// Print statistics
//
//*****************************************************************************
void printStatistics() {
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Statistics"));
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Voltage:"));
  Serial.print(F("Samples: ")); Serial.print(batteryStats.count());
  Serial.print(F("\tMin: "));   Serial.print(batteryStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(batteryStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(batteryStats.average());
  Serial.print(F("\tSD: ")); Serial.println(batteryStats.pop_stdev());
  Serial.println(F("ax:"));
  Serial.print(F("Samples: ")); Serial.print(axStats.count());
  Serial.print(F("\tMin: "));   Serial.print(axStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(axStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(axStats.average());
  Serial.print(F("\tSD: ")); Serial.println(axStats.pop_stdev());
  Serial.println(F("ay:"));
  Serial.print(F("Samples: ")); Serial.print(ayStats.count());
  Serial.print(F("\tMin: "));   Serial.print(ayStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(ayStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(ayStats.average());
  Serial.print(F("\tSD: ")); Serial.println(ayStats.pop_stdev());
  Serial.println(F("az:"));
  Serial.print(F("Samples: ")); Serial.print(azStats.count());
  Serial.print(F("\tMin: "));   Serial.print(azStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(azStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(azStats.average());
  Serial.print(F("\tSD: ")); Serial.println(azStats.pop_stdev());
  Serial.println(F("mx:"));
  Serial.print(F("Samples: ")); Serial.print(mxStats.count());
  Serial.print(F("\tMin: "));   Serial.print(mxStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(mxStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(mxStats.average());
  Serial.print(F("\tSD: ")); Serial.println(mxStats.pop_stdev());
  Serial.println(F("my:"));
  Serial.print(F("Samples: ")); Serial.print(myStats.count());
  Serial.print(F("\tMin: "));   Serial.print(myStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(myStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(myStats.average());
  Serial.print(F("\tSD: ")); Serial.println(myStats.pop_stdev());
  Serial.println(F("mz:"));
  Serial.print(F("Samples: ")); Serial.print(mzStats.count());
  Serial.print(F("\tMin: "));   Serial.print(mzStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(mzStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(mzStats.average());
  Serial.print(F("\tSD: ")); Serial.println(mzStats.pop_stdev());
  Serial.println(F("gx:"));
  Serial.print(F("Samples: ")); Serial.print(gxStats.count());
  Serial.print(F("\tMin: "));   Serial.print(gxStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(gxStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(gxStats.average());
  Serial.print(F("\tSD: ")); Serial.println(gxStats.pop_stdev());
  Serial.println(F("gy:"));
  Serial.print(F("Samples: ")); Serial.print(gyStats.count());
  Serial.print(F("\tMin: "));   Serial.print(gyStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(gyStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(gyStats.average());
  Serial.print(F("\tSD: ")); Serial.println(gyStats.pop_stdev());
  Serial.println(F("gz:"));
  Serial.print(F("Samples: ")); Serial.print(gzStats.count());
  Serial.print(F("\tMin: "));   Serial.print(gzStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(gzStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(gzStats.average());
  Serial.print(F("\tSD: ")); Serial.println(gzStats.pop_stdev());
}

//*****************************************************************************
//
// Print union/structure
//
//*****************************************************************************
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("axMean:\t\t\t")); Serial.println(message.axMean);
  Serial.print(F("ayMean:\t\t\t")); Serial.println(message.ayMean);
  Serial.print(F("azMean:\t\t\t")); Serial.println(message.azMean);
  Serial.print(F("axStdev:\t\t")); Serial.println(message.axStdev);
  Serial.print(F("ayStdev:\t\t")); Serial.println(message.ayStdev);
  Serial.print(F("azStdev:\t\t")); Serial.println(message.azStdev);
  Serial.print(F("mxMean:\t\t\t")); Serial.println(message.mxMean);
  Serial.print(F("myMean:\t\t\t")); Serial.println(message.myMean);
  Serial.print(F("mzMean:\t\t\t")); Serial.println(message.mzMean);
  Serial.print(F("gyroFlag:\t\t")); Serial.println(message.gyroFlag);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.println(F("-----------------------------------"));
}

//*****************************************************************************
//
// Print contents of union/structure
//
//*****************************************************************************
void printUnionBinary() {
  Serial.println(F("Union/structure "));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (unsigned int i = 0; i < sizeof(message); ++i) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(message.bytes[i], BIN);
  }
  Serial.println(F("-----------------------------------"));
}

//*****************************************************************************
//
// Print contents of transmiff buffer array
//
//*****************************************************************************
void printTransmitBuffer() {
  Serial.println(F("Transmit buffer"));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (unsigned int i = 0; i < sizeof(transmitBuffer); i++) {
    Serial.print(i);
    Serial.print(F("\t"));
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}

//*****************************************************************************
//
// Configure Watchdog Timer to perform a system reset if loop() blocks for more
// than 8-16 seconds
//
//*****************************************************************************
void configureWatchdog() {

  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |          // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);            // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |        // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K | // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);          // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |     // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;         // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                 // Set the Early Warning Interrupt Time Offset to 8 seconds // REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                       // Enable the Early Warning interrupt                       // REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                      // Set the WDT reset timeout to 16 seconds                  // REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                       // Enable the WDT in normal mode                            // REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

//*****************************************************************************
//
// Pet the Watchdog Timer
//
//*****************************************************************************
void petDog() {
  watchdogCounter = 0;              // Clear Watchdog Timer trigger counter
  WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
}

//*****************************************************************************
//
// Watchdog Timer interrupt service routine
//
//*****************************************************************************
void WDT_Handler() {
  // Permit a limited number of Watchdog Timer triggers before forcing system reset.
  if (watchdogCounter < 10) {
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    while (true);                     // Force Watchdog Timer to reset the system
  }
}
