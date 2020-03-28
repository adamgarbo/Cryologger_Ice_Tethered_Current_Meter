/*
  Title:    Cryologger - Ice Tethered Current Meter
  Date:     March 28, 2020
  Author:   Adam Garbo

    Components:
    - Adafruit Feather M0 Proto
    - Adafruit Ultimate GPS Featherwing
    - Rock Seven RockBLOCK 9603
    - Maxtena M1621HCT-P-SMA Iridium antenna
    - Adafruit IMU

  Comments:
  - Code is currently under development.
*/

// Libraries
#include <Arduino.h>                // https://github.com/arduino/ArduinoCore-samd (required before wiring_private.h)
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower
#include <IridiumSBD.h>             // https://github.com/PaulZC/IridiumSBD
#include <math.h>                   // https://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero
#include <LSM303.h>                 // https://github.com/pololu/lsm303-arduino
#include <Statistic.h>              // https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/wiring_private.h (required for pinPeripheral() function)

// Pin definitons
#define GPS_EN_PIN            A5    // GPS enable pin
#define ROCKBLOCK_RX_PIN      10    // RockBLOCK 9603 RX pin
#define ROCKBLOCK_TX_PIN      11    // RockBLOCK 9603 TX pin
#define ROCKBLOCK_SLEEP_PIN   12    // RockBLOCK 9603 sleep pin
#define VBAT_PIN              A7    // Battery voltage measurement pin

// Debugging constants
#define DEBUG         true  // Output debug messages to Serial Monitor
#define DIAGNOSTICS   true  // Output Iridium diagnostic messages to Serial Monitor
#define DEPLOY        false  // Disable debugging messages for deployment

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
RTCZero     rtc;
IridiumSBD  modem(IridiumSerial, ROCKBLOCK_SLEEP_PIN);
LSM303      imu; // I2C Address: 0x1E (Magnetometer), 0x6B (Accelerometer)
TinyGPSPlus gps;

// User defined global variable declarations
unsigned long alarmInterval           = 300;    // RTC sleep duration in seconds (Default: 3600 seconds)
unsigned int  sampleInterval          = 5;      // Sampling duration of current tilt measurements (seconds)
byte          sampleFrequency         = 1;      // Sampling frequency of current tilt measurements (seconds)
byte          transmitInterval        = 4;      // Number of messages in each Iridium transmission (340-byte limit)
byte          maxRetransmitCounter    = 1;      // Number of failed messages to reattempt in each Iridium transmission (340-byte limit)

// Global variable and constant declarations
bool          ledState                = LOW;    // Flag to toggle LED in blinkLed() function
volatile bool sleepFlag               = false;  // Flag to indicate to Watchdog Timer if in deep sleep mode
volatile bool alarmFlag               = false;  // Flag for alarm interrupt service routine
byte          resetFlag               = 0;      // Flag to force Watchdog Timer system reset
byte          transmitBuffer[340]     = {};     // RockBLOCK 9603 transmission buffer
unsigned int  messageCounter          = 0;      // RockBLOCK 9603 transmitted message counter
unsigned int  retransmitCounter       = 0;      // RockBLOCK 9603 failed data transmission counter
unsigned int  transmitCounter         = 0;      // RockBLOCK 9603 transmission interval counter
unsigned long previousMillis          = 0;      // Global millis() timer variable
unsigned long unixtime                = 0;      // UNIX Epoch time variable
unsigned long alarmTime               = 0;      // Epoch alarm time variable
tmElements_t  tm;

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
    unsigned long unixtime;           // Unix epoch time                (4 bytes)
    int           temperature;        // Temperature                    (2 bytes)
    int           axMean;             // Accelerometer x                (2 bytes)
    int           ayMean;             // Accelerometer y                (2 bytes)
    int           azMean;             // Accelerometer z                (2 bytes)
    int           axStdev;            // Accelerometer x                (2 bytes)
    int           ayStdev;            // Accelerometer y                (2 bytes)
    int           azStdev;            // Accelerometer z                (2 bytes)
    int           mxMean;             // Magnetometer x                 (2 bytes)
    int           myMean;             // Magnetometer y                 (2 bytes)
    int           mzMean;             // Magnetometer z                 (2 bytes)
    byte          gFlag;              // Gyroscope flag                 (1 byte)
    long          latitude;           // Latitude                       (4 bytes)
    long          longitude;          // Longitude                      (4 bytes)
    unsigned int  voltage;            // Battery voltage (mV)           (2 bytes)
    unsigned int  transmitDuration;   // Previous transmission duration (2 bytes)
    unsigned int  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                            // Total: 59 bytes
  byte bytes[39]; // To do: Look into flexible arrays in structures
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message); // Size (in bytes) of data message to be transmitted

// Setup
void setup() {

  // Pin assignments
  pinMode(GPS_EN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VBAT_PIN, INPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  //while (!Serial); // Prevent execution of script until Serial Monitor is open
  delay(5000);         // Delay to allow for opening of Serial Monitor

  // Configure Watchdog Timer
  configureWatchdog();

  Serial.println(F("Cryologger - Ice Tethered Current Meter"));
  Serial.println(F("---------------------------------------"));

  // I2C Configuration
  Wire.begin();           // Initialize I2C bus
  Wire.setClock(100000);  // Set I2C clock speed to 100kHz

  // Analog-to-digital converter (ADC) Configuration
  analogReadResolution(12); // Change the ADC resolution to 12 bits

  // Real-time clock (RTC) Configuration
  /*
    Alarm matches:
    MATCH_OFF          = RTC_MODE2_MASK_SEL_OFF_Val,          // Never
    MATCH_SS           = RTC_MODE2_MASK_SEL_SS_Val,           // Every Minute
    MATCH_MMSS         = RTC_MODE2_MASK_SEL_MMSS_Val,         // Every Hour
    MATCH_HHMMSS       = RTC_MODE2_MASK_SEL_HHMMSS_Val,       // Every Day
    MATCH_DHHMMSS      = RTC_MODE2_MASK_SEL_DDHHMMSS_Val,     // Every Month
    MATCH_MMDDHHMMSS   = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val,   // Every Year
    MATCH_YYMMDDHHMMSS = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val  // Once, on a specific date and a specific time
  */
  rtc.begin();                      // Initialize RTC
  rtc.setAlarmTime(0, 0, 0);        // Set alarm
#if DEBUG
  rtc.enableAlarm(rtc.MATCH_SS);    // Set alarm to seconds match
#else if DEPLOY
  rtc.enableAlarm(rtc.MATCH_MMSS);  // Set alarm to seconds and minutes match#endif
#endif
  rtc.attachInterrupt(alarmMatch);  // Attach alarm interrupt
  Serial.println(F("RTC initialized."));

  // IMU Configuration
  if (imu.init()) {
    Serial.println(F("IMU detected."));
  }
  else {
    Serial.println(F("Warning: IMU not detected. Please check wiring."));
  }

  // GPS Configuration

  // RockBLOCK 9603 Configuration
  if (modem.isConnected()) {
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
    modem.adjustSendReceiveTimeout(180); // Default = 300 seconds
    modem.adjustATTimeout(20);
    Serial.println(F("RockBLOCK 9603 detected."));
  }
  else {
    Serial.println(F("Warning: RockBLOCK 9603N not detected. Please check wiring."));
  }

  // Print current date and time
  Serial.print(F("Datetime: ")); printDatetime();

  // Print operating mode
  Serial.print(F("Mode: "));
#if DEBUG
  Serial.println(F("DEBUG"));
#else if DEPLOY
  Serial.println(F("DEPLOY"));
#endif

  // Blink LED to indicate setup has completed
  blinkLed(10, 100);

  // Set the RTC's date and time from GPS
  readGps();
}

// Loop
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
    Serial.print("Alarm trigger: "); printDatetime();

    /*
      // Perform measurements for 2 minutes
      unsigned long loopStartTime = millis();
      while (millis() - loopStartTime < sampleInterval * 1000UL) {
        petDog();         // Pet the Watchdog Timer
        readImu();        // Read IMU
        blinkLed(1, 100); // Blink LED
        //LowPower.sleep(1000); // Go to sleep to reduce current draw
      }

    */

    // Perform current measurements
    digitalWrite(LED_BUILTIN, HIGH);
    for (int i = 0; i < sampleInterval; i++) {
      petDog();         // Pet the Watchdog Timer
      readImu();        // Read IMU
#if DEBUG
      blinkLed(1, 100);
#else if DEPLOY
      // Go to sleep to reduce power consumption
      // To do: Investigate why the sleep duration appears to be double
      // (e.g. 1000 ms sleep = 2000 ms on the oscilloscope)
      LowPower.sleep(500);
#endif
    }
    digitalWrite(LED_BUILTIN, LOW);

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

    // Set RTC alarm
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
  sleepFlag = true; // Set Watchdog Timer sleep flag

#if DEBUG
  blinkLed(1, 500);
#endif

#if DEPLOY
  blinkLed(1, 10);
  LowPower.deepSleep(); // Enter deep sleep
#endif
}

// Measure battery voltage from 100/100 kOhm voltage divider
void readBattery() {

  // Start loop timer
  unsigned long loopStartTime = millis();
  float voltage = 0.0;

  // Average measurements
  for (byte i = 0; i < 10; ++i) {
    voltage += analogRead(VBAT_PIN);
    delay(1);
  }
  voltage /= 10;    // Average measurements
  voltage *= 2;     // Divided by 2, so multiply back
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 4096;  // Convert to voltage

  // Add to statistics object
  batteryStats.add(voltage);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print("readBattery() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read RTC
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
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" μs"));
}

// RTC alarm interrupt service routine
void alarmMatch() {
  alarmFlag = true; // Set alarm flag
}

// Print RTC date and time
void printDatetime() {
  char datetimeBuffer[20];
  snprintf(datetimeBuffer, sizeof(datetimeBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  Serial.println(datetimeBuffer);
}

// Print RTC alarm
void printAlarm() {
  char alarmBuffer[20];
  snprintf(alarmBuffer, sizeof(alarmBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           rtc.getAlarmYear() + 2000, rtc.getAlarmMonth(), rtc.getAlarmDay(),
           rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  Serial.println(alarmBuffer);
}

// Read IMU
void readImu() {

  // Start loop timer
  unsigned long loopStartTime = micros();

  float ax, ay, az, mx, my, mz, gx, gy, gz;
  /*
    Insert IMU code here
  */

  // Add to statistics object
  axStats.add(imu.a.x);
  ayStats.add(imu.a.y);
  azStats.add(imu.a.z);
  mxStats.add(imu.m.x);
  myStats.add(imu.m.y);
  mzStats.add(imu.m.z);
  //gxStats.add(imu.g.x);
  //gyStats.add(imu.g.y);
  //gzStats.add(imu.g.z);

  // Stop loop timer
  unsigned long loopEndTime = micros() - loopStartTime;
  Serial.print(F("readImu() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read GPS
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
  GpsSerial.println("$PGCMD,33,1*6C"); // Enable antenna updates
  GpsSerial.println("$PGCMD,33,0*6D"); // Disable antenna updates

  // Look for GPS signal for up to 5 minutes
  while (!fixFound && millis() - loopStartTime < 5UL * 60UL * 1000UL) {
    if (GpsSerial.available()) {
      charsSeen = true;
      char c = GpsSerial.read();
#if DEBUG
      //Serial.write(c);    // Echo NMEA sentences to serial
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

          if (fixCounter >= 10) {
            fixFound = true;

            // Sync RTC with GPS time
            rtc.setTime(gps.time.hour(), gps.time.minute(), gps.time.second());
            rtc.setDate(gps.date.day(), gps.date.month(), gps.date.year() - 2000);

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

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readGps() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Disable GPS
  digitalWrite(GPS_EN_PIN, HIGH);
}

// Calculate statistics and clear objects
void calculateStatistics() {

  // Write data to union
  message.axMean = axStats.average() * 100;
  message.ayMean = ayStats.average() * 100;
  message.azMean = azStats.average() * 100;
  message.mxMean = mxStats.average() * 100;
  message.myMean = myStats.average() * 100;
  message.mzMean = mzStats.average() * 100;
  //message.gxMean = gxStats.average() * 100;
  //message.gyMean = gyStats.average() * 100;
  //message.gzMean = gzStats.average() * 100;
  message.axStdev = axStats.pop_stdev() * 100;
  message.ayStdev = ayStats.pop_stdev() * 100;
  message.azStdev = azStats.pop_stdev() * 100;
  //message.mxStdev = mxStats.pop_stdev() * 100;
  //message.myStdev = myStats.pop_stdev() * 100;
  //message.mzStdev = mzStats.pop_stdev() * 100;
  //message.gxStdev = gxStats.pop_stdev() * 100;
  //message.gyStdev = gyStats.pop_stdev() * 100;
  //message.gzStdev = gzStats.pop_stdev() * 100;
  message.voltage = batteryStats.average() * 1000;

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

// Write union data to transmit buffer in preparation of data transmission
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

// Transmit data via the RockBLOCK 9603 transceiver
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
        byte resetFlagBuffer = (((byte)inBuffer[8] << 0) & 0xFF);
        unsigned int maxRetransmitCounterBuffer = (((unsigned int)inBuffer[7] << 0) & 0xFF) +  (((unsigned int)inBuffer[6] << 8) & 0xFFFF);
        unsigned int transmitIntervalBuffer = (((unsigned int)inBuffer[5] << 0) & 0xFF) + (((unsigned int)inBuffer[4] << 8) & 0xFFFF);
        unsigned long alarmIntervalBuffer = (((unsigned long)inBuffer[3] << 0) & 0xFF) + (((unsigned long)inBuffer[2] << 8) & 0xFFFF) +
                                            (((unsigned long)inBuffer[1] << 16) & 0xFFFFFF) + (((unsigned long)inBuffer[0] << 24) & 0xFFFFFFFF);

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
  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Write data to union
  message.transmitDuration = loopEndTime / 1000;

  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

  // Check if reset flag was transmitted
  if (resetFlag == 255) {
    while (1); // Wait for Watchdog Timer to reset system
  }
}

// Blink LED
void blinkLed(byte flashes, unsigned long interval) {
  byte i = 0;
  while (i == flashes) {
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
}

// Blink LED
void blinkLED(unsigned long interval) {
  digitalWrite(LED_BUILTIN, (millis() / interval) % 2 == 1 ? HIGH : LOW);
}

// RockBLOCK callback function
bool ISBDCallback() {
  petDog(); // Pet the Watchdog
  blinkLED(1000);
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

// Print statistics
void printStatistics() {
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Statistics"));
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Voltage:"));
  Serial.print(F("Samples: ")); Serial.print(batteryStats.count());
  Serial.print(F("\tMin: "));   Serial.print(batteryStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(batteryStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(batteryStats.average());
  Serial.print(F("\tSD: ")); Serial.println(batteryStats.unbiased_stdev());
}

// Print union/structure
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  //Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  //Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  //Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  //Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  //Serial.print(F("hdop:\t\t\t")); Serial.println(message.hdop);
  Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.println(F("-----------------------------------"));
}

// Print contents of union/structure
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

// Print contents of transmiff buffer array
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

// Configure the WDT to perform a system reset if loop() blocks for more than 8-16 seconds
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

// Pet the Watchdog Timer
void petDog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;        // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);   // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  if (sleepFlag) {
    sleepFlag = false;
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    while (true);                     // Force Watchdog Timer to reset the system
  }
}
