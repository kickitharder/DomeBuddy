#define VERSION "\nDomeBuddy Dongle V4.210326 by keith.rickard@hotmail.com"
// For Arduino Micro (same chip as the Leonardo)
/*
  DomeBuddy Dongle sits between a PC's USB port and a Celestron NexStar hand controller.
  It presents itself as a COM port serial device on the PC with the setting 9600 baud, 8 data bits, 1 stop bit, no parity.
  It allows the PC to communicate with the telescope unhindered.  If the dongle has a Bluetooth connection with the
  DomeBuddy Controller, then every 5 second it will wait for the comms between the PC and telescope to be quiet before 
  it asks for information from the telescope.  The information will allow the dongle to determine its slew status and azimuth.
  The dongle will then transmit the azimuth, reduced to a value between 0 and 255, and slew status in encoded form.
  There are two buttons which when both pressed will reset the dongle.  Pressing the on marked "Debug" will make the dongle
  send diagnostic information the PC.
  
  There are two LEDs:
  
  Scope LED:
  Flashes every two seconds when trying to establish a connection with telescope and will remain bright when it does.
  
  Comms LED:
  Flashes dim and off when data flows between the PC and telescope, and flashes bright and dim when sending data to the
  controller via Bluetooth.
*/

// PARAMETERS
#include  <SoftwareSerial.h>

#define   EVENTDELAY    500
#define   SER_TIMEOUT   500
#define   SER1_TIMEOUT  3500
#define   BTSER_TIMEOUT 500
#define   BTBAUD        38400
#define   HCBAUD        9600
#define   LXBAUD        9600
#define   PCBAUD        9600
#define   UPDATE_HC     0       // 1 = update NexStar Controller, 0 = no update
#define   btRXD         A2      // BT module data in
#define   btTXD         A3      // BT module data out
#define   btSTAT        A1      // BT module status (1 = connected)
#define   btEN          A0      // BT module enable (1 = enter AT mode)
#define   LED2_GND      3 
#define   LED1_GND      4
#define   LED1          5       // COMMS LED
#define   LED2          6       // SCOPE LED
#define   DEBUG_PIN     8
#define   BRIGHT        255
#define   DIM           15
#define   OFF           0
#define   RAD2DEG       57.2957795130823

byte debug = 0;                     // 1 = Debug Mode active
bool scopeConnected = 0;            // 1 = DomeDongle has identified the scope it is connected to
byte hcAZM = 0;                     // Telescope's calculated azimuth (0 to 255, North = 0);
byte lastAZM = 0;
float hcRA;                         // Telescope's RA
float hcDEC;                        // Telescope's DEC
char hcSLEW;                        // 'S' = slewing,  'T' = tracking, 'L' = lost dongle connection
int hcYear;
int hcVersion;
byte hcMonth, hcDay;
int jdYear;
byte jdMonth, jdDay;
const float GPS_DATE_ADJ = 7168.0;  // Difference in days between 10 Mar 2021 and 25 Jul 2001.
unsigned long jdStart;
float hcTh0;                        // Sidereal time at Nexstar connection
float hcLat, hcLong;
byte buf[20];
unsigned long eventTimer;           // Time of last PC <> Telescope comms

// --------------------------------------------------------------------------------------------------
void setup() {
  pinMode(btSTAT, INPUT);
  pinMode(LED1_GND, OUTPUT);
  pinMode(LED2_GND, OUTPUT);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  digitalWrite(LED1_GND, LOW);          // Make pin ground for LED1
  digitalWrite(LED2_GND, LOW);          // Make pin ground for LED2
  analogWrite(LED1, BRIGHT);            // Light up the LED1
  analogWrite(LED2, BRIGHT);            // Light up the LED2
  Serial.begin(PCBAUD);                 // USB Serial - for PC
  Serial1.begin(HCBAUD);                // Pins 0 (RX) and 1 (TX) - for Hand Box (via MAX3232)
  debugButton();
  if (debug) while(!Serial);            // Wait for the USB serial port to become active
  if (debug) Serial.println(VERSION);
  delay(2000);
  if (debug == 1) serialTest();
  analogWrite(LED1, DIM);
}

// --------------------------------------------------------------------------------------------------
void loop() {
  identifyScope();              // Work out what scope is connected to the Dome Dongle
  serviceSerial();              // Allow comms between the PC and telescope for 5 seconds
  getData();                    // Get azimuth and slew status
  sendAZM();                    // Send azimuth info to DomeBuddy Controller
}

// ==================================================================================================
// SERVICE THE THROUGHPUT OF DATA BETWEEN THE PC AND TELESCOPE
// ==================================================================================================
void serviceSerial() {
  analogWrite(LED1, OFF);
  debugButton();                              // Check the Debug button
  if (!scopeConnected){                       // Quit if connection with the telescope has not been established
    while(Serial.available()) Serial.read();  // First clear any data received in the Serial buffers
    while(Serial1.available()) Serial1.read();
    return;
  }
  unsigned long timer = millis() + 5000;      // Set timer for how long data is allowed to pass through
  eventTimer = millis();
  do {
    while (millis() < timer) {
      serialThruPut();                        // Allow data to pass between PC and telesceope
      if (!debug) if (!digitalRead(btSTAT)) timer = millis() + 5000; // Keep resetting timer if no Bluetooth connection
      if (millis() > eventTimer) analogWrite(LED1, DIM);
    }                                         // Keep doing this for at least 5 seconds
    timer = eventTimer;
  } while (millis() < eventTimer);            // Wait for a preset time after last PC & scope comms
}

// --------------------------------------------------------------------------------------------------
void serialThruPut() {
  while (Serial.available() || Serial1.available()){        // Allow data to pass between the PC and telescope
    analogWrite(LED1, OFF);
    if (Serial.available()) Serial1.write(Serial.read());   // Send any PC data to the telescope
    if (Serial1.available()) Serial.write(Serial1.read());  // Send any telescope data to the PC
    debugButton();
    eventTimer = millis() + EVENTDELAY;
  }
}

// ==================================================================================================
// CHECK THE DEBUG BUTTON
// ==================================================================================================
bool debugButton() {
  if (!digitalRead(DEBUG_PIN)) {
    delay(1000);
    if (!digitalRead(DEBUG_PIN)) {
      debug ^= 1;
      if (debug) {
        Serial.println(VERSION);
        Serial.println("Debug mode");
      }
      else Serial.println("Exited Debug mode");
    }
  }
  return debug;
}

// ==================================================================================================
// GET DATA FROM TELESCOPE
// ==================================================================================================
void getData() {
  if (!scopeConnected) return;                              // Return if no established connection with telescope
  analogWrite(LED1, BRIGHT);                                // LED1 will remain bright if there is a timeout

// CHECK ARDUINO'S CLOCK STATUS
  if (millis() < jdStart) {
    if(debug) Serial.print("Arduino's clock has rolled back to zero");
    return setupClock();                                    // Reset the Adruino's clock and come back here later
  }

// GET THE NEXSTAR TELESCOPE'S EQUATORIAL COORDINATES
  if (debug) Serial.println("Getting equatorial coords");
  byte len = 0;
  int b = 0;
  hcSLEW = 'S';                                             // Start off assuming telescope is slewing
  Serial1.print("E");                                       // Get the RA and DEC
  eventTimer = millis() + 1000;                           // Wait upto 1000ms for a response
  while(!Serial1.available() && eventTimer > millis());

  do {
    eventTimer = millis() + 1000;                           // Wait upto 1000ms for a response
    while(!Serial1.available() && eventTimer > millis());
    if ((b = Serial1.read()) < 0) {
      if (debug) Serial.println("Timed out");               // Timed out!
      scopeConnected = 0;                                   // Lost connection with the telescope
      hcSLEW = 'L';                                         // L means lost connection
      hcAZM = eq2azm();                                     // Allow dome to continue tracking
      return;
    }
    buf[len] = b;                                           // Update buf to compare next time
    if (debug) Serial.print((char)buf[len]);
    len++;
  } while (b != '#' && len < 10);                           // Terminator byte
  if (debug) Serial.println();

  hcRA = hex2deg(0) ;                                       // Calculate the RA in degrees
  hcDEC = hex2deg(5);                                       // Calculate the DEC in degrees
  if (hcDEC > 180) hcDEC -= 360;
  hcAZM = eq2azm();
  
  if (hcAZM == lastAZM) hcSLEW = 'T';
  lastAZM = hcAZM;

  if (debug) {
    Serial.println(hcSLEW == 'S' ? "\nTelescope is slewing" : "\nTelescope is tracking");
    Serial.print("RA:  ");
    Serial.println(hcRA / 15, 8);
    Serial.print("DEC: ");
    Serial.println(hcDEC, 8);
    Serial.print("hcAZM: "); Serial.println(hcAZM);
  }
  analogWrite(LED1, DIM);                                   // Dim LED1 - comms with the telescope are finished
}

// --------------------------------------------------------------------------------------------------
float hex2deg(byte p) {
  float deg = 4096.0 * hex2byte(buf[p + 0]);                  // Convert hex text to degrees from position p
  deg += 256.0 * hex2byte(buf[p + 1]);
  deg += 16.0 * hex2byte(buf[p + 2]);
  deg += hex2byte(buf[p + 3]);

  return 360.0 * deg / 65536.0;
}

// --------------------------------------------------------------------------------------------------
byte hex2byte(char h) {
  return (byte)h - ((byte)h >= 'A' ? 55 : 48);                // Convert hex character into a number 0-15
}

// ==================================================================================================
// SEND DATA TO DOME CONTROLLER
// ==================================================================================================
void sendAZM(){
  if (!scopeConnected) return;
// Sends ':', 'S' or 'T', Xh, Xl, -Xh, -Xl, '#', e.g. for hcAZM  = 0, ":T00FF#" is sent
// hcSLEW is 'S' or 'T' and HCAZM is the scope's azimuth 0-255 (0 is due north)

  byte negAZM = hcAZM ^ 0xFF;               // Calculate NOT of scopeAZM
  
// SEND AZIMUTH TO SERIAL PORT
  if (debug) {
    Serial.print("Transmitting azimuth data ");
    Serial.print(':');                      // Azimuth is being sent, 0 degrees is due north
    Serial.print(hcSLEW);                   // Send slewing info ('S' = slewing, 'T' = tracking, '')
    if(hcAZM < 16 ) Serial.print('0');      // Send Azimuth as two hexdigit number
    Serial.print(hcAZM, HEX);               // Azimuth has been mapped between 0-255
    if(negAZM < 16 ) Serial.print('0');     // Send negative Azimuth as two hexdigit number
    Serial.print(negAZM, HEX);              // Sending this ensures azimuth is sent correctly
    Serial.print("# ");                     // Send terminator
    Serial.print(hcAZM);
  }

// SENT AZIMUTH TO BLUETOOTH
  if (!digitalRead(btSTAT)) {
    if (debug) Serial.println(" - no Bluetooth connection - no data sent");
    return;
  }
  SoftwareSerial btSerial(btTXD, btRXD);
  btSerial.begin(BTBAUD);
  btSerial.print(':');                      // Azimuth is being sent, 0 degrees is due north
  btSerial.print(hcSLEW);                   // Send slewing info ('S' = slewing, 'T' = tracking)
  if(hcAZM < 16 ) btSerial.print('0');      // Send Azimuth as two hexdigit number
  btSerial.print(hcAZM, HEX);               // Send only the HSB (0 - 255) i.e. 256 point resolution
  if(negAZM < 16 ) btSerial.print('0');     // Send negative Azimuth as two hexdigit number
  btSerial.print(negAZM, HEX);              // Sending this ensures azimuth is sent correctly
  btSerial.print('#');                      // Send terminator
  if (debug) Serial.println(" - sent");
}

// ==================================================================================================
// FIND OUT WHETHER CONNECTED TO NEXSTAR
// ==================================================================================================
void identifyScope() {
  analogWrite(LED2, BRIGHT);
  if (scopeConnected) return;

// TESTING FOR NEXSTAR HAND CONTROLLER
  analogWrite(LED2, DIM);
  if (debug) Serial.println("Determining connection\nChecking for NexStar");
  Serial1.begin(HCBAUD);
  Serial1.flush();                                      // Clear transmit buffer
  while (Serial1.available());                          // Clear receive buffer
  Serial1.print("V");                                   // Send command to get hand controller version NexStar
  delay(2000);

  if (Serial1.peek() >= 0) {
    if (!Serial1.readBytes(buf, 3)) return;             // Get repsonse
    if (buf[2] ==  '#') {                               // Valid if '#'' is 3rd character received
      analogWrite(LED2, BRIGHT);
      scopeConnected = 1;                               // Connected to a NexStar telesope
      if (debug) Serial.println("\nNexStar found");
      hcVersion = buf[0] * 100 + buf[1];                // Get the hand controller's firmware version
      setupClock();                                     // Start the Arduino's "Sideral Clock"
      return;
    }
  }
  analogWrite(LED2, OFF);
  delay(2000);
};

// ==================================================================================================
// INITIALISE ARDUINO SIDEREAL CLOCK (FOR NEXSTAR CONNECTION)
// ==================================================================================================
void setupClock() {
  
byte b[6][8] = {{'P', 1, 176, 55, 0, 0, 0, 1},          // Get GPS link status (>0 if inked, 0 if not)
                {'P', 1, 176, 1, 0, 0, 0, 3},           // Get GPS latitude
                {'P', 1, 176, 2, 0, 0, 0, 3},           // Get GPS longitude
                {'P', 1, 176, 4, 0, 0, 0, 2},           // Get GPS year
                {'P', 1, 176, 3, 0, 0, 0, 2},           // Get GPS month and day
                {'P', 1, 176, 51, 0, 0, 0, 3}};         // Get GPS hour, minutes & seconds

// GET GPS LATITUDE
    if (debug) Serial.println("Getting latitude"); 
  do {
    for (byte i = 0; i <= 7; i++) Serial1.write(b[1][i]); // Send commands for coords, year, date and time
    delay(100);
    if (!Serial1.readBytes(buf, 4)) {
      if(debug) Serial.println("Timed out");
      scopeConnected = 0;
      return;
    }
  } while (buf[3] != '#');                                // 4th byte must be a '#'
  hcLat =  360.0 * (float(buf[0]) * 65536.0 + float(buf[1]) * 256.0 + float(buf[2])) / 16777216.0 - (buf[0] >= 128) * 360.0;

// GET GPS LONGITUDE 
    if (debug) Serial.println("Getting longitude"); 
  do {
    for (byte i = 0; i <= 7; i++) Serial1.write(b[2][i]); // Send commands for coords, year, date and time
    delay(100);
    if (!Serial1.readBytes(buf, 4)) {
      if(debug) Serial.println("Timed out");
      scopeConnected = 0;
      return;
    }
  } while (buf[3] != '#');                                // 4th byte must be a '#'
  hcLong =  360.0 * (float(buf[0]) * 65536.0 + float(buf[1]) * 256.0 + float(buf[2])) / 16777216.0 - (buf[0] >= 128 ) * 360.0;

// GET GPS YEAR
  if (debug) Serial.println("Getting year"); 
  do {
    for (byte i = 0; i <= 7; i++) Serial1.write(b[3][i]); // Send commands for coords, year, date and time
    delay(100);
    if (!Serial1.readBytes(buf, 3)) {
      if(debug) Serial.println("Timed out");
      scopeConnected = 0;
      return;
    }
  } while (buf[2] != '#');                                // 3rd byte must be a '#'
  hcYear =  256 * buf[0] + buf[1];

 // GET GPS DATE
  if (debug) Serial.println("Getting date");
  do {
    for (byte i = 0; i <= 7; i++) Serial1.write(b[4][i]); // Send commands for coords, year, date and time
    delay(100);
    if (!Serial1.readBytes(buf, 3)) {
      if(debug) Serial.println("Timed out");
      scopeConnected = 0;
      return;
    }
  } while (buf[2] != '#');                                // 3rd byte must be a '#'
  hcMonth =  buf[0];
  hcDay = buf[1];

 // GET GPS TIME
  if (debug) Serial.println("Getting time");
  do {
    for (byte i = 0; i <= 7; i++) Serial1.write(b[5][i]); // Send commands for coords, year, date and time
    delay(100);
    if (!Serial1.readBytes(buf, 4)) {
      if(debug) Serial.println("Timed out");
      scopeConnected = 0;
      return;
    }
  } while (buf[3] != '#');                                    // 4th byte must be a '#'
  jdStart = millis();                                         // Get Arduino's time at connection

  int hcHour =  buf[0];
  byte hcMin = buf[1];
  byte hcSec = buf[2];

// UPDATE HAND CONTROLLER'S DATE AS NECESSARY
  if (hcVersion < 422) {                                      // If version is before 4.22 the GPS
    jd2date((float(jdx10()) / 10.0) + GPS_DATE_ADJ);
    hcYear = jdYear;
    if (hcVersion >= 203) {                                   // Update the hand controller's clock
      buf[0] = 'H';                                           // Command to update date & time
      buf[1] = hcHour;
      buf[2] = hcMin;
      buf[3] = hcSec;
      buf[4] = hcMonth = jdMonth;
      buf[5] = hcDay = jdDay;
      buf[6] = hcYear - 2000;
      buf[7] = 0;                                             // Hour offset from Greenwich
      buf[8] = 0;                                             // 0 = standard time (no daylight savings)
      if (UPDATE_HC) {
        for (byte i = 0; i <= 8; i++) Serial1.write(buf[i]);
        if (debug) Serial.println("Updating Hand Controller");
        delay(100);
        if (!Serial1.readBytes(buf, 1) || buf[0] != '#') {
          if (debug) Serial.println("Timed out");             // Timed out!
          scopeConnected = 0;                                 // Lost connection with the telescope
          return;
        }
      }
    }
  }

// SET TRACKING MODE TO EQ NORTH
  Serial1.write('T');
  Serial1.write(2);
  if (debug) Serial.println("Setting tracking mode to EQ North");
  delay(100);
  if (!Serial1.readBytes(buf, 1) || buf[0] !='#') {
    if (debug) Serial.println("Timed out");
    scopeConnected = 0;
    return;
  }

  if (debug) {
    Serial.print("Firmware version: ");
    Serial.println(float(hcVersion) / 100);
    Serial.print("\nCalendar date: ");
    Serial.print(hcDay);
    Serial.print("/");
    Serial.print(hcMonth);
    Serial.print("/");
    Serial.print(hcYear);
    Serial.print(" ");
    Serial.print(hcHour);
    Serial.print(":");
    Serial.print(hcMin);
    Serial.print(":");
    Serial.println(hcSec);
    Serial.print("Lat:  ");
    Serial.println(hcLat, 8);
    Serial.print("Long: ");
    Serial.println(hcLong, 8);
  }

// CALCULATE SIDEREAL TIME AT GREENWICH AT CONNECTION
  long  jd = (jdx10() - 24515450);
  float T = float(jd) / 365250.0;
  float Th0 = 100.46061837 + 36000.770053608 * T + 0.000387933 * T * T - (T * T * T) / 38710000;
  Th0 += 1.00273790935 * (hcHour * 15 + hcMin * 0.25 + hcSec / 240.0);
  hcTh0 = norm(Th0, 360);
  if(debug) {
    Serial.print("Sidereal Time at connection: ");
    Serial.println(hcTh0 / 15, 8);
  }
}

// ==================================================================================================
// CALCULATIONS
// ==================================================================================================
// Thanks to Jean Meuus's book Astronomical Algorithms (First Edition) ISBN 0-943396-35-2

byte eq2azm(){                                    // Calculates telescope's azimuth
                                                  // Astronomical Algorithms First Edition page 89
  float L = hcLong;                               // Longitude in degs (+ve West, -ve East)
  float a = hcRA;                                 // RA in degs
  float d = hcDEC;                                // Dec in degs
  float H = norm(sidTime0() - L - a, 360);        // Hour angle from meridian in degs
  float phi = hcLat;                              // Latitude in degs

  float A = atan2d(sind(H), (cosd(H) * sind(phi) - tand(d) * cosd(phi)));
                                                  // Azimuth in degrees, South is 0 degs
  if (debug) {
    Serial.print("Azimuth: ");
    Serial.println(norm(A + 180, 360), 8);
  }
  return byte(256.0 * norm(A + 180, 360) / 360 + 0.5);  // Returns Azimuth value between 0 and 255
}

// --------------------------------------------------------------------------------------------------
float sidTime0(){                         // Calculates current Sidereal Time at Greenwich in degs
                                          // Astronomical Algorithms page 83
  float Th0 = hcTh0 + 1.00273790935 * (float(millis() - jdStart) / 240000.0); // Time lapsed since startup
  return norm(Th0, 360);
}

// --------------------------------------------------------------------------------------------------
unsigned long jdx10(){                    // Calculates Julian Day number x 10 at 0h00m00s UT
                                          // Astronomical Algorithms page 59
  int a, b, y = hcYear;
  long i, j;
  byte m = hcMonth;

  if(m < 3){
    y--;
    m+= 12;
  }
  a = int(y / 100);
  b = 2 - a + int(a / 4);
  i = (float)(365.25 * (y + 4716));
  j = 30.6001 * (m + 1);
  return (i + j + hcDay + b) * 10 - 15245L;
}

// --------------------------------------------------------------------------------------------------
void jd2date(float Z){                            // Astronomical Algorithms First Edition page 63
  float a, A, B, C, D, E, F;
  Z = Z + 0.5;
  F = Z - long(Z);
  Z = long(Z);
  a = long((Z - 1867216.25) / 36524.25);
  A = Z + 1 + a - long(a / 4);
  B = A + 1524.0;
  C = long((B - 122.1) / 365.25);
  D = long(365.25 * C);
  E = long((B - D) / 30.6001);
  jdDay = B - D - long(30.6001 * E) + F;
  jdMonth = (E < 14) ? E - 1 : E - 13;
  jdYear = (jdMonth > 2) ? C - 4716 :  C - 4715;
}

// --------------------------------------------------------------------------------------------------
float sind(float n){                // Calculates sine of an angle in degrees
  return sin(n / RAD2DEG);
}

// --------------------------------------------------------------------------------------------------
float cosd(float n){                // Calculates cosine of an angle in degrees
  return cos(n / RAD2DEG);
}

// --------------------------------------------------------------------------------------------------
float tand(float n){                // Calculates tangent of an angle in degrees
  return tan(n / RAD2DEG);
}

// --------------------------------------------------------------------------------------------------
float atan2d(float m, float n){     // Calculates tangent of an angle in degrees
  return atan2(m, n) * RAD2DEG;
}

// --------------------------------------------------------------------------------------------------
float norm(float a, float b){       //Normalises a between 0 and b
  return a - int(a / b) * b + (a < 0) * b;
}

// ==================================================================================================
// SERIAL ECHO TEST
// ==================================================================================================
void serialTest() {
  char c;
  Serial.println("Serial Echo Test - press and hold Debug button to quit or send 'X'");
  unsigned long scopeTimer = millis();
  unsigned long pcTimer = millis();
  while (!Serial);
  while (debug) {
    debugButton();
    if (Serial.available()) {
      Serial.write(c = Serial.read());
      if (c == '%') return;
      Serial1.write(c);
      analogWrite(LED1, OFF);
      pcTimer = millis() + 100;
    }
    if (Serial1.available()){
      Serial.print("[");
      Serial.print(byte(Serial1.peek()), DEC);
      Serial.print("]");
      Serial.write(Serial1.read());
      analogWrite(LED2, OFF);
      scopeTimer = millis() + 100;
    }
    if (pcTimer < millis()) analogWrite(LED1, BRIGHT);
    if (scopeTimer < millis()) analogWrite(LED2, BRIGHT);
  }
}