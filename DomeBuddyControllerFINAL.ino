#define VERSION "\nDomeBuddy Controller MVC V1.210501 by keithrickard@hotmail.com"

// Work started 20 October 2020 -  for the Arduino Nano

// PARAMETERS
#define RAMP_SPD      20
#define ABORT_SPD     10
#define MAX_SPD       0
#define STUCK_TIME    2000    // Period in millisecs for non-detection of dome motion
#define ENC_DIR       1       // 0 = test rig, 1 = dome rig
#define MTRA_DIR      1
#define MTRB_DIR      1
#define ROLLBACK      5
#define TESTMODE      0       // 1 = test mode, 0 = live mode

#include <SoftwareSerial.h>
#include <EEPROM.h>           // 0-3 = Resolution, 4-5 = Azimuth pointer, 6+ scopeAZM and lastDir (1 byte each)

#define BTBAUD        38400
#define ENC_ChA       2
#define ENC_ChB       3
#define DIR1          4
#define PWM1          5
#define PWM2          6
#define DIR2          7
#define MTR_GND       8
#define LED_GND       9
#define LED           10
#define btTXD         11
#define btSTATUS      12
#define btRXD         A7
#define BUZZER        A0
#define BTN_L         A1
#define BTN_R         A2
#define BTN_P         A3
#define GND1          A4
#define GND2          A5

#define PRESSED       1
#define RELEASED      0

volatile long currEnc;
long prevEnc;
long enc360;
int enc180; 
int encOne;
byte homeAZM;
byte scopeAZM;                     // 0 = N, 64 = E, 128 = S, 192 = W.
byte domeAZM;
byte lastDir = 0;
byte parked = 1;
byte ledFlash = LOW;
byte btCxnMsg = 0;
byte calibrating = 0;

SoftwareSerial btSerial(btTXD, btRXD);  // Bluetooth Module is in Master mode & bound with DomeBuddy Dongle's address

//==============================================================================================================
void setup() {
  pinMode(ENC_ChA, INPUT_PULLUP);
  pinMode(ENC_ChB, INPUT_PULLUP);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(MTR_GND, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED_GND, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_P, INPUT_PULLUP);
  pinMode(BTN_R, INPUT_PULLUP);
  pinMode(BTN_L, INPUT_PULLUP);
  pinMode(GND1, OUTPUT);
  pinMode(GND2, OUTPUT);
  pinMode(btSTATUS, INPUT);
  digitalWrite(GND1, LOW);
  digitalWrite(GND2, LOW);
  digitalWrite(MTR_GND, LOW);
  digitalWrite(DIR1, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(BUZZER, HIGH);
  digitalWrite(LED_GND, LOW);
  digitalWrite(LED, HIGH);

  btSerial.begin(BTBAUD);                           // Open connection to Bluetooth module
  btSerial.setTimeout(500);
  Serial.begin(9600);
  Serial.println(VERSION);
  
  attachInterrupt(digitalPinToInterrupt(ENC_ChA), encISR, RISING);  // encISR monitors encoder movement
  if (checkButton(BTN_P, PRESSED, 500)) calibrateDome();            // If PARK button is being pressed, calibrate dome resolution
  
  EEPROM.get(0, enc360);                            // Encoder counts for 360 degrees
  int addr;
  EEPROM.get(4, addr);
  EEPROM.get(addr, currEnc);                         // Get last known dome encoder position
  EEPROM.get(addr + 4, lastDir);
  if (enc360 < 0) enc360 = 10800;                   // 600 * 1350 / 75
//  enc360 = 18432;                                 // 1024 * 1350 / 75
  enc180 = enc360 >> 1;                             // Value for 180 degrees
  encOne = enc360 >> 8;                             // Number of encoder counts azimuth one reference point
  scopeAZM = domeAZM = homeAZM = enc2azm(currEnc);
  Serial.print("Resolution: ");
  Serial.println(enc360);
  Serial.print("Last known move direction: ");
  Serial.print(lastDir);
  Serial.println(lastDir ? "=L (CCW)" : "=R (CW)");
  Serial.print("DOME PARKED AT: ");
  Serial.print(domeAZM);
  Serial.print("=");
  Serial.println(currEnc);
  delay(500);
  buzz(500);
}

//==============================================================================================================
void loop() {
  if (bluetooth()) digitalWrite(LED, parked);       // Flash LED if waiting for Bluetooth connection. Light LED if parked
  getScopeAZM();                                    // Get any update of the telescope's azimuth
  if(Serial.available()) getTestAZM();              // See if user has sent a test azimuth to slew to then set scopeAZM to that
  domeAZM = enc2azm(normEnc());                     // Normalise currEnc then map domeAZM from value 0 - 255
  if (!parked) moveDome(scopeAZM, 0);               // If not parked, dome if necessary.
  
  manualMove(BTN_R);                                // See if RIGHT (CW) button is pressed to turn dome clockwise
  manualMove(BTN_L);                                // See if LEFT (CCW) button is pressed to turn dome counter clockwise
  parkButton();                                     // See if PARK button is being pressed and act on it is so
}

//==============================================================================================================
bool moveDome(byte targAZM, byte rb) {          // rb = 1 means rollback is required
  if (domeAZM == targAZM) return 0;             // Return if target is same as current dome azimuth
  
  // INITIALISE ------------------------------------------------------------------------------------------------
  long targEnc = azm2enc(targAZM);              // Convert target (0 - 255) into encoder reference point
  byte dir = 0;
  long distEnc = targEnc - currEnc;             // Get distance to travel
  prevEnc = currEnc;                            // Remember initial currEnc value

  if (abs(distEnc) > enc180) {                  // Is the distance more than 180 degs?
    if (currEnc < enc180) currEnc += enc360;    // If currEnc is less than 180 degs then 360 degs needs to be added
    else targEnc += enc360;                     // Otherwise 360 degs needs to be added to the targEnc
    distEnc = targEnc - currEnc;                // Recalculate distance (now <= 180 degs)
  }

  dir = (distEnc <= 0);                          // Get direction (FALSE = right (CW), TRUE = left (CCW) )
  if (!rb && dir != lastDir && abs(distEnc) <= ROLLBACK * encOne) {
    // Test: if dir for this move is different from the last and the dist to move is more than 5 steps
    Serial.println("\nROLLING BACK...");
    currEnc = prevEnc;
    moveDome(domeAZM + (dir ? ROLLBACK : -ROLLBACK), 1); // Roll-back 5 steps before moving (1 = do roll-back move)
    Serial.println("\nNOW SLEWING TO TARGET POSITION");
    return;
  }
  lastDir = dir;
  long midPoint = targEnc - distEnc / 2;        // Calc mid point reference point between start and end positions
  prevEnc = currEnc;                            // Remember starting encoder position

  Serial.print("AUTO MOVING from ");
  Serial.print(domeAZM);
  Serial.print("=");
  Serial.print(currEnc);
  Serial.print(" to ");
  Serial.print(targAZM);
  Serial.print("=");
  Serial.println(targEnc);
//  Serial.print("enc360   ");
//  Serial.println(enc360);
  Serial.print("encOne   ");
  Serial.println(encOne);
  Serial.print("distEnc  ");
  Serial.println(distEnc);
  Serial.print("midPoint ");
  Serial.println(midPoint);
  Serial.print("dir      ");
  Serial.print(dir);
  Serial.println(dir ? "=L (CCW)" : "=R (CW)");

  // START-UP --------------------------------------------------------------------------------------------------
  digitalWrite(DIR1, MTRA_DIR ? dir : 1 - dir); // Select direction on the motor control board
  digitalWrite(DIR2, MTRB_DIR ? dir : 1 - dir);
  byte pwm = 0;
  int8_t brakEnc[256];
  long stepDist;
  prevEnc = currEnc;
  Serial.println("RAMPING-UP");
  do {                                        // Ramp-up the motors
    delay(RAMP_SPD);                          // Delay creates a rate of ramping-up
    if (checkBtns()) break;                   // A button has been pressed so stop immediately
    if (!dir) if (currEnc > midPoint) break;  // Mid point check for RIGHT (CW)
    if (dir) if (currEnc < midPoint) break;   // Mid point check for LEFT (CCW)
    analogWrite(LED, pwm);                    // Brighten the LED
    analogWrite(PWM1, pwm);                   // Increase the motors' speed
    analogWrite(PWM2, pwm);
    stepDist = currEnc - prevEnc;             // See how far the dome has moved
    prevEnc = currEnc;                        // Remember current encoder value for next time
    if (stepDist < -128) stepDist = -128;     // Cap the value of stepDist if necessary
    if (stepDist > 127) stepDist = 127;
    targEnc -= brakEnc[pwm] = int8_t(stepDist); // Update tragetEnc - point for when to slow down
    //targEnc -= brakEnc[pwm] = byte(currEnc - prevEnc);
    pwm++;   
  } while (pwm != MAX_SPD);                   // Exit ramping-up if halfway there or maximum speed reached

  // SLEW ------------------------------------------------------------------------------------------------------
  byte finalPwm = pwm - 1;
  if (abs(distEnc) > encOne * 2) targEnc += (dir ? encOne : -encOne) / 2; // (CCW : CW) - adj for moves > 2 ref points
  //if(abs(prevEnc - currEnc) < encOne) targEnc = currEnc + (dir ? -encOne : encOne) / 3;
  prevEnc = targEnc;
  Serial.print("currEnc  ");
  Serial.println(currEnc);
  Serial.print("brakEnc  ");
  Serial.println(prevEnc);
  Serial.print("pwm      ");
  Serial.println(pwm);
  Serial.println("SLEWING");

  // SLOW DOWN -------------------------------------------------------------------------------------------------
  long lastEnc = currEnc;
  unsigned long moveTimer = millis() + STUCK_TIME;
  byte firstPass = 1;
  do {                                            // Slow down the motors
    pwm--;
    if (parked) delay(ABORT_SPD);                 // If slew aborted (parked = 1) then just slow down
    else {                                        // Else do a controlled slow down
      while (!checkBtns()) {                      // Wait for encoder to reach the value for this PWM step
        if (!dir) if (currEnc > targEnc) break;                     // RIGHT (CW)
        if (dir)  if (currEnc < targEnc) break;                     // LEFT (CCW)
        if (currEnc != lastEnc) moveTimer = millis() + STUCK_TIME;  // If encoder is moving then moveTimer is reset
        if (parked = (millis() > moveTimer)) break;                 // If no motion detected within STUCK_TIME then abort
        lastEnc = currEnc;                                          // Remember current encoder value for next time round
      }
      if (parked) Serial.println("MOTION NOT DETECTED - ABORTED");
      if (firstPass) {
        Serial.println("SLOWING DOWN");           // Send this message only once
        firstPass = 0;
      }
    }
    targEnc += brakEnc[pwm];                      // Upate targEnc
    analogWrite(LED, pwm);                        // Dim the LED
    analogWrite(PWM1, pwm);                       // Slow down the motors
    analogWrite(PWM2, pwm);
  } while (pwm);                                  // When pwm is 0 then the motors are turned off
  
  // FINISH-UP -------------------------------------------------------------------------------------------------
  Serial.print("finalEnc  ");
  Serial.println(currEnc);
  Serial.print("targEnc   ");
  Serial.println(targEnc);
  Serial.print("STOPPED at ");
  domeAZM = enc2azm(normEnc());                   // Normalise currEnc and update domeAZM
  Serial.print(domeAZM);
  Serial.print("=");
  Serial.println(currEnc);
  updateEEPROM(currEnc, dir);                     // Update EEPROM with current Dome Azimuth  
  if(parked) {
    Serial.println("DOME PARKED");
    buzz(1000);
  }
  
  if (TESTMODE) {
    Serial.println("brakEnc array");
    for(byte i = 0; i < finalPwm; i++){
      if (!(i % 8)) {
        Serial.println();
        Serial.print(i);
        Serial.print(':');
      }
      Serial.print(' ');
      Serial.print(prevEnc);
      prevEnc += brakEnc[finalPwm - i];
    }
    Serial.println();
  }
  return domeAZM != targAZM;                      // Return 0 if domeAZM is same as target azimuth, 1 if not
}

//==============================================================================================================
void manualMove(byte button) {
  if (checkButton(button, RELEASED, 0)) return;     // Return if button not pressed

  byte dir = lastDir = (button == BTN_L);           // dir is 0 if button is R (CW) or 1 if L (CCW)
  buzz(10);
  Serial.print("MANUAL MOVE ");
  if (calibrating) Serial.print("(calibrating) ");
  if (parked) Serial.print("(parked) ");
  if (!parked && !calibrating) Serial.print("(dome active) ");
  Serial.print(dir ? "L (CCW) from " : "R (CW) from ");
  if (!calibrating){
      Serial.print(domeAZM);
      Serial.print("=");
  }
  Serial.print(currEnc);
  Serial.print(" to ");

  // START-UP ------------------------------------------------------------------------------------------------
  byte pwm = 0;
  digitalWrite(DIR1, MTRA_DIR ? dir : dir ^1);      // Select direction on the motor control board
  digitalWrite(DIR2, MTRB_DIR ? dir : dir ^1);      // dir = 0 is R (CW), dir = 1 is L (CCW)

  do {                                              // Ramp-up the motors
    analogWrite(LED, pwm);                          // Brighten the LED
    analogWrite(PWM1, pwm);
    analogWrite(PWM2, pwm);
    pwm++;
    if (checkButton(button, RELEASED, 0)) break;    // Break out if a button is no longer pressed
    delay(RAMP_SPD);
  } while (pwm != MAX_SPD);                         // Exit if maximum speed reached (pwm = 0 is 256)
  
  // CONTINUE --------------------------------------------------------------------------------------------------
  if (!pwm) analogWrite(pwm, 255);
  while (checkButton(button, PRESSED, 0));          // Keep the dome turning while button is being pressed

  // SLOW DOWN -------------------------------------------------------------------------------------------------
  do{                                               // Slow down the motors
    pwm--;
    analogWrite(LED, pwm);                          // Dim the LED
    analogWrite(PWM1, pwm);
    analogWrite(PWM2, pwm);
    delay(ABORT_SPD);
  } while (pwm);                                    // When pwm is 0 then the motors are turned off
  
  // FINISH-UP --------------------------------------------------------------------------------------------------
  if (!calibrating) {                               // Do stuff if not calibrating
    if (parked) domeAZM = enc2azm(normEnc());       // If parked, normalise currEnc
    else currEnc = azm2enc(domeAZM);                // Make currEnc be encoder value for domeAZM
    Serial.print(domeAZM);
    Serial.print("=");
    Serial.println(currEnc);
    updateEEPROM(currEnc, lastDir);                 // Update EEPROM with current Dome Azimuth
  }
  else Serial.println(currEnc);
  buzz(10);
}

//==============================================================================================================
bool checkBtns() {                                // Look at all buttons.  Returns 1 if a button is being pressed
  if (!parked || Serial.available()) {
    parked = (!digitalRead(BTN_P) || !digitalRead(BTN_R) || !digitalRead(BTN_L));
    if(Serial.read() >= 0) parked = 1;
    if (parked) Serial.println("MOVE ABORTED BY USER");
  }
  return parked;
}

//==============================================================================================================
bool checkButton(byte button, bool state, unsigned long delayTime) { 
                                                      // state = 0 test releasing of button, = 1 test pressing of button
  delayTime += millis();                              // Wait upto delayTime ms for button being released/pressed before quitting
  do {
    if (!digitalRead(button) == state) {              // Has button been released/pressed? (button pin: 1 = released, 0 = pressed)
      delay(50);                                      // Wait 50ms for debounce
      if (!digitalRead(button) == state) return 1;    // Test button again - return if it is still released/pressed
    }
  } while (millis() < delayTime);
  return 0;                                           // Button not released/pressed within delay time, return 0.
  
// state: 1 = PRESSED, 0 = RELEASED
}

//==============================================================================================================
void buzz(int duration) {
  digitalWrite(LED, HIGH);                            // Sound buzzer and light LED for 'duration' milliseconds
  digitalWrite(BUZZER, HIGH);
  delay(duration);
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED, LOW);
}

//==============================================================================================================
int normEnc() {
  long value = currEnc;                   // Noramlises currEnc between 0 and enc360 - 1
  return currEnc = value % enc360 + ((value < 0) ? enc360 : 0);
}

//==============================================================================================================
byte enc2azm (long enc){
  return (enc * 256) / enc360;
}

//==============================================================================================================
long azm2enc (long azm){
  return (azm * enc360) / 256;
}

//==============================================================================================================
void calibrateDome(){
  Serial.println("CALIBRATING DOME");
  for (byte i = 1; i <= 10; i++){                           // Give 10 quick beeps
    delay(200); buzz(200);
  }
  calibrating = 1;
  enc360 = 0x7FFFFFF;                                       // Set dome resolution to a really high number!
  Serial.println("Updating dome encoder resolution...");
  
  do {
    currEnc = 0;                                            // Set the encoder value to 0 before waiting for buttons to be pressed
    while(checkButton(BTN_P, RELEASED, 0)){                 // Keep looping while the PARK button is not being pressed
      digitalWrite(LED, HIGH);                              // Keep the LED lit
      manualMove(BTN_R);                                    // See RIGHT (CW) button is being pressed and turn dome accordingly
      manualMove(BTN_L);                                    // See LEFT (CCW) button is being pressed and turn dome accordingly
    }
    buzz(10);                                               // The PARK button has been pressed so beep
  } while(checkButton(BTN_P, RELEASED, 5000));              // Loop back if the PARK button was pressed for less than 5 secs

  EEPROM.put(0, abs(currEnc));                              // Success!  Save the new value of the dome resolution
  Serial.println("DOME CALIBRATED!");
  Serial.print("Resolution: ");
  Serial.println(abs(currEnc));
  buzz(50); delay(200); buzz(50); delay(75); buzz(50); delay(75); 
  buzz(50); delay(200); buzz(50); delay(450); buzz(50); delay(200); buzz(50);

  delay(1000);
  restart();
}

//==============================================================================================================
void parkButton() {
  if (digitalRead(BTN_P)) return;                       // Return immediately if PARK button not pressed
  if (checkButton(BTN_P, RELEASED, 3000)) parkUnpark(); // If button is released within 3 seconds then park/unpark dome
}
//==============================================================================================================
void parkUnpark(){  
  parked ^= 1;                                          // Flip parked status
  Serial.println(parked ? "DOME PARKED" : "DOME ACTIVE"); 
  buzz(1000);
}

//==============================================================================================================
void goHome(){
  delay(500);
  buzz(50); delay(200); buzz(50); delay(750);
  buzz(50); delay(200); buzz(50); delay(200); buzz(50); delay(450);
  buzz(50); delay(200); buzz(50); delay(200); buzz(50); delay(200); buzz(50); delay(200);
  delay(250); buzz(50); delay(200); buzz(50);
}

//==============================================================================================================
void getScopeAZM() {                                              // Get and interpret scope's azimuth from DomeBuddy Dongle via Bluetooth
  if (!digitalRead(btSTATUS)) return;                             // Return immediately if not connected to DomeBuddy Dongle via Bluetooth
  if (!btSerial.available()) return;                              // Return if no received data present
  digitalWrite(LED, HIGH);
  //Arduino should receive via Bluetooth {':', S' or 'T', Xh, Xl, -Xh, -Xl, '#'} i.e. 7 characters
  char buf[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  byte value, chksum;
  do {
    while (btSerial.peek() >= 0){                                 // Find the sentence start marker ':'
      if (btSerial.peek() == ':') break;                          // Found the marker so break out
      if (btSerial.peek() < 0) return;                            // Return if timed out
      btSerial.read();                                            // Dump the invalid character in the receive buffer
    }                                                             // Loop back and try again
    btSerial.readBytesUntil("#", buf, 7);                         // Get azmiuth info (7 bytes)
  } while (btSerial.peek() >= 0);
  Serial.print("\nGot from DomeBuddy Dongle: ");                  // Report what has been received from Dome Dongle
  Serial.println(buf);  

  if (buf[1] == 'S') {                                            // 'T' = tracking, 'L' lost telescope, return if not.
    Serial.println("TELESCOPE IS SLEWING");
    return;
  }
  if (buf[1] == 'L') Serial.println("TELESCOPE NOT RESPONDING");
  else { 
    if (buf[1] != 'T') {
      Serial.println("DATA NOT VALID!");
      return;
    }
  }
  value = buf[2] - ((buf[2] >= 'A') ? 55 : 48);                   // Get telescope azimuth into value (from hex)
  value = value * 16 + buf[3] - ((buf[3] >= 'A') ? 55 : 48);
  chksum = buf[4] - ((buf[4] >= 'A') ? 55 : 48);                  // Calculate the checksum (from hex)
  chksum = chksum * 16 + buf[5] - ((buf[5] >= 'A') ? 55 : 48);
  if (value == byte(~chksum)) {
    scopeAZM = value;                                             // Azimuth has been received correctly
    Serial.print("TELESCOPE AZIMUTH = ");
    Serial.print(scopeAZM);
    Serial.print("=");
    Serial.println(enc2azm(scopeAZM));
  }
  else Serial.println("DATA NOT VALID!");
}
//==============================================================================================================
void getTestAZM() {
  int azm;
  if (isdigit(Serial.peek())){
    azm = Serial.parseInt();
    delay(200);
    while (Serial.read() >= 0);
    Serial.print("User supplied azimuth = ");
    Serial.println(azm);
    if (azm < 0 || azm > 255) Serial.println("Invalid azimuth - expected value between 0-255");
    else scopeAZM = azm;
  }
  else {
    int c = Serial.read();
    if (c < 0) return;
    delay(200);
    while (Serial.read() >= 0);
    switch (c) {
      case 'B': buzz(1000);       break;
      case 'C': calibrateDome();  break;
      case 'P': parkUnpark();     break;
      case 'R': restart();        break;
    }
  }
}
//==============================================================================================================
void restart() {
  // Turn off LED, motors and buzzer then restart the sketch
  analogWrite(LED, 0);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(BUZZER, LOW);  
  asm volatile ("jmp 0");
}

//==============================================================================================================
byte bluetooth() {
  if (digitalRead(btSTATUS)) {                      // Check to see if a Bluetooth connection has been made
    if (btCxnMsg) Serial.println("Bluetooth connected");
    btCxnMsg = 0;
  }
  else {
    digitalWrite(LED, ledFlash = 1 - ledFlash);     // No Bluetooth connection so flash the LED rapidly
    if (!btCxnMsg) Serial.println("Waiting for Bluetooth connection...");
    btCxnMsg = 1;
  }
  return !btCxnMsg;
}
//==============================================================================================================
void updateEEPROM(long enc, byte dir) {
  int addr;
  byte eeDir;
  long eeEnc;
  EEPROM.get(4, addr);            // Address pointer for storing dome encoder value
  EEPROM.get(addr, eeEnc);        // Get last stored encoder value
  EEPROM.get(addr + 4, eeDir);    // Get last stored direction value
  addr--;

  // Store at EEPROM address addr.  If it cannot store at this address, the next address is tried.
  do {
    addr++;
    if (addr > 1019) {
      Serial.print("EEPROM write FAILED at address ");
      Serial.println(addr);
      return;
    }
    EEPROM.put(addr, enc);
    EEPROM.put(addr + 4, dir);
    EEPROM.get(addr, eeEnc);
    EEPROM.get(addr + 4, eeDir);
  } while (eeEnc != enc || eeDir != dir);

  EEPROM.put(4, addr);                          // Update addr pointer as necessary
  Serial.print("EEPROM address: ");
  Serial.print(addr);
  Serial.print(" = ");
  Serial.print(eeEnc);
  Serial.print(", ");
  Serial.print(dir);
  Serial.println(dir ? " (CCW)" : "(CW)");
}
//==============================================================================================================
void encISR() {
#ifdef ENC_DIR
  (PIND & B1000) ? currEnc-- : currEnc++; // PIND & B1000 is pin D3 (ENC_ChB)
#else
  (PIND & B1000) ? currEnc++ : currEnc--; // PIND & B1000 is pin D3 (ENC_ChB)
#endif
// When ENC_ChA rises from 0 to 1, ENC_ChB = 1 means RIGHT (CW), ENC_ChB = 0 means LEFT (CCW)
}

//==============================================================================================================
/*
  Programming DomeBuddy's HC-05 Bluetooth Module:
  
  AT+ORGL                   Restore default values
  AT+ROLE=1                 Master mode (0 = slave mode)
  AT+CMODE=0                Connect to Bluetooth device with BIND address
  AT+BIND=98D3,71,F5F856    DomeBuddy Dongle's Bluetooth Address
  AT+PSWD=1234              DomeBuddy Dongle's pin code
  AT+UART=38400,0,0         DomeBuddy's UART setting (38400 baud)
  AT+NAME=DomeBuddy         Give the Bluetooth Module its name
  AT+RESET                  Reset the module to restart with new settings

  Nano Connections
  ================
  
                  +-------+
              D13 o       o D12   BT_STAT
              3V3 o       o D11~  BT_TXD
              REF o       o D10~  LED     Bk
Bk R  BUZZER  A0  o       o D9~   LED_GND W
O  Y  BTN_R   A1  o       o D8    MC_GND  Bn
Be Gn BTN_L   A2  o       o D7    MC_DIR2 O
Pe W  BTN_P   A3  o       o D6~   MC_PWM2 R
Jmpr  GND1    A4  o       o D5~   MC_PWM1 Y
Jmpr  GND2    A5  o       o D4    MC_DIR1 Gn
              A6  o       o D3~   ENC_CHB Gn
     (BT_RXD) A7  o       o D2    ENC_CHA W
R     ENC_VCC 5V  o  ICSP o GND   ENC_GND Bk
              RST o  ooo  o RST
Bk    PSU_GND GND o 1ooo  o RX0
Pe    PSU_12V VIN o       o TX1
                  +-------+

      ICSP
      ooo      5V  D11  GND
     1ooo      D12 D13  RST
            
V      BT_5V     5V
Gy     BT_TXD    D11
Bk     BT_GND    GND
W      BT_STAT   D12

  ENCODER
  =======

  Encoder               Arduino
  Pin 1 W   ChA   ---   Gn    Pin 2 (interrupt)
  Pin 2 R   Vcc   ---   R    +5V
  Pin 3 Gn  ChB   ---   Gn    Pin 3 (interrupt)
  Pin 4 Bk  GND   ---   Bk   GND

  Mini 4 Pin DIN plug for encoder (looking into plug at pins)
  -----------------------------------------------------------
   2 3
  1   4
    =


This sketch controls two car wiper motors that rotate a dome in either direction so that the dome's
slit is always positioned in front of the housed telescope.  DomeBuddy receives via its HC-05
Bluetooth module the telescope's reported azimuth, as given by the DomeDongle connected to the
telescope's mount.  The motors are connected to an motor control module which can ramp-up and ramp-down
each motor's rotation.

The motors use rubber tyred wheels to rotate the inner dome rim.  There is also a rotary encoder with a
similar wheel in contact with the rim so that the distance and direction of the movement of the dome
can be measured.  There are three button switches, one to park the dome and two to move the dome.
A buzzer gives feedback when synchronising, button pressing, and if an error has occurred.

Consideration is given for:

1)  Speed of rotation, taking account the distance of travel and pulses
2)  The shortest distance of travel - if at 350 degs and it needs to go to 10 degs that it will only
    travel 20 degs CW, not 340 degs CCW!
    
This project is primarily to control a 9.25" Celestron NexStar GPS SCT telescope (235mm) inside a 
1350mm diameter dome with a 400mm wide slit.

The motors are controlled by a Cytron 10Amp 5V-30V DC Motor Driver MDD10A (2 Channels):

  https://www.ebay.co.uk/itm/Cytron-10Amp-5V-30V-DC-Motor-Driver-2-Channels/154099988474
  
The rotary encoder has a resolution of 600ppr (2400ppr quadature resolution will not be used)

  https://www.amazon.co.uk/Wisamic-Incremental-Encoder-Dc5-24v-Voltage/dp/B015GYY7XU
  
  Wires: ChA = Green, ChB = White, +5V = Red, GND = Black

*/