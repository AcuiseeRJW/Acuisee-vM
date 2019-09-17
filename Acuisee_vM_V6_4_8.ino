// CONDITIONAL COMPILATION FLAGS
// **************************************************************
//uncomment define below for tester mode
//#define TESTERMODE 1


// COMMENTS
// **************************************************************
// date        who  ver	what
// ----------------------------------------------------------------------------------------------------
// 3/24/15	    rjw 5.0	rework for version 3.0 of command set (see below)
// 3/25/15	    rjw 5.0	added up and down servo variables
// 4/24/2015    rjw 5.0 removed mstimer2 - interrupt issues/conflicts with sound functions
// 4/24/2015    rjw 5.0 reworked run light blink to more standard time compare algorithm
// 4/24/2015    rjw 5.0 added tone generation
// 4.27.2015    rjw 5.0 removed servo cycle from startup loop - force tablet to issue reset
// 4.29.2015    rjw 5.0 removed all debug logic, shortened play from 6 to 4 function calls
// 9.16.2015    rjw 5.1 added BT communications
//10.6.2015     rjw 5.1 added Pololu stepper code
//10.8.2015     rjw 5.1 optimized bt and serial char processing
//10.8.2015     rjw 5.1 separated out the command processing from the char read
//10.9.2015     rjw 5.1 modularized speaker calls and led calls
//10.20.2015    rjw 5.1 reworked stepper code
//10.27.2015    rjw 5.2 added timer to dipper up routine to stop if LS fails
//10.29.2015    rjw 5.3 optimized JP layout for cabling
//11.04.2015    rjw 5.4 added fault logic for dipper travel
//11.04.2015    rjw 5.4 separated dipper up/dwn and dipper up ls
//11.04.2015    rjw 5.4 added LED rapid blink when fault detected
//12.15.2015    rjw 6.0 +/- offset for dipper down travel distance
//12.15.2015    rjw 6.0 offset saved in NVRAM
//12.15.2015    rjw 6.0 implemented ? command to provide status. version, and paramters
//12.15.2015    rjw 6.0 COMMAND FOR AUTO CYCLE MODE = "^"
//03.07.2016    RJW 6.1 shifted LED color to green and dimmed down, added 10KR to Peizo
//04.06.2016    rjw 6.2 The commands I'll need for the software recovery mode are a response that tells
//                      me when the dipper is going up, (we could call it 'q') and a response that tells
//                      me when the dipper is going down, as an 'r'.
//04.06.2016    rjw 6,2 disabled the following unused commands (b,d,m,n,o,p) not used oper Dillon,BT only
//04.11.2016    rjw 6.21 increase dipper timeout from 3 to 5 seconds to fix odd problem
//                       of not sending an "i'. modified if in UPDIPPER to check if no dipperfault
//                       rather than dipper up ls to eliminate bounce possibility\
//04.14.2016    rjw 6.3 incorporated a retry command (s) for bluetooth character mis-reads
//04.21.2016    rjw 6.4 added PING logic to see if Android is available - if no go into MANUAL and
//                      allow dipper PB to be active
//04.21.2016    rjw 6.41 30 second timeout on inactivity.  Manual = true on startup
//04.21.2016    rjw 6.42 added conditional defines for tester and non-tester mode
//05.18.2016    rjw 6.43 removed and optimized a whole bunch of delay statements
//06.02.2016    rjw 6.44 removed bluetooth retry limit of 3 to retry indefinately
//09.27.2016    rjw 6.45 added 3 second hold down timer to dipper pb to toggle AUTO mode  ** NOT YET COMPLETE
//08.07.2017    rjw 6.46 remove redled and replaced with chamberlt - always on D11
//04.08.2019    rjw 6.47 intercept and discard CR(13) or LF(10) in bt read
//09.10.2019    rjw 6.48 added code to handle air puff system
//                       remove all commented out lines to send bluetooth responses - 
//                       weren't used by tablet app so disabled until we built a debug app, then reenabled code
// ----------------------------------------------------------------------------------------------------

// FROM MASTER TO ARDUINO
// A-Status Request
// B-Reset Request
// C-ON Feed Light
// D-OFF Feed Light
// E-UP Dipper
// F-DOWN Dipper
// G-ON SOUND
// H-OFF SOUND
// I-ON PUFF SYSTEM
// J-OFF PUFF SYSTEM
// X-KEEP ALIVE RESPONSE TO x SENT FROM ARDUINO
// Z-NOOP (get a z returned but no actions taken 500ms delay)
// ^-AUTO CYCLE toggle auto cycle on/off
// +-UP offset increment and save to EEPROM
// - DOWN offset increment and save to EEPROM
// @-CLEAR offsets
// ?-Print offet information to usb serial port
// ----------------------------------------------------------------------------------------------------
// FROM ARDUINO TO MASTER
// a-Left LS ON
// b-Left LS OFF            * not used, by android, so not sent over bluetooth
// c-Right LS ON
// d-Right LS OFF           * not used, by android, so not sent over bluetooth
// e-Feed Detect LS ON
// f-Feed Detect LS OFF
// g-Feeder Light ON
// h-Feeder Light OFF
// i-Dipper UP
// j-Dipper DOWN
// k-Sound ON
// l-Sound OFF
// m-Dipper PB pressed      * not used, by android, so not sent over bluetooth
// n-Dipper PB not pressed  * not used, by android, so not sent over bluetooth
// o-Dipper LS ON           * not used, by android, so not sent over bluetooth
// p-Dipper LS OFF          * not used, by android, so not sent over bluetooth
// q-Dipper up start
// r-Dipper down start
// s-resend last command
// t-puff system on         AIR PUFF ON
// u-puff system off        AIR PUFF OFF
// x-KEEP ALIVE             sent to Android, if no response, allow manual dipper PB operation
// y-Dipper fault
// z-noop return
// ----------------------------------------------------------------------------------------------------
//PIN	I/O/A	DESCRIPTION
//-----------------------------------------------------
//D2	INPUT	LEFT LIMIT SWITCH
//D4	INPUT	RIGHT LIMIT SWITCH
//D7	INPUT	FEED DETECT
//D8  OUTPUT SPEAKER
//D9  OUTPUT PWM - SERVO SIGNAL
//D11	OUTPUT	RUN LIGHT
//D12 OUTPUT AIR PUFF controlPin        AIR PUFF CONTROL PIN D12
//D13	OUTPUT 	FEED LIGHT

/**************************************************************************
  POLOLU specifics - hard wired to eighth step for this app
  for a 200 step/rev stepper motor
  ---------------------------------------------------
  MS1	MS2	MS3	Microstep Resolution  Steps per Rev
  -------------------------------------------------------------------------------------------
  Low	Low	Low	Full step            1*200 = 200
  High	Low	Low	Half step            2*200 = 400
  Low	High	Low	Quarter step         4*200 = 800
  High	High	Low	Eighth step          8*200 = 1600
  High	High	High	Sixteenth step      16*200 = 3200

*************************************************************************** */

#include <SoftwareSerial.h>
#include <EEPROM.h>

#define Version        6.48
#define bluetoothRx    2  // RX-I
#define bluetoothTx    3  // TX-O 
#define STEP_PIN       4
#define DIR_PIN        5
#define feedDetect     6
#define feedLight_Grn  9
#define feedLight_Ylw  10
#define chamberlt      11
#define puffPin        12
#define runLight       13
#define rightLS        14
#define dipperupLS     15
#define leftLS         16
#define dipperPB       17
#define speaker        18

#define BLUE           15    // 0-255
#define RED            200   // 0-255
#define GREEN           5    // 0-255

// DEBUG
boolean DEBUG         = false ;
boolean dipperLSfault = false;
boolean autoCycle     = false;

// Bluetooth
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// Stepper
int step_size     = 8 ; // 1/16 (16), 1/8 (8), 1/4 (4), 1/2 (2), 1/1 (1)
int steps_per_rev = 42 ; // 200 step stepper
int steps_total   = step_size * steps_per_rev;  // total steps at resolution of stepper driver settings
int delay_time    = 100 ; // 70 time to delay between step waveform pulses - greater the microstep, less time needed

// state variables
int leftLSval        = 0;         // Left ls state
int rightLSval       = 0;         // Right ls state
int feedDetectval    = 0;         // feed detect state

// miscellaneous variables
int feedLight_Grnval = 0;         // feed light state
int feedLight_Ylwval = 0;         // feed light state
int DipperUP         = 0;         // dipper up FLAG
int DipperDN         = 0 ;        // dipper down FLAG
int DipperPBval      = 0 ;        // dipper pushbutton
int DipperupLSval    = 0 ;        // dipper LS state
int Speakerval       = 0 ;        // speaker state
byte upOffset ;                   // EEPROM offset for dipper
byte downOffset;                  // EEPROM offset for dipper
long previousMillis  = 0;         // will store last time run LED was updated
long interval        = 1000;      // interval at which to blink (milliseconds)
int divisor          = 1;         // used to control blink rate of LED
long lastDebounceTime  = 0;       // the last time the input check was made
long debounceDelay     = 25;      // the debounce time 50ms=.0050sec
long accumulatedTravelTime  = 0;  // variable to time stepper motion
unsigned long currentMillis = millis(); // variable to store current time 

#if defined(TESTERMODE)
long maxTravelTime    = 15000 ;   // 15 sec maximum stepper up travel time
#else
long maxTravelTime    = 5000  ;    // 5 sec maximum stepper up travel time
#endif

// Manual mode
unsigned long ManualTimeoutMillis = millis();
boolean ManualMode                = true ;   // manual enabled at startup
int     OneShot                   = 0;       // enable the one shot

// string variables
char inChar = -1; // Where to store the character read

// misc
int     CounterA            = 0;
int     autoCounter         = 0;
int     badCharCounter      = 0 ; // bluetooth bad char counter
boolean badCharFlag         = false ;
unsigned long badCharTimer  = millis();
long badCharTimerLimit      = 500  ;

/* **************************************************************************** */
//  SETUP()
void setup()
{
  // stepper
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // IO
  pinMode(dipperPB,   INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(dipperupLS, INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(leftLS,     INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(rightLS,    INPUT_PULLUP); // since PULLUP, LOW is ON
  pinMode(feedDetect, INPUT_PULLUP); //open collector transistor output - need a pull up resistor, LOW is ON
  pinMode(speaker,       OUTPUT);
  pinMode(chamberlt,     OUTPUT);
  pinMode(feedLight_Grn, OUTPUT);
  pinMode(feedLight_Ylw, OUTPUT);
  pinMode(runLight,      OUTPUT);
  pinMode(puffPin ,      OUTPUT);

  // Set Initial Values
  //analogWrite(feedLight_Red, feedLight_Redval);
  digitalWrite(chamberlt, HIGH);
  analogWrite(feedLight_Grn, 0);
  analogWrite(feedLight_Ylw, 0);
  digitalWrite(runLight,  LOW);
  digitalWrite(puffPin, LOW);

  //  bluetooth
  // 115200 (default) can be too fast at times for SoftSerial to relay the data reliably
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");     // Print three times individually to enter command mode
  bluetooth.print("$");
  bluetooth.print("$");
  delay(100);               // Short delay, wait for the BT module to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  delay(100);
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
  Serial.begin(9600);     // Start serial port at 9600
  Serial.print(F("\n\n  AcuiSee-vM Version: "));  Serial.println(Version);
  Serial.print(F("\n\n  AIR PUFF CONTROL on D12"));
  Serial.println(F("-------------------------------"));

#if defined(TESTERMODE)
  Serial.println(F("TESTERMODE DEFINED"));
#else
  Serial.println(F("TESTERMODE NOT DEFINED"));
#endif
  Serial.print(F("maxTravelTime = ")); Serial.println(maxTravelTime);

  upOffset = EEPROM[0];
  if (upOffset == 255 ) {
    upOffset = 0;
  }
  downOffset = EEPROM[1];
  if (downOffset == 255 ) {
    downOffset = 0;
  }
  Serial.print(F("downoffset: ")); Serial.print((int)downOffset);
  Serial.print(F("  upOffset: ")); Serial.print((int)upOffset); Serial.write("\n");
  lastDebounceTime = millis();
} // setup

//  STAT() ///////////////////////////////////////////////
void STAT() {
  if (digitalRead(leftLS) == LOW) {
    Serial.write("a"); bluetooth.print("a");
  } else {
    Serial.write("b");
    bluetooth.print("b"); // not used on bluetooth tablet
  }
  if (digitalRead(rightLS) == LOW) {
    Serial.write("c"); bluetooth.print("c");
  } else {
    Serial.write("d");
    bluetooth.print("d"); // not used on bluetooth tablet
  }
  if (digitalRead(feedDetect) == LOW) {
    Serial.write("e"); bluetooth.print("e");
  } else {
    Serial.write("f"); bluetooth.print("f");
  }
  if (IsFeedLightOn()) {
    Serial.write("g"); bluetooth.print("g");
  } else {
    Serial.write("h"); bluetooth.print("h");
  }
  if (DipperUP == 1) {
    Serial.write("i"); bluetooth.print("i");
  } else {
    Serial.write("j"); bluetooth.print("j");
  }
  if (Speakerval == 1) {
    Serial.write("k"); bluetooth.print("k");
  } else {
    Serial.write("l"); bluetooth.print("l");
  }
  if (digitalRead(dipperPB) == LOW) {
    Serial.write("m");
    bluetooth.print("m"); // not used on bluetooth tablet
  } else {
    Serial.write("n");
    bluetooth.print("n"); // not used on bluetooth tablet
  }

  if (digitalRead(dipperupLS) == LOW) {
    Serial.write("o");
    bluetooth.print("o");
  } else {
    Serial.write("p");
    bluetooth.print("p"); // not used on bluetooth tablet
  } // else

  if (digitalRead(puffPin) == LOW){
   Serial.write("u");  bluetooth.print("u");
  } else {
   Serial.write("t");  bluetooth.print("t");  
  }
  Serial.write("\n"); bluetooth.print("\n");
} // STAT

// ROTATE A NUMBER OF STEPS ///////////////////////////////////////////////////////////////////////////
void rotate(int steps, float speed) {
  //rotate a specific number of microsteps (8 microsteps per step) - (negative for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (steps > 0) ? HIGH : LOW;
  steps = abs(steps);
  digitalWrite(DIR_PIN, dir);
  float usDelay = (1 / speed) * delay_time; // increase for less microsteps ie 700 for single step, 70 for 1/8 step
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
}

// ROTATE A CERTAIN NUMBER OF DEGREES ////////////////////////////////////C
//
void rotateDeg(float deg, float speed) {
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0) ? HIGH : LOW;
  digitalWrite(DIR_PIN, dir);
  int steps = abs(deg) * (steps_total / 360.0);
  float usDelay = (1 / speed) * delay_time; // increase for less microsteps ie 700 for single step. , 70 for 1/8 step
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
}

//   UPStepper() /////////////////////////////////////////
void UPStepper() {
  Serial.write("q\n"); bluetooth.print("q\n");
  accumulatedTravelTime = millis() ;                  // start timing the while loop
  while (digitalRead(dipperupLS) != LOW) {
    rotate(-8, .40);                             // forward for 8 microsteps = 1 step at a time (was .20 originally)
    // check if we've exceeded the amount of time needed stop
    if ((millis() - accumulatedTravelTime) > maxTravelTime) {
      Serial.write("y-dipper ls fault\n"); bluetooth.print("y\n");
      dipperLSfault = true;
      break;
    } // if millis
  } // while
  DipperUP = 1 ; // dipper up
  DipperDN = 0 ; // dipper down
  //   if (digitalRead(dipperupLS) == LOW) {
  if (!(dipperLSfault)) {
    Serial.write("i\n"); bluetooth.print("i\n");
  }
} // UPStepper

//   DOWNStepper() /////////////////////////////////////////
void DOWNStepper() {

#if !defined(TESTERMODE)
  if ((digitalRead(dipperupLS) != LOW) && (dipperLSfault == false)) {
    UPStepper();  // make sure we know where we're at
  }
#endif

  Serial.write("r\n"); bluetooth.print("r\n");
  int num1 = 3000 - (upOffset * 10) + (downOffset * 10);
  Serial.write("Rotating :");
  Serial.print(num1);
  Serial.write("\n");
  rotate(num1, .40); // speed 1 (was .25)
  Serial.write("Rotating :");
  Serial.println("1500");
  rotate(1500, .10); // speed 2 slower in liquid
  Serial.write("j\n"); bluetooth.print("j\n");
  DipperUP = 0 ; // dipper up
  DipperDN = 1 ; // dipper down
  dipperLSfault = false;

} // DOWNStepper

//  play() ////////////////////////////////////////////////
void play() {
  // Humans can hear sounds from about 16 to 20,000 Hz (20 kHz)
  // the range of the rat's hearing is around 200 Hz to 80 or 90 kHz
  // TimerFreeTone(Speaker_PIN, FREQUENCY, DURATION);
  tone(speaker, 4500); // Uses TIMER2 which messes with PWM on D3 and D11
}

// processCommand()   ////////////////////////////////////////////

char processCommand(char c1)
{
  switch (c1) {
    case 'A':
      STAT();
      return 0;
      break;
    case 'B':
      digitalWrite(puffPin, LOW); Serial.write("u\n");  bluetooth.print("u\n");
      speakerOff();
      FeedLightOff();
      UPStepper();
      DOWNStepper();
      autoCycle = false;
      autoCounter = 0;
      dipperLSfault = false;
      ManualMode = false;
      return 0;
      break;
    case 'C':
      FeedLightOn();
      return 0;
      break;
    case 'D':
      FeedLightOff();
      return 0;
      break;
    case 'E':
      UPStepper();
      return 0;
      break;
    case 'F':
      DOWNStepper();
      return 0;
      break;
    case 'G':
      speakerOn();
      return 0;
      break;
    case 'I':
      digitalWrite(puffPin, HIGH); Serial.write("t\n");  bluetooth.print("t\n");
      return 0;
      break;
    case 'J':
      digitalWrite(puffPin, LOW); Serial.write("u\n");  bluetooth.print("u\n");
      return 0;
      break;
    case 'X':
      ManualMode = false;
      Serial.write("ManualMode false\n");
      return 0;
      break;
    case'Z':
      Serial.write("z\n");  bluetooth.print("z\n"); // NO-OP
      delay(50);
      return 0;
      break;
    case '^':
      autoCycle = !autoCycle; // change the autocycle state
      if (autoCycle == false) {
        autoCounter = 0;
      }
      return 0;
      break;
    case '+':
      //Serial.write("+ processed\n");
      downOffset = 0;
      if (upOffset < 254) {
        upOffset += 1;
      }
      //Serial.write("downoffset: "); Serial.print((int)downOffset);
      //Serial.write("  upOffset: "); Serial.print((int)upOffset); Serial.write("\n");
      EEPROM.update(0, upOffset);
      EEPROM.update(1, downOffset);
      return 0;
      break;
    case '-':
      //Serial.write("- processed\n");
      if (downOffset < 254) {
        downOffset += 1;
      }
      upOffset = 0;
      //Serial.write("downoffset: "); Serial.print((int)downOffset);
      //Serial.write("  upOffset: "); Serial.print((int)upOffset); Serial.write("\n");
      EEPROM.update(0, upOffset);
      EEPROM.update(1, downOffset);
      return 0;
      break;
    case '@':
      //Serial.write("& processed\n");
      EEPROM[0] = 255; upOffset = 0;
      EEPROM[1] = 255; downOffset = 0;
      //Serial.write("downoffset: "); Serial.print((int)downOffset);
      //Serial.write("  upOffset: "); Serial.print((int)upOffset); Serial.write("\n");
      return 0;
      break;
    case '?':
      //Serial.write("? processed\n");
      Serial.print("\n\nAcuiSee-vR Version: "); Serial.println(Version);
      Serial.write("downoffset: "); Serial.print((int)downOffset);
      Serial.write("  upOffset: "); Serial.print((int)upOffset); Serial.write("\n");
      Serial.write("ManualMode: ");
      if (ManualMode == true) {
        Serial.write("TRUE\n");
      } else {
        Serial.write("FALSE\n");
      }
#if defined(TESTERMODE)
      Serial.println("TESTERMODE DEFINED");
#else
      Serial.println("TESTERMODE NOT DEFINED");
#endif
      Serial.print("maxTravelTime = "); Serial.println(maxTravelTime);

      bluetooth.print("\n\nAcuiSee-vR Version: "); bluetooth.print (Version); bluetooth.print("\n");
      bluetooth.print("downoffset: "); bluetooth.print((int)downOffset); bluetooth.print("\n");
      bluetooth.print("  upOffset: "); bluetooth.print((int)upOffset); bluetooth.print("\n");
      return 0;
      break;
    default:
      return 0;
  } // of switch
} // end of processCommand()

// readSerialPort()   ////////////////////////////////////////////
char readSerialPort(void) {

  while (Serial.available() > 0) { // Don't read unless you know there is data
    inChar = Serial.read(); // Read a character

    // ASCII Decimal Range for A through Z caps only or ^(auto) or + or - or ?
    if (((inChar >= 63) && (inChar <= 90))      // ascii A through Z, ?, and @
        || (inChar == 10)                     // ascii LF
        || (inChar == 13)                    // ascii CR
        || (inChar == 94)                   // ascii ^
        || (inChar == 43)                  // ascii +
        || (inChar == 45) ) {             // ascii -
      processCommand(inChar);
    } // of if inchar
  } // of While
} // of function readSerialPort

// readBlueTooth()  Bluetooth  ////////////////////////////////////////////
char readBlueTooth(void) {

  while (bluetooth.available() > 0) { // Don't read unless you know there is data

    ManualMode = false;                        // if we received a char, disable manual mode BT only
    ManualTimeoutMillis = millis();           // reset the idle timer to enable Manual mode

    Serial.print("bluetooth.available() = ");  //debug
    Serial.println(bluetooth.available());    //debug

    inChar = bluetooth.read();  // Read a character
    Serial.print("bt raw >>");
    Serial.print(inChar);
    Serial.print(", hex: ");
    Serial.println(inChar, HEX);   // prints value as string in hexadecimal (base 16):

    if (((inChar >= 63) && (inChar <= 90))      // ascii ?, @, A through Z,
        || (inChar == 10)                     // ascii LF
        || (inChar == 13)                    // ascii CR
        || (inChar == 94)                   // ascii ^
        || (inChar == 43)                  // ascii +
        || (inChar == 45) ) {             // ascii -

      // if we make it here, it is a good character that passed the filter tests
      Serial.print("bt filtered >>");
      Serial.print(inChar);
      Serial.print(", hex: ");
      Serial.println(inChar, HEX);               // prints value as string in hexadecimal (base 16):
      processCommand(inChar);                 // process the command
      badCharCounter = 0;
      badCharFlag = false;


    } else {

      // if we make it here, character failed the filter test, enable retries
      while (bluetooth.available() > 0) inChar = bluetooth.read();  // Flush the buffer

      delay(5);                                                     // Delay a slight amount
      bluetooth.print("s\n");                                      // send a retry character
      badCharCounter++;
      badCharFlag = true;
      badCharTimer = millis() ;
      Serial.write("s\n");                                         // send a retry character
      Serial.print("badCharCounter1: ");
      Serial.println(badCharCounter);
      // in the main loop check to see if retry needs to be sent again by checking badCharFlag and badChar Timer

    } // of if inchar
  } // of While
} // of function readBlueTooth

// FeedLightOn  ///////////////////////////////////////////////////////////
void FeedLightOn(void) {
  //feedLight_Redval = RED    ;  // feed light value 0-255
  feedLight_Grnval = GREEN  ;  // feed light value 0-255
  feedLight_Ylwval = BLUE   ;  // feed light value 0-255
  // analogWrite(feedLight_Red, feedLight_Redval);
  analogWrite(feedLight_Grn, feedLight_Grnval);
  analogWrite(feedLight_Ylw, feedLight_Ylwval);
  Serial.write("g\n"); bluetooth.print("g\n");
} //FeedLightOn
// FeedLightOn  ///////////////////////////////////////////////////////////

// FeedLightOff ///////////////////////////////////////////////////////////
void FeedLightOff(void) {
  //feedLight_Redval = 0;  // feed light value 0-255
  feedLight_Grnval = 0;  // feed light value 0-255
  feedLight_Ylwval = 0;  // feed light value 0-255
  // analogWrite(feedLight_Red, feedLight_Redval);
  analogWrite(feedLight_Grn, feedLight_Grnval);
  analogWrite(feedLight_Ylw, feedLight_Ylwval);
  Serial.write("h\n"); bluetooth.print("h\n");
} //FeedLightOff
// FeedLightOff ///////////////////////////////////////////////////////////

// IsFeedLightOn ///////////////////////////////////////////////////////////
boolean IsFeedLightOn(void) {
  if ((feedLight_Grnval == 0) && (feedLight_Ylwval == 0))
  {
    return false ;
  } else {
    return true;
  }
} //FeedLightOff
// FeedLightOff ///////////////////////////////////////////////////////////

// speakerOn    ///////////////////////////////////////////////////////////
void speakerOn() {
  Serial.write("k\n"); bluetooth.print("k\n");
  Speakerval = 1;
  // delay(50); // if not we get timer2 conflict with BT on pin 3
  play();
  // delay(50); // if not we get timer2 conflict with BT on pin 3
}
// speakerOn ///////////////////////////////////////////////////////////

// speakerOff  ///////////////////////////////////////////////////////////
void speakerOff() {
  Serial.write("l\n"); bluetooth.print("l\n");
  Speakerval = 0;
  noTone(speaker);
}
// speakerOff ///////////////////////////////////////////////////////////

// loop()   //////////////////////////////////////////////
void loop()
{
  //*************************************************
  // CHECK THE IO AND REPORT BACK IF STATE CHANGE
  //*************************************************

  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis() ;

    // check  leftLS
    if (digitalRead(leftLS) != leftLSval) {
      leftLSval = digitalRead(leftLS);
      if (leftLSval == LOW) {
        Serial.write("a\n"); bluetooth.print("a\n");
      } else {
        Serial.write("b\n");
        bluetooth.print("b\n"); // not used on bluetooth tablet
      }
    }// End of check leftLS

    // check rightLS
    if (digitalRead(rightLS) != rightLSval) {
      rightLSval = digitalRead(rightLS);
      if (rightLSval == LOW) {
        Serial.write("c\n"); bluetooth.print("c\n");
      } else {
        Serial.write("d\n");
        bluetooth.print("d\n"); // not used on bluetooth tablet
      }
    } // End of check rightLS

    // check feedDetect
    if (digitalRead(feedDetect) != feedDetectval) {
      feedDetectval = digitalRead(feedDetect);
      if (feedDetectval == LOW) {
        Serial.write("e\n"); bluetooth.print("e\n");
      } else {
        Serial.write("f\n"); bluetooth.print("f\n");
      }
    }  // End of check feedDetect

    // check Dipper UP/DWN Pushbutton
    if (ManualMode == true) {
      if ((digitalRead(dipperPB) != DipperPBval) ) {
        DipperPBval = digitalRead(dipperPB);
        if (digitalRead(dipperPB) == LOW) {
          Serial.write("m\n");
          bluetooth.print("m\n"); // not used on bluetooth tablet
          if (CounterA % 2 == 0) // if no remainder it's even
            UPStepper();       // Even
          else
            DOWNStepper();     // Odd
          CounterA++;        // increment counter
        } else {
          Serial.write("n\n");
          bluetooth.print("n\n"); // not used on bluetooth tablet
        }
      } // if dipperPB
    } // if manualmode is true

    // dipperLS
    if (digitalRead(dipperupLS) != DipperupLSval) {
      DipperupLSval = digitalRead(dipperupLS);
      if (digitalRead(dipperupLS) == LOW) {
        Serial.write("o\n");
        bluetooth.print("o\n"); // not used on bluetooth tablet
        CounterA = 1; // stepper is up so set counter to be odd
        // so it will send dipper down next push
        dipperLSfault = false; // if we detect the ls, reset the fault flag
      } else {
        CounterA = 0; // stepper is down so set counter to be even
        // so it will send dipper up next time it's pushed
        Serial.write("p\n");
        bluetooth.print("p\n"); // not used on bluetooth tablet
      } // else
    } // if dipperupLS

    // autoCycle Mode
    if (autoCycle == true) {
      if (autoCounter % 2 == 0) { // if no remainder it's even
        // Even autoCounter
        UPStepper();
        autoCounter++;        // increment counter
        FeedLightOn();
        speakerOn();
        delay(2000);
      }   else   {
        // Odd autoCounter
        speakerOff();
        FeedLightOff();
        DOWNStepper();
        autoCounter++;     // increment counter
        delay(2000);
      } // if dipperPB
    } // if AutoCycle

  } // end of if millis

  //*************************************************
  // END OF CHECK IO BLOCK
  //*************************************************

  // Read and process any commands that have arrived via USB
  if (Serial.available()    > 0) readSerialPort(); // serial port
  if (bluetooth.available() > 0) readBlueTooth();  // bluetooth port
  // execute a BT character retry if necessary
  if (((millis() - badCharTimer) > badCharTimerLimit) && (badCharFlag == true)) {

    // send out the next retry request
    bluetooth.print("s\n");                                      // send a retry character
    badCharCounter++;
    badCharTimer = millis() ;
    Serial.write("s\n");                                         // debug echo
    Serial.print("badCharCounter2: ");
    Serial.println(badCharCounter);
  }

  // Blink the run light
  currentMillis = millis();
  if (dipperLSfault) {
    divisor = 10;
  } else {
    divisor = 1;
  }
  if (currentMillis - previousMillis > interval / divisor) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    digitalWrite(runLight,  !digitalRead(runLight)); // change state of run light
  }

  // Manual mode enable

  // if we haven't received a BT character in 29 seconds, ping android
  if (((millis() - ManualTimeoutMillis) > 29000) && (ManualMode == false)) {
    if (OneShot != 1) {
      bluetooth.print("x\n");
      Serial.write("x\n");
    }
    OneShot = 1;
  }
  // if we havene't received a BT char after ping - go to manual mode
  if (((millis() - ManualTimeoutMillis) > 30000) && (ManualMode == false)) {
    ManualMode = true;
    Serial.write("ManualMode true\n");
    OneShot = 0; // enable the one shot
  }




} // loop
