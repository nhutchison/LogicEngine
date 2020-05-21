// Arduino Libraries
#include "FastLED.h"
#include "Wire.h"

// Local .h files in the same directory as the main sketch
#include "config.h"
//#include "font_fld_5px.h"
#include "fld_font.h"


///
// Board validity checks
///

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #warning [not an actual error dont worry] - Compiling for a Teensy 3.1 or 3.2
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)  || defined(__AVR_ATmega168__)  
  #warning [not an actual error dont worry] - Compiling for an AT328P (Arduino Uno etc)
#elif defined(__SAMD21G18A__)
  #warning [not an actual error dont worry] - Compiling for Reactor Zero
#elif defined(ARDUINO_ARCH_ESP32)
  #warning [not an actual error dont worry] - Compiling for ESP32  
#else
  #error UNRECOGNIZED BOARD! Are you sure you chose the correct Board from the Tools menu?
#endif

///
// LED Banks
///
CRGB front_leds[NUM_FRONT_LEDS];
CRGB rear_leds[NUM_REAR_LEDS];
CRGB statusLED[1];

////////////////////////
//Teeces PSI's...
#if (TEECESPSI>0)
  const int PSIpause[2] = { 3000, 6000 };
  const byte PSIdelay[2]PROGMEM = { 25, 35 };
  #include <LedControl.h>
  #undef round
  LedControl lcChain = LedControl(TEECES_D_PIN, TEECES_C_PIN, TEECES_L_PIN, 2);
  byte PSIstates[2] = { 0, 0 };
  unsigned long PSItimes[2] = { millis(), millis() };
  unsigned int PSIpauses[2] = { 0, 0 };
#endif

///
// status LED related...
///
bool statusFlipFlop=0;
bool prevStatusFlipFlop=1;
#define slowBlink 2000 //number of millis between Status LED changes in normal mode
#define fastBlink 100  //number of millis between Status LED changes in Adjust mode
unsigned long prevFlipFlopMillis = 0;
int statusFlipFlopTime = slowBlink;
// Prevent updates to the LED's when getting data on serial
bool dataRcvInProgress = false;

///
// Command loop processing times
///
unsigned long previousMillis = 0;
unsigned long interval = 5;  // Limits us to 200 updates per second.

///
// Pattern State
///
bool firstTime[3];
bool patternRunning[3] = {false, false, false};
int lastEventCode[3] = {defaultPattern, defaultPattern, defaultPattern};
uint8_t globalPatternLoops;
int updateLed = 0;

// Timing values received from command are stored here.
bool timingReceived = false;
unsigned long commandTiming = 0;
unsigned long doNext[3];
unsigned long globalTimeout[3];

// make hue global ;)
uint8_t hue = 0;

uint8_t front_cnt = 0;
uint8_t rear_cnt = 0;


///
// Scrolling text stuff
///
uint8_t scrollCount[3] = {0,0,0};
uint8_t maxScrollCount[3] = {0,0,0};
uint8_t currentCharShiftsRemaining[3] = {0,0,0};
uint8_t totalShiftsForChar[3] = {0,0,0};
//bool scrollDisplayFirstTime[3] = {false};

// Function prototype for helper functions
void fill_row_front(uint8_t row, CRGB color, uint8_t scale_brightness=0);
void fill_row_rear(uint8_t row, CRGB color, uint8_t scale_brightness=0);

///
// Different board setup stuff....
///

//////////////////////////////////
//without RTCZero, millis() counts will be all off on the Reactor Zero
//this shouldn't be necesaary if using a Teensy, AVR or ESP32
#if defined(__SAMD21G18A__)
  #include <RTCZero.h>
  RTCZero rtc;
#endif

void setup() { 
  
  // Setup two arrays of LED's for the front and rear Logics
  FastLED.addLeds<NEOPIXEL, FRONT_PIN>(front_leds, NUM_FRONT_LEDS); 
  FastLED.addLeds<NEOPIXEL, REAR_PIN>(rear_leds, NUM_REAR_LEDS); 

  // Setup the Status LED on the control board
  FastLED.addLeds<NEOPIXEL, STATUSLED_PIN>(statusLED, 1); 

  FastLED.setBrightness(20);

#ifdef DEBUG
    // Pause to allow board init.
    delay(2000);
#endif  

  // Setup the debug Serial Port.
  SerialUSB.begin(BAUDRATE);
  debugSerialPort = &SerialUSB;
  Serial1.begin(BAUDRATE);
  serialPort = &Serial1;

  // Setup I2C
  Wire.begin(I2CAdress);                   // Start I2C Bus as Master I2C Address
  Wire.onReceive(receiveEvent);            // register event so when we receive something we jump to receiveEvent();

// Setup clock for SAMD21
#if defined(__SAMD21G18A__)
  rtc.begin();
  rtc.setTime(0, 0, 0);
#endif

#if (TEECESPSI>0)
  lcChain.shutdown(0, false); //take the device out of shutdown (power save) mode
  lcChain.clearDisplay(0);
  lcChain.shutdown(1, false); //take the device out of shutdown (power save) mode
  lcChain.clearDisplay(1);
  lcChain.setIntensity(0, FPSIbright); //Front PSI
  lcChain.setIntensity(1, RPSIbright); //Rear PSI
  #if (TEECESPSI>1)
    while (1==1) {
      setPSIstate(0, 0);
      setPSIstate(1, 0);
      delay(1000);
      setPSIstate(0, 6);
      setPSIstate(1, 6);
      delay(1000);
    }        
  #endif //(TEECESPSI>1)
#endif // (TEECESPSI>0)

  // Fill the Array with a set of colors to start with...
  for( int i = 0; i < NUM_FRONT_LEDS; i++) {
    frontColorIndex[i] = random16();
  }

  for( int i = 0; i < NUM_REAR_LEDS; i++) {
    rearColorIndex[i] = random(255);
  }

  // Initialise the pattern arrays
  //for (int i=0; i<3; i++){
  //  lastEventCode[i] = defaultPattern;
  //}


  // Setup Status LED
  statusLED[0] = 0x008800;
  statusLED[0] %= 20;
  updateDisplays();
}

///
// Helper Functions 
///

// This updates the FastLED's only if we're not receiving a command
// When updating the LED's, the interrupts are disabled which can
// cause issues with Serial / i2c command handling
void updateDisplays(){
  if (!dataRcvInProgress) FastLED.show();
}

// This will toggle the Status LED colors on the control board.
// Happy blinky light.
void setStatusLED() {
    if (statusFlipFlop == 0) {
      statusLED[0] = 0x000088;
    }
    else {
      statusLED[0] = 0x008800;
    }
    statusLED[0] %= 20;
}

void allOFF(int logicDisplay, bool showLED, unsigned long runtime=0)
{
  if (logicDisplay == 0){
    DEBUG_PRINT_LN("ADD ALL HANDLING");
    return;
  }
  
  if (firstTime[logicDisplay-1]) {
    //DEBUG_PRINT_LN("All Off");
    // If the address is 0, turn all off, else turn only one display off
    firstTime[logicDisplay-1] = false;
    patternRunning[logicDisplay-1] = true;
    // Setup timing
    if ((runtime != 0) && (!timingReceived)) set_global_timeout(logicDisplay, runtime);
    if (timingReceived) set_global_timeout(logicDisplay, commandTiming);
    //DEBUG_PRINT_LN(runtime);
  }
  
  //DEBUG_PRINT_LN("LED All Off");
  //FastLED.clear();
  // Can't use FastLED.clear() since it clear's all LED's!

  int start_row, end_row = 0;

  
  if (logicDisplay == FLD_TOP)
  {
    start_row = 0;
    end_row = 5;
  }
  else if (logicDisplay == FLD_BOTTOM)
  {
    start_row = 5;
    end_row = 10;
  }

  if ((logicDisplay == FLD_TOP) || (logicDisplay == FLD_BOTTOM)) {
    for (int i = start_row; i < end_row; i++) {
       fill_row_front(i, 0x000000);
    }
  }
  else if (logicDisplay == RLD) {
    for (int i = 0; i < 4; i++) {
      fill_row_rear(i, 0x000000);
    }
  }
  
  if (showLED) updateDisplays();

  if (logicDisplay == 0) {
    if ((runtime != 0) || (timingReceived)) {
      // Check for the global timeout to have expired.
      for(int i=0; i<3;i++){
        globalTimerDonedoRestoreDefault(i);
      }
    }
  } else {
    if ((runtime != 0) || (timingReceived)) {
      globalTimerDonedoRestoreDefault(logicDisplay);
    }
  }
}

// Row is defined as Top to bottom 0-9, where 0 is Top Logic, Top Row, and 9 is Bottom Logic, Bottom Row.
// Note that color can be either CRGB or CHSV ;)
void fill_row_front(uint8_t row, CRGB color, uint8_t scale_brightness) {
  
  // If row is less than 5, it's the top logic, else it's the bottom logic
  if (row < FRONT_ROW) {
    for (int i = 0; i < FRONT_COL; i++) {
      int8_t ledIndex = frontTopLedMatrix[i][row];
      if (ledIndex != -1) {
        front_leds[ledIndex] = color;
        if (scale_brightness != 0) front_leds[ledIndex] %= scale_brightness;
      }
    }
  }
  // it's the bottom logic
  else
  {
    for (int i = 0; i < FRONT_COL; i++) {
      int8_t ledIndex = frontBottomLedMatrix[i][row - FRONT_ROW];
      if (ledIndex != -1) {
        front_leds[ledIndex] = color;
        if (scale_brightness != 0) front_leds[ledIndex] %= scale_brightness;
      }
    }
  }
}


// Row is defined as Top to bottom 0-3.
// Note that color can be either CRGB or CHSV ;)
void fill_row_rear(uint8_t row, CRGB color, uint8_t scale_brightness) {
  
  for (int i = 0; i < REAR_COL; i++) {
    int8_t ledIndex = rearLedMatrix[i][row];
    if (ledIndex != -1) {
      rear_leds[ledIndex] = color;
      if (scale_brightness != 0) rear_leds[ledIndex] %= scale_brightness;
    }
  }
}


///
// Timing functions
///

//set the global pattern timeout
void set_global_timeout(int logicDisplay, unsigned long timeout)
{
// use 256 to set as "always on"
// 256 sec == ~4 mins. To make the pattern run longer, square the value
// resulting in ~18 hours
  if (timeout == 256) timeout *= timeout;
  globalTimeout[logicDisplay-1] = millis() + (timeout * 1000);
  DEBUG_PRINT("Current time "); DEBUG_PRINT_LN(millis());
  DEBUG_PRINT("Timeout received "); DEBUG_PRINT_LN(timeout);
  DEBUG_PRINT("End time Timeout "); DEBUG_PRINT_LN(globalTimeout[logicDisplay-1]);
}

// Check if the global timeout has expired.
// This will return true if the timer has expired.
// If "alwaysOn" is set, the function will never return true.
bool globalTimeoutExpired(int logicDisplay)
{
  bool timerExpired = false;
  if ((millis() >= globalTimeout[logicDisplay-1]) && (!alwaysOn)){ 
    timerExpired = true;
    DEBUG_PRINT("Global Timer Expired at  "); DEBUG_PRINT_LN(millis());
  }
  return timerExpired;
}

void globalTimerDonedoRestoreDefault(int logicDisplay)
{
  if (globalTimeoutExpired(logicDisplay)) {
    // Set the loops to 0 to catch any cases like that.
    globalPatternLoops = 0;
    // Global timeout expired, go back to default mode.
    lastEventCode[logicDisplay-1] = defaultPattern;
    patternRunning[logicDisplay-1] = false;
  }
}

//This is the non-blocking delay function
// When called it sets some global variables to allow checking of timer exipration
// To check if the timer has expired, call checkDelay()
void set_delay(int logicDisplay, unsigned long timeout)
{
  doNext[logicDisplay - 1] = millis() + timeout;
  //DEBUG_PRINT("Set delay to "); DEBUG_PRINT_LN(doNext[logicDisplay]);
}

// Call this to see if the timer for set_delay() has expired
bool checkDelay(int logicDisplay)
{
  bool timerExpired = false;
  if (millis() >= doNext[logicDisplay-1]) timerExpired = true;
  return timerExpired;
}


///
// TEECES CODE
///

#if (TEECESPSI>0)
 /*
   Each PSI has 7 states. For example on the front...
    0 = 0 columns Red, 6 columns Blue
    1 = 1 columns Red, 5 columns Blue (11)
    2 = 2 columns Red, 4 columns Blue (10)
    3 = 3 columns Red, 3 columns Blue  (9)
    4 = 4 columns Red, 2 columns Blue  (8)
    5 = 5 columns Red, 1 columns Blue  (7)
    6 = 6 columns Red, 0 columns Blue
*/
void setPSIstate(bool frontRear, byte PSIstate) {
  //set PSI (0 or 1) to a state between 0 (full red) and 6 (full blue)
  // states 7 to 11 are moving backwards
  if (PSIstate > 6) PSIstate = 12 - PSIstate;
  for (byte col = 0; col < 6; col++) {
    if (col < PSIstate) {
      if (col % 2) lcChain.setColumn(frontRear, col, B10101010);
      else lcChain.setColumn(frontRear, col,      B01010101);
    }
    else {
      if (col % 2) lcChain.setColumn(frontRear, col, B01010101);
      else lcChain.setColumn(frontRear, col,      B10101010);
    }
  }
}

void updateTeeces()
{
#if (TEECESPSI==1)
  for (byte PSInum = 0; PSInum < 2; PSInum++) {
    if (millis() - PSItimes[PSInum] >= PSIpauses[PSInum]) {
      //time's up, do something...
      PSIstates[PSInum]++;
      if (PSIstates[PSInum] == 12) PSIstates[PSInum] = 0;
      if (PSIstates[PSInum] != 0 && PSIstates[PSInum] != 6) {
        //we're swiping...
        PSIpauses[PSInum] = pgm_read_byte(&PSIdelay[PSInum]);
      }
      else {
        //we're pausing
        PSIpauses[PSInum] = random(PSIpause[PSInum]);
        //decide if we're going to get 'stuck'
        if (random(100) <= PSIstuck) {
          if (PSIstates[PSInum] == 0) PSIstates[PSInum] = random(1, 3);
          else PSIstates[PSInum] = random(3, 5);
        }
      }
      setPSIstate(PSInum, PSIstates[PSInum]);
      PSItimes[PSInum] = millis();
    }
  }
#endif
}
#endif //(TEECESPSI>0)

///
// Patterning code
///

void randomBlinkies(int logicDisplay, int mode){
  // If we want to switch between palettes, we can

  // If it is time to update the text
  if (checkDelay(logicDisplay)) {
    /*
    if (mode = 2) 
    {
      ChangePalettePeriodically(logicDisplay);
  
      // Crossfade current palette slowly toward the target palette
      //
      // Each time that nblendPaletteTowardPalette is called, small changes
      // are made to currentPalette to bring it closer to matching targetPalette.
      // You can control how many changes are made in each call:
      //   - the default of 24 is a good balance
      //   - meaningful values are 1-48.  1=veeeeeeeery slow, 48=quickest
      //   - "0" means do not change the currentPalette at all; freeze
  
      // Because of the state system, although we update less frequently, we bash throught this too
      // quickly ... Update to be fully state driven!
      uint8_t maxChanges = 2;
      if (logicDisplay < 3) {
        nblendPaletteTowardPalette( frontCurrentPalette, frontTargetPalette, maxChanges);
      }
      else if (logicDisplay == 3) {
        nblendPaletteTowardPalette( rearCurrentPalette, rearTargetPalette, maxChanges);
      }
    }
  */
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    startIndex %= 255;
    if (logicDisplay == 0) {
      FillLEDsFromPaletteColors(FLD_TOP);
      FillLEDsFromPaletteColors(FLD_BOTTOM);
      FillLEDsFromPaletteColors(RLD);
    }
    else if (logicDisplay == FLD_TOP) {
      FillLEDsFromPaletteColors(FLD_TOP);
    } else if (logicDisplay == FLD_BOTTOM) {
      FillLEDsFromPaletteColors(FLD_BOTTOM);
    } else if (logicDisplay == RLD) {
      FillLEDsFromPaletteColors(RLD);
    } else {
      DEBUG_PRINT_LN("Invalid Display address received");
    }
  
    updateDisplays();

    set_delay(logicDisplay, 1000 / UPDATES_PER_SECOND);
  }
}


void FillLEDsFromPaletteColors(int logicDisplay)
{
  uint8_t brightness = 255;
  int chanceChange;
  uint8_t ledIndex, count, tempStep, currentColor = 0;
  int start_row, end_row = 0;

  if (logicDisplay == FLD_TOP) {
    // Set start and end rows
    start_row = 0;
    end_row = 5;
  }
  if (logicDisplay == FLD_BOTTOM) {
    // Set start and end rows
    start_row = 5;
    end_row = 10;
  }
  if ((logicDisplay == FLD_TOP) || (logicDisplay == FLD_BOTTOM)){
    // Updated algorithm that uses the matrix and can manage dead pixels ;)
    // Also helps to remove "waves" from the updates.
    for (int i=0; i< FRONT_COL; i++){
      for (int y=start_row; y<end_row; y++){
        // Use a Percentage chance that we update and LED.  Aaain, persuit of randomness!
        chanceChange = random(100);
        if (chanceChange < PER_CHANGE_CHANCE) {
          ledIndex = bothFrontLedMatrix[i][y];
          if (ledIndex != -1) {
            front_leds[ledIndex] = ColorFromPalette( frontTargetPalette, frontColorIndex[ledIndex]/* + sin8(count*32)*/, brightness);
            // This is all about randomisation
            // We pick a random step size up to the MAX Step we allow.
            // All in the name of removing visible repeated patterning.
            tempStep = frontColorIndex[ledIndex] + random(FRONT_COLOR_STEP);
            //tempStep %= 255;
            //colorIndex += FRONT_COLOR_STEP; //colorIndex %= 255;  // Update the index in the color palette
            frontColorIndex[ledIndex] = tempStep;
            count++;
          }
        }
      }
    }
  } else if (logicDisplay == RLD){
    for (int i=0; i< REAR_COL; i++){
      for (int y=0; y<REAR_ROW; y++){
        chanceChange = random(100);
        if (chanceChange < PER_CHANGE_CHANCE) {
          ledIndex = rearLedMatrix[i][y];
          if (ledIndex != -1) {
            rear_leds[ledIndex] = ColorFromPalette( rearTargetPalette, rearColorIndex[ledIndex]/* + sin8(count*32)*/, brightness);
            tempStep = rearColorIndex[ledIndex] + REAR_COLOR_STEP;
            //tempStep %= 255;
            //colorIndex += FRONT_COLOR_STEP; //colorIndex %= 255;  // Update the index in the color palette
            rearColorIndex[ledIndex] = tempStep;
            count++;
          }
        }
      }
    }
  }
}


void ChangePalettePeriodically(int logicDisplay)
{
  uint8_t secondHand = (millis() / 1000) % 40;
  static uint8_t lastSecond = 99;
  
  if( lastSecond != secondHand) {
    lastSecond = secondHand;
    if( secondHand ==  0)  { 
      if (logicDisplay == 3) rearTargetPalette = front_gp;
      } else {
        frontTargetPalette = rear_gp;
      }
    if( secondHand == 20)  { 
      if (logicDisplay == 3) {
        rearTargetPalette = rear_gp; 
      } else {
        frontTargetPalette = front_gp;
      }
    }
  }
}


///
// Scrolling text stuff
//
///

void SetRow(uint8_t row, unsigned char RowState, CRGB color){
  //for(int Col=0; Col<8; Col++){
  //  VMagicPanel[LEDRow][Col]=((RowState >> Col) & 1);
  //}
  // If row is less than 5, it's the top logic, else it's the bottom logic
  if (row < FRONT_ROW) {
    for (int i = 0; i < FRONT_COL; i++) {
      int8_t ledIndex = frontTopLedMatrix[i][row];
      if (ledIndex != -1) {
        if ((RowState >> ((FRONT_COL - 1) - i)) & 1) {
          //DEBUG_PRINT_LN("Pix on");
          front_leds[ledIndex] = color;
        }
        else
        {
          front_leds[ledIndex] = 0x000000;
          //DEBUG_PRINT_LN("Pix off");
        }
      }
    }
  }
  // it's the bottom logic
  else
  {
    for (int i = 0; i < FRONT_COL; i++) {
      int8_t ledIndex = frontBottomLedMatrix[i][row - FRONT_ROW];
      if (ledIndex != -1) {
        if ((RowState >> ((FRONT_COL - 1) - i)) & 1) {
          //DEBUG_PRINT_LN("Pix on");
          front_leds[ledIndex] = color;
        }
        else
        {
          front_leds[ledIndex] = 0x000000;
          //DEBUG_PRINT_LN("Pix off");
        }
      }
    }
  } 
}

//////////////////////
// Set String
void setText(byte logicDisplay, const char* message)
{
  strncpy(logicText[logicDisplay-1], message, MAXSTRINGSIZE);
  logicText[logicDisplay-1][MAXSTRINGSIZE]=0; // just in case
}


// Scroll Message
// Pass the message, the logic display to use, the mode (alphabet) and the color of the text
// Valid logicDisplay is:
//                        1 - Front Top, 
//                        2 - Front Bottom,
//                        3 - Rear
void scrollMessage(char messageString[], int logicDisplay, int mode, CRGB color) {

  if (logicDisplay == 0)
  {
    DEBUG_PRINT_LN("ADD ALL HANDLING");
    return;
  }

  // setup first time stuff
  if (firstTime[logicDisplay-1]) {
    firstTime[logicDisplay-1] = false;
    patternRunning[logicDisplay-1] = true;
    
    // Count the number of characters we've loaded.
    scrollCount[logicDisplay-1] = 0;
    maxScrollCount[logicDisplay-1] = strlen((const char*)messageString);

    // Count the number of times we've moved the current character
    currentCharShiftsRemaining[logicDisplay-1] = 0;
    totalShiftsForChar[logicDisplay-1] = 0;

    // Clear temp buffer to save the end of the last message being re-displayed.
    for (int i=0; i<10; i++) bufferLong[logicDisplay-1][i] = 0;
    
    //DEBUG_PRINT("String Length "); DEBUG_PRINT_LN(maxScrollCount[logicDisplay]);
  }

  updateLed = 0;
  int myChar=0;

  // If it is time to update the text
  if (checkDelay(logicDisplay)) {
    //DEBUG_PRINT_LN("Timeout Expired");
    // Check if we need to load the new character
    if (currentCharShiftsRemaining[logicDisplay-1] == 0) {
      // load the next character...
      //DEBUG_PRINT_LN("Loading next Character");
      myChar =  pgm_read_byte_near(messageString + scrollCount[logicDisplay-1]); 
      // If its a valid character, then load it.
      if (myChar != 0){
          loadBufferLong(myChar, logicDisplay, mode);
          scrollCount[logicDisplay-1]++;
          // Set the width of this character.
          currentCharShiftsRemaining[logicDisplay-1] = totalShiftsForChar[logicDisplay-1];
          //DEBUG_PRINT("Char Width: "); DEBUG_PRINT_LN(currentCharShiftsRemaining[logicDisplay]);
      }
    } else {
      // Rotate the current character until we need to load a new character
      if (currentCharShiftsRemaining[logicDisplay-1]) {
        //DEBUG_PRINT_LN("Rotating Character");
        //DEBUG_PRINT("Char Width Rem: "); DEBUG_PRINT_LN(currentCharShiftsRemaining[logicDisplay]);
        // Move the pixels
        shiftBuffer(logicDisplay);
        currentCharShiftsRemaining[logicDisplay-1]--;
        printScrollBuffer(logicDisplay, color);
        updateLed = 1;
      }
      set_delay(logicDisplay, scrollDelay[logicDisplay-1]);
    } 
  }

  if (updateLed){
    //DEBUG_PRINT_LN("Update Display");
    updateDisplays();
  }

  //DEBUG_PRINT("Char "); DEBUG_PRINT(scrollCount[logicDisplay]); DEBUG_PRINT(" of "); DEBUG_PRINT_LN(maxScrollCount[logicDisplay]);
  //DEBUG_PRINT("Char Width: "); DEBUG_PRINT_LN(currentCharShiftsRemaining[logicDisplay]);

  // Might need updating for multi display message stuff ....
  if ((currentCharShiftsRemaining[logicDisplay-1] == 0) && (maxScrollCount[logicDisplay-1] == scrollCount[logicDisplay-1])) {
    DEBUG_PRINT_LN("Scrolling Done ");
    lastEventCode[logicDisplay-1] = defaultPattern;
    patternRunning[logicDisplay-1] = false;
  }
}

// Rotate the buffer
void shiftBuffer(int logicDisplay){
    for (int a=0;a<5;a++){                      // Loop 5 times for a 5x5 font, once per row.
        unsigned long x = bufferLong[logicDisplay-1][a*2];     // Get low buffer entry
        byte b = bitRead(x,31);                 // Copy high order bit that gets lost in rotation
        x = x<<1;                               // Rotate left one bit
        bufferLong[logicDisplay-1][a*2] = x;                   // Store new low buffer
        x = bufferLong[logicDisplay-1][a*2+1];                 // Get high buffer entry
        x = x<<1;                               // Rotate left one bit
        bitWrite(x,0,b);                        // Store saved bit
        bufferLong[logicDisplay-1][a*2+1] = x;                 // Store new high buffer
    }
}  

// Display Buffer on LED matrix
void printScrollBuffer(int logicDisplay, CRGB color){
  int row;
  for (int a=0;a<5;a++){                    // Loop 5 times for a 5x5 font  // Once per row.
    unsigned long x = bufferLong[logicDisplay-1][a*2+1];   // Get high buffer entry
    byte y = x;                             // Mask off first character
    if (logicDisplay == FLD_TOP) {
      row = a;
    }
    else if (logicDisplay == FLD_BOTTOM) {
      row = a+5;
    }
    SetRow(row, y, color);
    x = bufferLong[logicDisplay-1][a*2];                   // Get low buffer entry
    y = (x>>24);                            // Mask off second character
    SetRow(row, y, color);
    y = (x>>16);                            // Mask off third character
    SetRow(row, y, color);
    y = (x>>8);                             // Mask off forth character
    SetRow(row, y, color);
  }
}

// Load character into scroll buffer
void loadBufferLong(int ascii, int logicDisplay, int mode){
    if (ascii >= 0x20 && ascii <=0x7f){
        for (int a=0;a<5;a++){                      // Loop 5 times for a 5x5 font, once per row
            unsigned long c ; 
            if (mode == 1)
            {
              // Read from the English font table
              c = pgm_read_byte_near(font5x5 + ((ascii - 0x20) * 6) + a);     // Index into character table to get row data
            }
            else if (mode == 2)
            {
              // Read from the Aurek Besh table
              c = pgm_read_byte_near(aurabesh5x5 + ((ascii - 0x20) * 6) + a);     // Index into character table to get row data// To Do!!
            }
            unsigned long x = bufferLong[logicDisplay-1][a*2];     // Load current scroll buffer
            x = x | c;                              // OR the new character onto end of current
            bufferLong[logicDisplay-1][a*2] = x;                   // Store in buffer
            //DEBUG_PRINT("BufferLong: "); DEBUG_PRINT_LN(bufferLong[logicDisplay][a*2]);
        }

        if (mode == 1)
        {
           totalShiftsForChar[logicDisplay-1] = pgm_read_byte_near(font5x5 +((ascii - 0x20) * 6) + 5);     // Index into character table for kerning data
        }
        else if (mode == 2)
        {
           totalShiftsForChar[logicDisplay-1] = pgm_read_byte_near(aurabesh5x5 +((ascii - 0x20) * 6) + 5);     // Index into character table for kerning data
        }
    }
}


/////////
/////////
////////


void loop() {

  // Get current time.
  unsigned long currentMillis = millis();
  //uint8_t delta;

  if (currentMillis - previousMillis > interval)
  {
    //DEBUG_PRINT_LN("Main Loop Tick");
    previousMillis = currentMillis;

    // Handle each display in the Logic independently.
    for (int i=0; i<3; i++) {
      // The Logics address is 1-3, so set that correctly.
      if (patternRunning[i])
      {
        //DEBUG_PRINT("Pattern "); DEBUG_PRINT(lastEventCode[i]); DEBUG_PRINT(" runnning on LD "); DEBUG_PRINT_LN(i+1);
        runPattern(i+1, lastEventCode[i]);
      }
      else
      {
        //DEBUG_PRINT("No Pattern Running "); DEBUG_PRINT_LN(i);
        lastEventCode[i] == defaultPattern;
        runPattern(i+1, lastEventCode[i]);
      }
    }
/*
    // Grab the POT Average value.
    tempglobalPOTaverage = averagePOT();
    delta = (tempglobalPOTaverage >= previousglobalPOTaverage) ? tempglobalPOTaverage - previousglobalPOTaverage : previousglobalPOTaverage - tempglobalPOTaverage;
    
    // Allow you to debounce the POT :D
    if (delta > POT_VARIANCE_LEVEL){
      previousglobalPOTaverage = tempglobalPOTaverage;
      globalPOTaverage = tempglobalPOTaverage;
    }
*/
#if (TEECESPSI>0)
  // Do teeces updates
  updateTeeces();
#endif // (TEECESPSI>0)

  }  

  // Status LED Stuff.
  if (currentMillis - prevFlipFlopMillis >= statusFlipFlopTime) {
    statusFlipFlop = statusFlipFlop ^ 1;
    prevFlipFlopMillis=currentMillis;
  }
  
  setStatusLED();
  updateDisplays();
   
}

char scrolly[] PROGMEM ={"{}|~ "};

// The following takes the Pattern code, and executes the relevant function
// This allows i2c and serial inputs to use the same function to start patterns
// so we avoid the need to duplicate this code.
void runPattern(int logicDisplay, int pattern) {

  if (logicDisplay == 0)
  {
    DEBUG_PRINT_LN("Should never receive a logic address of 0.");
    return;
  }

  // Used to restore state if an invalid pattern code is received.
  int currentPattern = lastEventCode[logicDisplay-1];

  if (lastEventCode[logicDisplay-1] != pattern)
  {
    lastEventCode[logicDisplay-1] = pattern;
    firstTime[logicDisplay-1] = true;
  }
  else
  {
    firstTime[logicDisplay-1] = false;
  }

  switch (pattern) {
    case 0:
      allOFF(logicDisplay, true);
      break;
    case 1:
      randomBlinkies(logicDisplay, 1);
      break;
    case 2:
      // Set display to Top front
      //scrollMessage(scrolly, logicDisplay, 2, 0x0000ff);
      setText(logicDisplay, scrolly);
      scrollMessage(logicText[logicDisplay-1], logicDisplay, 2, 0x0000ff);
      break;
    case 3:
      // Random blinkies with colorshifts
      randomBlinkies(logicDisplay, 2);
      break;
    case 100:
      // Set display to Top front
      scrollMessage(logicText[logicDisplay-1], logicDisplay, 1, 0x0000ff);
      break;      
    default:
      DEBUG_PRINT("Pattern "); DEBUG_PRINT(pattern); DEBUG_PRINT_LN(" not valid.  Ignoring");
      lastEventCode[logicDisplay-1] = currentPattern;
      firstTime[logicDisplay-1] = false;
      break;
  }
 
/*
    // Display new pixel
    //front_leds[front_cnt] = CHSV( hue,187,255);
    fill_row_front(front_cnt, CHSV( hue,187,255));
    //rear_leds[rear_cnt] = CHSV( hue,187,255);
    fill_row_rear(rear_cnt, CHSV( hue,187,255));

    // Update color
    hue++;
    hue %= 255;
    updateDisplays();
    // Pause
    FastLED.delay(100);
    
    // Clear and setup for the next loop.
    fill_row_front(front_cnt, CRGB::Black);
    fill_row_rear(rear_cnt, CRGB::Black);
    front_cnt++; front_cnt %= FRONT_ROW * 2;
    rear_cnt++; rear_cnt %= REAR_ROW;
 */

}


// function that executes whenever data is received from an I2C master
// this function is registered as an event, see setup()
void receiveEvent(int eventCode) {

  dataRcvInProgress = true;
  while (Wire.available()) {

    // New I2C handling
    // Needs to be tested, but uses the same parser as Serial!
    bool command_available;
    char ch = (char)Wire.read();

    DEBUG_PRINT("I2C Character received "); DEBUG_PRINT_LN(ch);
    
    command_available=buildCommand(ch, cmdString);  // build command line
      
    if (command_available) 
    {
      parseCommand(cmdString);  // interpret the command
    } 
  }
  dataRcvInProgress = false;
}


// Manage all the serial stuff

/*
   SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEventRun(void)
{
  //if (serialPort->available()) serialEvent();
  if (debugSerialPort->available()) debugSerialEvent();
}

void debugSerialEvent() {

  DEBUG_PRINT_LN("Debug Serial In");
  bool command_available;

  dataRcvInProgress = true;
  while (debugSerialPort->available()) {  
    char ch = (char)debugSerialPort->read();  // get the new byte

    // New improved command handling
    command_available=buildCommand(ch, cmdString);  // build command line
    if (command_available) 
    {
      parseCommand(cmdString);  // interpret the command
    }
  }
  dataRcvInProgress = false;
  sei();
}

////////////////////////////////////////////////////////
// Command language - JawaLite emulation
///////////////////////////////////////////////////////


////////////////////////////////
// command line builder, makes a valid command line from the input
byte buildCommand(char ch, char* output_str)
{
  static uint8_t pos=0;
  switch(ch)
 {
    case '\r':                          // end character recognized
      output_str[pos]='\0';   // append the end of string character
      pos=0;        // reset buffer pointer
      return true;      // return and signal command ready
      break;
    default:        // regular character
      output_str[pos]=ch;   // append the  character to the command string
      if(pos<=CMD_MAX_LENGTH-1)pos++; // too many characters, discard them.
      break;
  }
  return false;
}

///////////////////////////////////
// command parser and switcher, 
// breaks command line in pieces, 
// rejects invalid ones, 
// switches to the right command
void parseCommand(char* inputStr)
{
  byte hasArgument=false;
  byte hasTiming=false;
  int argument;
  int address;
  int timing;
  byte pos=0;
  byte endArg=0;
  byte length=strlen(inputStr);
  if(length<2) goto beep;   // not enough characters

  DEBUG_PRINT(" Here's the input string: ");
  DEBUG_PRINT_LN(inputStr);
  
  // get the adress, one or two digits
  char addrStr[3];
  if(!isdigit(inputStr[pos])) goto beep;  // invalid, first char not a digit
    addrStr[pos]=inputStr[pos];
    pos++;                            // pos=1
  if(isdigit(inputStr[pos]))          // add second digit address if it's there
  {  
    addrStr[pos]=inputStr[pos];
    pos++;                            // pos=2
  }
  addrStr[pos]='\0';                  // add null terminator
  
  address= atoi(addrStr);        // extract the address

  //DEBUG_PRINT(" I think this is the address! ");
  //DEBUG_PRINT_LN(address);
  
  // check for more
  if(!length>pos) goto beep;            // invalid, no command after address
  
  // special case of M commands, which take a string argument
  if(inputStr[pos]=='M')
  {
    pos++;
    if(!length>pos) goto beep;     // no message argument
    doMcommand(address, inputStr+pos);   // pass rest of string as argument
    return;                     // exit
  }
  
  // other commands, get the numerical argument after the command character

  pos++;                             // need to increment in order to peek ahead of command char
  if(!length>pos) {hasArgument=false; hasTiming=false;}// end of string reached, no arguments
  else
  {
    for(byte i=pos; i<length; i++)
    {
      if (inputStr[i] == '|')
      {
        //we have a timing parameter for the T command.
        hasTiming = true;
        endArg = i;
        break;
      }
      if(!isdigit(inputStr[i])) goto beep; // invalid, end of string contains non-numerial arguments
    } 
    argument=atoi(inputStr+pos);    // that's the numerical argument after the command character
    hasArgument=true;
    
    if (hasTiming){
      timing=atoi(inputStr+endArg+1);
    }
    else {
      timing = 0;
    }
    /*
    DEBUG_PRINT(" I think this is the address! ");
    DEBUG_PRINT_LN(address);
    DEBUG_PRINT(" I think this is the Command! ");
    DEBUG_PRINT_LN(inputStr[pos-1]);
    DEBUG_PRINT(" I think this is the Command Value! ");
    DEBUG_PRINT_LN(argument);
    if (hasTiming){
      DEBUG_PRINT(" I think this is the Timing Value! ");
      DEBUG_PRINT_LN(timing);
    }
    */
  }
  
  // switch on command character
  switch(inputStr[pos-1])               // 2nd or third char, should be the command char
  {
    case 'T':
      if(!hasArgument) goto beep;       // invalid, no argument after command
      doTcommand(address, argument, timing);      
      break;
    case 'D':                           // D command is weird, does not need an argument, ignore if it has one
    case 'A':                           // A command does the same as D command, so just fall though.
      doDcommand(address);
      break;
    case 'C':                           // Set the speed for the scrolling text.
      doCcommand(address, argument);
      break;
    case 'P':    
      if(!hasArgument) goto beep;       // invalid, no argument after command
      doPcommand(address, argument);
      break;
    //case 'R':    
    //  if(!hasArgument) goto beep;       // invalid, no argument after command
    //  doRcommand(address, argument);
    //  break;
    //case 'S':    
    //  if(!hasArgument) goto beep;       // invalid, no argument after command
    //  doScommand(address, argument);
    //  break;
    default:
      goto beep;                        // unknown command
      break;
  }
  
  return;                               // normal exit
  
  beep:                                 // error exit
    // Dont know what this does ... idnoring it for now!
    //serialPort->write(0x7);             // beep the terminal, if connected
    return;
}

////////////////////
// Command Executors

// set text command
void doMcommand(int address, char* message)
{

  DEBUG_PRINT_LN();
  DEBUG_PRINT("Command: M ");
  DEBUG_PRINT("Address: ");
  DEBUG_PRINT(address);
  DEBUG_PRINT(" Argument: ");
  DEBUG_PRINT_LN(message);

  //serialPort->println();
  //serialPort->print("Command: M ");
  //serialPort->print("Address: ");
  //serialPort->print(address);
  //serialPort->print(" Argument: ");
  //serialPort->print(message);

  //TODO : Text on all
  //if(address==0) {setText(0, message); setText(1, message); setText(2, message);}
  if(address==1) {setText(FLD_TOP, message);}
  if(address==2) {setText(FLD_BOTTOM, message);}
  if(address==3) {setText(RLD, message);}  
  
}

// various commands for states and effects
void doTcommand(int address, int argument, int timing)
{
  /*
  DEBUG_PRINT_LN();
  DEBUG_PRINT("Command: T ");
  DEBUG_PRINT("Address: ");
  DEBUG_PRINT(address);
  DEBUG_PRINT(" Argument: ");
  DEBUG_PRINT_LN(argument);
  if (timing){
    DEBUG_PRINT(" Timing: ");
    DEBUG_PRINT_LN(timing); 
  }
  */

  if (timing != 0){
    DEBUG_PRINT_LN("Timing Value received in command");
    timingReceived = true;
    commandTiming = timing;
  }
  else {
    DEBUG_PRINT_LN("Disable Global Timing");
    timingReceived = false;
    commandTiming = 0;
  }
 
  // If the Top PSI was addressed, send the command to the top
  if (address == FLD_TOP)
  {
    DEBUG_PRINT_LN("Pattern request for Front Top");
    runPattern(FLD_TOP, argument);
  }
  // Else bottom front
  else if (address == FLD_BOTTOM)
  {
    DEBUG_PRINT_LN("Pattern request for Front Bottom");
    runPattern(FLD_BOTTOM, argument);
  }
  //else rear
  else if (address == RLD)
  {
    DEBUG_PRINT_LN("Pattern request for Rear");
    runPattern(RLD, argument);
  }
  //else all ...
  else if (address == 0){
    DEBUG_PRINT_LN("Pattern request for All");
    for (int i=1; i<4; i++) {
      runPattern(i, argument);
    }
  }
  else
  {
    DEBUG_PRINT("Address "); DEBUG_PRINT(address); DEBUG_PRINT_LN(" not recognised");
  }
  DEBUG_PRINT_LN("T command Handling End ");
}

void doDcommand(int address)
{
  /*
  DEBUG_PRINT_LN();
  DEBUG_PRINT("Command: D ");
  DEBUG_PRINT("Address: ");
  DEBUG_PRINT_LN(address); 
  */

  // Set the default scroll speed back to default. (75)
  for (int i=0; i<3; i++){
      scrollDelay[i] = 75;
    }
  
  for (int i=1; i<4; i++) {
    runPattern(i, defaultPattern);
  }
}

void doCcommand(int address, int argument)
{
  /*
  DEBUG_PRINT_LN();
  DEBUG_PRINT("Command: C ");
  DEBUG_PRINT("Address: ");
  DEBUG_PRINT_LN(address); 
  DEBUG_PRINT(" Argument: ");
  DEBUG_PRINT_LN(argument);
  */
  
  // If setting speed for all, set the speed for all
  if (address == 0){
    for (int i=0; i<3; i++){
      scrollDelay[i] = argument;
    }
  }
  
  // Else set the individual speeds.
  if (address == FLD_TOP)
  {
    scrollDelay[0] = argument;
  }   else if (address == FLD_BOTTOM) {
    scrollDelay[1] = argument;
  }  else if (address == RLD) {
   scrollDelay[2] = argument;
  }
}

// Parameter handling for PSI settings
void doPcommand(int address, int argument)
{
  /*
  DEBUG_PRINT_LN();
  DEBUG_PRINT("Command: P ");
  DEBUG_PRINT("Address: ");
  DEBUG_PRINT(address);
  DEBUG_PRINT(" Argument: ");
  DEBUG_PRINT_LN(argument);  
  */
  switch(address)
  {
    default:
      break;
  }  
}
