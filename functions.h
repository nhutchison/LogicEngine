// Globals that we need for the Status LED, etc

/////////////////////////////////////////
// a structure to keep our settings in...
// And dammn this is getting big!
// NOTE:  The storage on the Reactor Zero is wiped when you upload a new sketch.
//        There is no way around this!  So you probably need to remember your settings
//        Before you upload a new sketch.  I have provided a command to dump this
//        structure so that you can restore it afterwards.  It's a pain, but it's the 
//        best I can do on the Reacrot Zero.  The Teensy will not have this issue.
typedef struct {
  uint8_t writes; //keeps a count of how many times we've written settings to flash
  uint8_t maxBri;
  // Saturation (Desat) also doesn't really have a meaning, but again keeping for now.
  // Currently there's not a way to control brightness per display, but I'm leaving the hook here
  // so that I can come back and hopefully add it at some point.
  uint8_t frontTopDelay; uint8_t frontTopFade; uint8_t frontTopBri; uint8_t frontTopHue; uint8_t frontTopPalNum; uint8_t frontTopDesat;
  uint8_t frontBotDelay; uint8_t frontBotFade; uint8_t frontBotBri; uint8_t frontBotHue; uint8_t frontBotPalNum; uint8_t frontBotDesat;
  uint8_t rearDelay;     uint8_t rearFade;     uint8_t rearBri;     uint8_t rearHue;     uint8_t rearPalNum;     uint8_t rearDesat;
  // Scrolling text stuff
  unsigned long frontTopScrollSpeed; unsigned long frontBotScrollSpeed; unsigned long rearScrollSpeed;
  unsigned long frontTopScrollColor; unsigned long frontBotScrollColor; unsigned long rearScrollColor;
  uint8_t frontTopScrollLang; uint8_t frontBotScrollLang; uint8_t rearScrollLang;
  uint8_t rearScrollSlant;
  uint8_t internalBrightness;
  uint8_t statusLEDBrightness;
} Settings;

// We can use this to change where in EEPROM we write for the TEENSY.
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define EEPROM_base_addr = 0;
#endif

#if defined(__SAMD21G18A__)
  // the Reactor Zero doesn't have EEPROM, so we use the FlashStorage library to store settings persistently in flash memory
  #include <FlashStorage.h>
  FlashStorage(my_flash_store, Settings); // Reserve a portion of flash memory to store Settings
#endif 

// We need these regardless of the board type!
Settings activeSettings;   //create an working copy of the variable structure in SRAM
Settings tempSettings; //create a temporary variable structure in SRAM


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
bool uartRcvInProgress = false;


////
// Function Prototypes
////
void setStatusLED(uint8_t mode=0, unsigned long delay=250, uint8_t loops=2);

// Brightness control
// This is where we'll read from the pot, etc
uint8_t brightness() {
  //LED brightness is capped at 200 (out of 255) to reduce heat and extend life of LEDs. 
  if (useTempInternalBrightness[0]) return tempGlobalBrightnessValue[0];
  else if (internalBrightness[0]) return globalBrightnessValue[0];
  else return loopTrimpots[2];
}

// Firmware Routine to average the value received from the POT so that the external resistor isn't needed
// WARNING - DO NOT PUT DEBUG OUTPUT IN THIS FUNCTION, YOU WILL CRASH THE BOARDS!
void calcAveragePOT() {
  
  // Calculate the Rolling Sum
  for (int i=0; i<4; i++) {
    
    POTSum[i] -= POTReadings[i][POTIndex[i]];
    if (i == 0) POTReadings[0][POTIndex[i]] = map(analogRead(delayPin), 0, 1023, MIN_DELAY, MAX_DELAY);
    else if (i == 1) POTReadings[1][POTIndex[i]] = map(analogRead(fadePin), 0, 1023, 0, MAX_FADE);
    else if (i == 2) POTReadings[2][POTIndex[i]] = map(analogRead(briPin), 0, 1023, MIN_BRI, MAX_BRI);
    else if (i == 3) POTReadings[3][POTIndex[i]] = map(analogRead(huePin), 0, 1023, 0, MAX_PAL);
    POTSum[i] += POTReadings[i][POTIndex[i]];
  
    // Adjust the index so we maintain a circular buffer.
    POTIndex[i]++;
    POTIndex[i] = POTIndex[i] % POT_AVG_SIZE;
  }
  //return POTSum[potNum] / POT_AVG_SIZE;
}

uint8_t getAveragePOT(int potNum) {
  return POTSum[potNum] / POT_AVG_SIZE;
}


//TODO
void checkTrimpots(bool startTrim = 0) {
  
  //check the current trimpot values and put them into startTrimpots[] or loopTrimpots[]
  //DEBUG_PRINT("StartTrim ");DEBUG_PRINT_LN(startTrim);
  if (startTrim == 0) {
    loopTrimpots[0] = getAveragePOT(0); //map(analogRead(delayPin), 0, 1023, MIN_DELAY, MAX_DELAY);
    loopTrimpots[1] = getAveragePOT(1); //map(analogRead(fadePin), 0, 1023, 0, MAX_FADE);
    loopTrimpots[2] = getAveragePOT(2); //map(analogRead(briPin), 0, 1023, MIN_BRI, activeSettings.maxBri);
    loopTrimpots[3] = getAveragePOT(3); //map(analogRead(huePin), 0, 1023, 0, MAX_PAL);
  }
  else {
    startTrimpots[0] = getAveragePOT(0);//map(analogRead(delayPin), 0, 1023, MIN_DELAY, MAX_DELAY);
    startTrimpots[1] = getAveragePOT(1);//map(analogRead(fadePin), 0, 1023, 0, MAX_FADE);
    startTrimpots[2] = getAveragePOT(2);//map(analogRead(briPin), 0, 1023, MIN_BRI, activeSettings.maxBri);
    startTrimpots[3] = getAveragePOT(3);//map(analogRead(huePin), 0, 1023, 0, MAX_PAL);
  } 
  //DEBUG_PRINT("Brightness loop: "); DEBUG_PRINT_LN(loopTrimpots[2]);
  //DEBUG_PRINT("Brightness start: "); DEBUG_PRINT_LN(startTrimpots[2]);
  
}

//TODO
void compareTrimpots(byte adjMode = 0) {
  
  checkTrimpots();
  // Loop through each Pot ....
  for (byte x = 0; x < 4; x++) {
    //DEBUG_PRINT("Adjustment ");DEBUG_PRINT(x); DEBUG_PRINT(" start "); DEBUG_PRINT(startTrimpots[x]); DEBUG_PRINT(" loop "); DEBUG_PRINT(loopTrimpots[x]); 
    if (/* x > 1 && */adjEnabled[x] == 0 && ( startTrimpots[x] - loopTrimpots[x] > adjThreshold || loopTrimpots[x] - startTrimpots[x] > adjThreshold )  ) { //compare Brightness and Hue using adjThreshold, as changes there can be a lot of work
      adjEnabled[x] = 1;
    }
    else if ( adjEnabled[x] == 0 && startTrimpots[x] != loopTrimpots[x] ) {
      adjEnabled[x] = 1;
      DEBUG_PRINT_LN(x);
      DEBUG_PRINT_LN("ENABLED");
    }
    else if ( adjEnabled[x] == 1) {
      //if (loopTrimpots[x] != startTrimpots[x]) {
      if ((x==1 && loopTrimpots[x] != startTrimpots[x]) || (loopTrimpots[x]-startTrimpots[x]>=2 || startTrimpots[x]-loopTrimpots[x]>=2)) {

        DEBUG_PRINT("POT Loop Start Value ");
        DEBUG_PRINT(x);
        DEBUG_PRINT(" = ");
        DEBUG_PRINT_LN(loopTrimpots[x]);
        DEBUG_PRINT("Adjustment Mode ");DEBUG_PRINT_LN(adjMode);

        //adjustment is enabled for this pot, if settings have changed see if we need to recalc colors and all that jazz
        if (adjMode == 1) {
            //FRONT ADJUSTMENTS...
            if (x == 0) {
              DEBUG_PRINT_LN("Front Delay Changed");
              blinky_updates_per_sec[0] = activeSettings.frontTopDelay = map(loopTrimpots[x], 0, 1023, 0, 200);
              blinky_updates_per_sec[1] = activeSettings.frontBotDelay = map(loopTrimpots[x], 0, 1023, 0, 200);
            }
            else if (x == 1) 
            {
              DEBUG_PRINT_LN("Front Fade Changed");
              percentage_change_chance = activeSettings.frontTopFade = activeSettings.frontBotFade = map(loopTrimpots[x], 0, 1023, 0, 100);
            }
            else if (x == 2) {
              DEBUG_PRINT_LN("Front Brightness Changed");
              activeSettings.frontTopBri = activeSettings.frontBotBri = map(loopTrimpots[x], 0, 1023, 0, MAX_BRI); //if loopTrimpots were int's
            }
            else if (x == 3) {
              DEBUG_PRINT_LN("Front Palette Changed");
              activeSettings.frontTopPalNum = activeSettings.frontBotPalNum = map(loopTrimpots[x], 0, 1023, 0, MAX_PAL);
              frontTopTargetPalette = paletteArray[currentPalette[0]][activeSettings.frontTopPalNum];
              frontBotTargetPalette = paletteArray[currentPalette[1]][activeSettings.frontBotPalNum];
            }
        }
        if (adjMode == 3) {
            if (x == 0) {
              DEBUG_PRINT_LN("Rear Delay Changed");
              blinky_updates_per_sec[2] = activeSettings.rearDelay = map(loopTrimpots[x], 0, 1023, 0, 200);
            }
            else if (x == 1) {
              DEBUG_PRINT_LN("Rear Fade Changed");
              percentage_change_chance = activeSettings.rearFade = map(loopTrimpots[x], 0, 1023, 0, 100);
            }
            else if (x == 2) {
              DEBUG_PRINT_LN("Rear Brightness Changed");
              activeSettings.rearBri = map(loopTrimpots[x], 0, 1023, 0, MAX_BRI); //if loopTrimpots were int's
            }
            else if (x == 3) {
              DEBUG_PRINT_LN("Rear Palette Changed");
              activeSettings.rearPalNum = map(loopTrimpots[x], 0, 1023, 0, MAX_PAL);
              rearTargetPalette = paletteArray[currentPalette[2]][activeSettings.rearPalNum];              
            }
        }
      }
      //save the values for the next loop
      startTrimpots[x] = loopTrimpots[x];
    }
  }
  
}

//TODO - This code from Paul seems unreliable.
// The state stracking isn't right such that if you set the switch to "front" then center, then front, 
// it doesn't go back into fast blinky mode.  Need to look at this more closely.
void checkAdjSwitch() {
  
  if (digitalRead(FADJ_PIN) == 0 && prevAdjMode != 1 && startAdjMode == 0) {
    adjMode = 1;
    checkTrimpots(1); //put initial trimpot values into startTrimpots[]

    DEBUG_PRINT_LN("adj Front");

    //adjMillis = millis();
    adjLoops=0;
    statusFlipFlopTime = fastBlink;
  }
  else if (digitalRead(RADJ_PIN) == 0 && prevAdjMode != 3 && startAdjMode == 0) {
    adjMode = 3;
    checkTrimpots(1); //put initial trimpot values into startTrimpots[]

    DEBUG_PRINT_LN("adj Rear");

    //adjMillis = millis();
    adjLoops=0;
    statusFlipFlopTime = fastBlink;
  }
  else if ( (prevAdjMode != 0 && digitalRead(RADJ_PIN) == 1 && digitalRead(FADJ_PIN) == 1 && startAdjMode == 0) || (adjLoops>adjLoopMax) ) {

      if (adjLoops>adjLoopMax) DEBUG_PRINT_LN("MAXED OUT"); 
      statusFlipFlopTime = slowBlink; 

      //if we were in previous adjMode for way too long, save settings here  SAVE STUFF HERE and go back to regular mode!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (adjLoops > adjLoopMin)  {

        /*
        DEBUG_PRINT_LN("save");
        DEBUG_PRINT_LN("FDelay ");
        DEBUG_PRINT_LN(mySettings.frontDelay);
        DEBUG_PRINT_LN("FFade ");
        DEBUG_PRINT_LN(mySettings.frontFade);
        DEBUG_PRINT_LN("FHue ");
        DEBUG_PRINT_LN(mySettings.frontHue);
        DEBUG_PRINT_LN("FBri ");
        DEBUG_PRINT_LN(mySettings.frontBri);
        DEBUG_PRINT_LN("FPal ");
        DEBUG_PRINT_LN(mySettings.frontPalNum);        

        mySettings.writes++;
        saveSettings();
        */
        if (adjLoops>adjLoopMax) {
          startAdjMode=adjMode;
          adjLoops=0;
        }
        adjMode = 0;
        for (byte x = 0; x < 4; x++) adjEnabled[x] = 0; //  Reset the adjustment mode, so no more adjustments are allowed.         
      }
  }
  else if (digitalRead(RADJ_PIN) == 1 && digitalRead(FADJ_PIN) == 1 && startAdjMode != 0) {
    //adjMode didn't start off centered, which could have messed us up.
    //now it is centered though, so let's get back to our normal state.
    startAdjMode = 0;
    statusFlipFlopTime = slowBlink; 
  }
  if (adjMode != prevAdjMode) {
    //statusBlink(2, 250, 1, 0, 2); //blink purple 2 times
    setStatusLED(1,250,2);
  }
  prevAdjMode = adjMode;
  
}

int checkPalButton() {
  
  if (digitalRead(PAL_PIN) == 0) {
    //button is held
    DEBUG_PRINT_LN("Palette Button Held");
    palPinLoops++;
    if (palPinStatus == 1 && prevPalPinStatus == 1) {
      //we just started holding the button
      palPinStatus = 0;
      palPinLoops=0;
    }
    return (0);
  }  
  else if (digitalRead(PAL_PIN) == 1 && palPinStatus == 0 && prevPalPinStatus == 0) {
  //else if (digitalRead(PAL_PIN) == 1 && prevPalPinStatus == 0) {
    //button has just been released
    DEBUG_PRINT_LN("Palette Button Released");
    palPinLoops++;
    palPinStatus = 1;

    // Check the Adjustment Switch to see if we're moving everything to the next Palette
    // Or just Front/Rear ...
    DEBUG_PRINT("Front Switch: "); DEBUG_PRINT_LN(digitalRead(FADJ_PIN));
    DEBUG_PRINT("Rear Switch: "); DEBUG_PRINT_LN(digitalRead(RADJ_PIN));

    // We change the palette when you release the button ...
    if (digitalRead(RADJ_PIN) == 1 && digitalRead(FADJ_PIN) == 1) {
      for (int i=0; i<3; i++) {
        currentPalette[i]++;
        if (currentPalette[i] == MAX_PAL) currentPalette[i] = 0;
      }
    } else if (digitalRead(FADJ_PIN) == 0) {
      // Advance the Front Palette only
      currentPalette[0]++;
      currentPalette[1]++;
      if (currentPalette[0] == MAX_PAL) currentPalette[0] = 0;
      if (currentPalette[1] == MAX_PAL) currentPalette[1] = 0;
    } else if (digitalRead(RADJ_PIN) == 0) {
      // Advance the Rear Palette only
      currentPalette[2]++;
      if (currentPalette[2] == MAX_PAL) currentPalette[2] = 0;
    }

    // Set the respective Target Palettes
    frontTopTargetPalette = paletteArray[currentPalette[0]][0];
    frontBotTargetPalette = paletteArray[currentPalette[1]][1];
    rearTargetPalette = paletteArray[currentPalette[2]][2];
    
    return (palPinLoops);
  }
  prevPalPinStatus = palPinStatus;
}


void saveSettings() {
  #if defined(__SAMD21G18A__)
    my_flash_store.write(activeSettings);
  #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    EEPROM.put(EEPROM_base_addr, activeSettings);
  #endif
}

void loadSettings(bool resetSettings=false) {

#if defined(__SAMD21G18A__)
  // Read the flash into a temp storage, so we can parse it, and set valid things.
  tempSettings = my_flash_store.read();
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  // Read the EEPROM into a temp storage so we can parse it ...
  tempSettings = EEPROM.read(EEPROM_base_addr);
#endif

  DEBUG_PRINT_LN("****************************************");
  DEBUG_PRINT("** Settings written ");DEBUG_PRINT(tempSettings.writes);DEBUG_PRINT_LN(" times so far **");
  DEBUG_PRINT_LN("****************************************");

  if (tempSettings.writes > 9000) DEBUG_PRINT_LN("WARNING::  FLASH NEARING END OF LIFE.");
  else if (tempSettings.writes > 5000) DEBUG_PRINT_LN("WARNING::  FLASH NEAR 50% OF LIFE.");

  // So let's see if this is the first time since we uploaded the sketch
  if ((tempSettings.writes == 0) || (resetSettings))
  {
    DEBUG_PRINT_LN("Using Default Settings");
    // First time ... just load a set of defaults.  
    // We don't auto store these back to the flash since that would waste a write
    // If the user never changes anything, why bother!

    // Also this can be used to restore the whole panel back to default.
    // To do that pass in the reset parameter.

    activeSettings.maxBri = MAX_BRI;
    activeSettings.frontTopDelay = blinky_updates_per_sec[0] = 50;
    activeSettings.frontTopFade = percentage_change_chance = 60;
    // Here we load the internal brightness value, but will only use that if the internal brightness
    // control is set.  Otherwise, the POT will be used.
    activeSettings.frontTopBri = internalBrightness[0] = 10; // Initially set pretty low.  // Only this one is used!
    activeSettings.frontTopHue = 0; // Not Used.
    activeSettings.frontTopPalNum = currentPalette[0] = 0;
    activeSettings.frontTopDesat = 0; // Not used.
    
    activeSettings.frontBotDelay = blinky_updates_per_sec[1] = 50;
    activeSettings.frontBotFade = percentage_change_chance = 60;
    activeSettings.frontBotBri = internalBrightness[1] = 10; // Initially set pretty low.
    activeSettings.frontBotHue = 0; // Not Used.
    activeSettings.frontBotPalNum = currentPalette[1] = 0;
    activeSettings.frontBotDesat = 0; // Not used.
    
    activeSettings.rearDelay = blinky_updates_per_sec[2] = 50;
    activeSettings.rearFade = percentage_change_chance = 60;
    activeSettings.rearBri = internalBrightness[2] = 10; // Initially set pretty low.
    activeSettings.rearHue = 0; // Not Used.
    activeSettings.rearPalNum = currentPalette[2] = 0;
    activeSettings.rearDesat = 0; // Not used.
    
    activeSettings.frontTopScrollSpeed = scrollDelay[0] = 75;
    activeSettings.frontBotScrollSpeed = scrollDelay[1] = 75;
    activeSettings.rearScrollSpeed = scrollDelay[2] = 75;
    activeSettings.frontTopScrollColor = fontColor[0] = 0x0000ff; // Blue
    activeSettings.frontBotScrollColor = fontColor[1] = 0x0000ff; // Blue
    activeSettings.rearScrollColor = fontColor[2] = 0x00ff00; // Green
    activeSettings.frontTopScrollLang = alphabetType[0] = LATIN;
    activeSettings.frontBotScrollLang = alphabetType[1] = LATIN;
    activeSettings.rearScrollLang = alphabetType[2] = LATIN;
    activeSettings.rearScrollSlant = 1; // Right Lean
    
    activeSettings.internalBrightness = internalBrightness[0] = internalBrightness[1] = internalBrightness[2] = false;
    activeSettings.statusLEDBrightness = STATUS_BRIGHTNESS = 25;  // If zero the LED is off.

    // Set the Palettes
    frontTopTargetPalette = paletteArray[currentPalette[0]][0];
    frontBotTargetPalette = paletteArray[currentPalette[1]][1];
    rearTargetPalette = paletteArray[currentPalette[2]][2];
  }
  else
  {
    DEBUG_PRINT_LN("Using Stored Settings");
    // We have some valid settings that were saved, load whatever the user set...

    // Track the number of writes (since we last updated FW...)
    activeSettings.writes = tempSettings.writes;
    
    activeSettings.maxBri = tempSettings.maxBri;

    // TOP FLD Settings
    blinky_updates_per_sec[0] = activeSettings.frontTopDelay = tempSettings.frontTopDelay;
    percentage_change_chance = activeSettings.frontTopFade = tempSettings.frontTopFade;
    // Here we load the internal brightness value, but will only use that if the internal brightness
    // control is set.  Otherwise, the POT will be used.
    internalBrightness[0] = activeSettings.frontTopBri = tempSettings.frontTopBri; // Only this one is used currently!
    activeSettings.frontTopHue = tempSettings.frontTopHue; // Not Used.
    currentPalette[0] = activeSettings.frontTopPalNum = tempSettings.frontTopPalNum;
    activeSettings.frontTopDesat = tempSettings.frontTopDesat; // Not used.

    // Bottom FLD Settings
    blinky_updates_per_sec[1] = activeSettings.frontBotDelay = tempSettings.frontBotDelay;
    percentage_change_chance = activeSettings.frontBotFade = tempSettings.frontBotFade;
    internalBrightness[1] = activeSettings.frontBotBri = tempSettings.frontBotBri;
    activeSettings.frontBotHue = tempSettings.frontBotHue; // Not Used.
    currentPalette[1] = activeSettings.frontBotPalNum = tempSettings.frontBotPalNum;
    activeSettings.frontBotDesat = tempSettings.frontBotDesat; // Not used.

    // Setup the Rear Display Settings
    blinky_updates_per_sec[2] = activeSettings.rearDelay = tempSettings.rearDelay;
    percentage_change_chance = activeSettings.rearFade = tempSettings.rearFade;
    internalBrightness[2] = activeSettings.rearBri = tempSettings.rearBri;
    activeSettings.rearHue = tempSettings.rearHue; // Not Used.
    currentPalette[2] = activeSettings.rearPalNum = tempSettings.rearPalNum;
    activeSettings.rearDesat = tempSettings.rearDesat; // Not used.
    
    scrollDelay[0] = activeSettings.frontTopScrollSpeed = tempSettings.frontTopScrollSpeed;
    scrollDelay[1] = activeSettings.frontBotScrollSpeed = tempSettings.frontBotScrollSpeed;
    scrollDelay[2] = activeSettings.rearScrollSpeed = tempSettings.rearScrollSpeed;
    fontColor[0] = activeSettings.frontTopScrollColor = tempSettings.frontTopScrollColor;
    fontColor[1] = activeSettings.frontBotScrollColor = tempSettings.frontBotScrollColor;
    fontColor[2] = activeSettings.rearScrollColor = tempSettings.rearScrollColor;
    alphabetType[0] = activeSettings.frontTopScrollLang = tempSettings.frontTopScrollLang;
    alphabetType[1] = activeSettings.frontBotScrollLang = tempSettings.frontBotScrollLang;
    alphabetType[2] = activeSettings.rearScrollLang = tempSettings.rearScrollLang;
    activeSettings.rearScrollSlant = tempSettings.rearScrollSlant;
    
    internalBrightness[0] = internalBrightness[1] = internalBrightness[2] = activeSettings.internalBrightness = tempSettings.internalBrightness;
    STATUS_BRIGHTNESS = activeSettings.statusLEDBrightness = tempSettings.statusLEDBrightness;

    // Set the Palettes
    frontTopTargetPalette = paletteArray[currentPalette[0]][0];
    frontBotTargetPalette = paletteArray[currentPalette[1]][1];
    rearTargetPalette = paletteArray[currentPalette[2]][2];
  }
}


/*
 * 
 *  
    activeSettings.writes
    activeSettings.maxBri = 
    activeSettings.frontTopDelay
    activeSettings.frontTopFade
    activeSettings.frontTopBri
    activeSettings.frontTopHue
    activeSettings.frontTopPalNum
    activeSettings.frontTopDesat
    
    activeSettings.frontBotDelay
    activeSettings.frontBotFade
    activeSettings.frontBotBri
    activeSettings.frontBotHue
    activeSettings.frontBotPalNum
    activeSettings.frontBotDesat
    
    activeSettings.rearDelay
    activeSettings.rearFade
    activeSettings.rearBri
    activeSettings.rearHue
    activeSettings.rearPalNum
    activeSettings.rearDesat
    
    activeSettings.frontTopScrollSpeed
    activeSettings.frontBotScrollSpeed
    activeSettings.rearScrollSpeed
    activeSettings.frontTopScrollColor
    activeSettings.frontBotScrollColor
    activeSettings.rearScrollColor
    activeSettings.frontTopScrollLang
    activeSettings.frontBotScrollLang
    activeSettings.rearScrollLang
    activeSettings.rearScrollSlant
    
    activeSettings.internalBrightness
    activeSettings.statusLEDBrightness

*/
