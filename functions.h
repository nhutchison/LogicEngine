
//TODO
void checkTrimpots(bool startTrim = 0) {
  /*
  //check the current trimpot values and put them into startTrimpots[] or loopTrimpots[]
  if (startTrim == 0) {
    loopTrimpots[0] = map(analogRead(delayPin), 0, 1023, MIN_DELAY, MAX_DELAY);
    loopTrimpots[1] = map(analogRead(fadePin), 0, 1023, 0, MAX_FADE);
    loopTrimpots[2] = map(analogRead(briPin), 0, 1023, MIN_BRI, mySettings.maxBri);
    loopTrimpots[3] = map(analogRead(huePin), 0, 1023, 0, 255);
  }
  else {
    startTrimpots[0] = map(analogRead(delayPin), 0, 1023, MIN_DELAY, MAX_DELAY);
    startTrimpots[1] = map(analogRead(fadePin), 0, 1023, 0, MAX_FADE);
    startTrimpots[2] = map(analogRead(briPin), 0, 1023, MIN_BRI, mySettings.maxBri);
    startTrimpots[3] = map(analogRead(huePin), 0, 1023, 0, 255);
  }
  */
}

//TODO
void compareTrimpots(byte adjMode = 0) {
  /*
  checkTrimpots();
  for (byte x = 0; x < 4; x++) {
    if ( x > 1 && adjEnabled[x] == 0 && ( startTrimpots[x] - loopTrimpots[x] > adjThreshold || loopTrimpots[x] - startTrimpots[x] > adjThreshold )  ) { //compare Brightness and Hue using adjThreshold, as changes there can be a lot of work
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

        DEBUG_PRINT_LN(x);
        DEBUG_PRINT_LN("=");
        DEBUG_PRINT_LN(loopTrimpots[x]);

        //adjustment is enabled for this pot, if settings have changed see if we need to recalc colors and all that jazz
        if (adjMode == 1) {
            //FRONT ADJUSTMENTS...
            if (x == 0) mySettings.frontDelay = loopTrimpots[x];
            else if (x == 1) mySettings.frontFade = loopTrimpots[x];
            else if (x == 2) {
              //map(hsvColor[2],0,255,0,byte(float(mySettings.maxBri)/255*mySettings.frontBri) )
              //mySettings.frontBri = map(loopTrimpots[x], 0, 1023, 0, 255); //if loopTrimpots were int's
              mySettings.frontBri = loopTrimpots[x];
              calcColors(mySettings.frontPalNum, 0);
            }
            else if (x == 3) {
              //mySettings.frontHue = map(loopTrimpots[x], 0, 1023, 0, 255); //if loopTrimpots were int's
              mySettings.frontHue = loopTrimpots[x];
              calcColors(mySettings.frontPalNum, 0);
            }
        }
        if (adjMode == 3) {
            if (x == 0) mySettings.rearDelay = loopTrimpots[x];
            else if (x == 1) mySettings.rearFade = loopTrimpots[x];
            else if (x == 2) {
              //map(hsvColor[2],0,255,0,byte(float(mySettings.maxBri)/255*mySettings.frontBri) )
              //mySettings.frontBri = map(loopTrimpots[x], 0, 1023, 0, 255); //if loopTrimpots were int's
              mySettings.rearBri = loopTrimpots[x];
              calcColors(mySettings.rearPalNum, 1);
            }
            else if (x == 3) {
              //mySettings.frontHue = map(loopTrimpots[x], 0, 1023, 0, 255); //if loopTrimpots were int's
              mySettings.rearHue = loopTrimpots[x];
              calcColors(mySettings.rearPalNum, 1);
            }
        }
      }
      //save the values for the next loop
      startTrimpots[x] = loopTrimpots[x];
    }
  }
  */
}

// TODO
void saveSettings() {
  //my_flash_store.write(mySettings); //////////////////////////////////////////////////////////////// TODO: make compatible with standard Arduino EEPROM
}

//TODO
void checkAdjSwitch() {
  /*
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

      //if we were in previous adjMode for way too long, save settings here  SAVE STUFF HERE and go back to regular mode!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (adjLoops > adjLoopMin)  {

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
        if (adjLoops>adjLoopMax) {
          startAdjMode=adjMode;
          adjLoops=0;
        }
        adjMode = 0;
        for (byte x = 0; x < 4; x++) adjEnabled[x] = 0; //
        statusFlipFlopTime = slowBlink;            
      }
  }
  else if (digitalRead(RADJ_PIN) == 1 && digitalRead(FADJ_PIN) == 1 && startAdjMode != 0) {
    //adjMode didn't start off centered, which could have messed us up.
    //now it is centered though, so let's get back to our normal state.
    startAdjMode = 0;
  }
  if (adjMode != prevAdjMode) {
    statusBlink(2, 250, 1, 0, 2); //blink purple 2 times
  }
  prevAdjMode = adjMode;
  */
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
    frontTargetPalette = paletteArray[currentPalette[0]][0];
    rearTargetPalette = paletteArray[currentPalette[2]][2];
    
    return (palPinLoops);
  }
  prevPalPinStatus = palPinStatus;
}

// TODO - Load previous settings
void loadSettings(byte deviceNum = 0) {
  /*
  //load stored settings for specified logic display(s) (0 for all, 1 for front, 3 for rear)
  if (deviceNum==0) {
    mySettings = my_flash_store.read();                  //////////////////////////////////////////////////////////////// TODO: make compatible with standard Arduino EEPROM
  }
  else {
    tempSettings = my_flash_store.read();
  }
  if (deviceNum==1) {
    //just read front specific settings from stored values
    mySettings.frontDelay=tempSettings.frontDelay;
    mySettings.frontFade=tempSettings.frontFade;
    mySettings.frontBri=tempSettings.frontBri;
    mySettings.frontHue=tempSettings.frontHue;
    mySettings.frontPalNum=tempSettings.frontPalNum;
    mySettings.frontDesat=tempSettings.frontDesat;
    mySettings.frontScrollSpeed=tempSettings.frontScrollSpeed;
  }
  else if (deviceNum==3) {
    //just read rear specific settings from stored values
    mySettings.rearDelay=tempSettings.rearDelay;
    mySettings.rearFade=tempSettings.rearFade;
    mySettings.rearBri=tempSettings.rearBri;
    mySettings.rearHue=tempSettings.rearHue;
    mySettings.rearPalNum=tempSettings.rearPalNum;
    mySettings.rearDesat=tempSettings.rearDesat;
    mySettings.rearScrollSpeed=tempSettings.rearScrollSpeed;
  }
  if (deviceNum==0||deviceNum==1) calcColors(mySettings.frontPalNum, 0);
  if (deviceNum==0||deviceNum==3) calcColors(mySettings.rearPalNum, 1);  
  */
}

//TODO - Reset to defaults (caled on D command!)
void factorySettings() {
  /*
  mySettings = { (mySettings.writes+1), MAX_BRIGHTNESS,
                   DFLT_FRONT_DELAY, DFLT_FRONT_FADE, DFLT_FRONT_BRI, DFLT_FRONT_HUE, DFLT_FRONT_PAL, DFLT_FRONT_DESAT,
                   DFLT_REAR_DELAY,  DFLT_REAR_FADE,  DFLT_REAR_BRI,  DFLT_REAR_HUE,  DFLT_REAR_PAL, DFLT_REAR_DESAT,
                   DFLT_FRONT_SCROLL,DFLT_REAR_SCROLL
                 };
  calcColors(mySettings.frontPalNum, 0);
  calcColors(mySettings.rearPalNum, 1);
  */                
}

//TODO - Ad Pallettes, and set here...
void changePalNum(byte logicAddress, byte palNum=0) {
  /*
    if (logicAddress == 1)   {
      if (palNum==0) {
        //new pal num wasnt specified, so cycle pals up
        mySettings.frontPalNum++;
        if (mySettings.frontPalNum == NUM_PALS) mySettings.frontPalNum = 0;
      }
      else mySettings.frontPalNum=palNum-1;
      //generate new front palette here!!!
      calcColors(mySettings.frontPalNum, 0);
      
      DEBUG_PRINT_LN("pal");
      DEBUG_PRINT_LN(mySettings.frontPalNum);

    }
    else if (logicAddress == 3) {
      if (palNum==0) {
        mySettings.rearPalNum++;
        if (mySettings.rearPalNum == NUM_PALS) mySettings.rearPalNum = 0;
      }
      else mySettings.rearPalNum=palNum-1;
      //generate new rear palette here!!!
      calcColors(mySettings.rearPalNum, 1);
          
      DEBUG_PRINT_LN("pal");
      DEBUG_PRINT_LN(mySettings.rearPalNum);
  
    }
  */
}
