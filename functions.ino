void handleLightSensor(myConfig config, uint32_t t){
  
  //amw18 need to figure out a min/max for lastLightValue R/L
  //use the min to make sure pupil size doesn't just keep getting smaller
  //use the max to make sure we can darken the light sensor and get a response

  //updates lastLightValue L/R according to whichever sensor/state each eye is in
  uint32_t val2 = analogRead(A2); //read the D2 JST light sensor - left eye
  uint32_t val3 = analogRead(A3); //read the D3 JST light sensor - right eye
  uint32_t val = (val3 > val2) ? val3 : val2; //set val to largest of either sensor
  lastLightReadTime = t;
  lightSensorFailCount = 0;
  //handle the left eye value
    switch(config.lLight){
      case BOTH:{
        //use val (largest of either sensor) to calculate
        float v = (float)val/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueL    = irisMinL + v * irisRange; //amw18 changed to irisMinL from irisMin....works?
        break;
      }
      case NONE:{
      //  lightSensorPin = 0;
        lastLightValueL = irisMinL;
        break;
      }
      case RIGHT:{
        //use val3 sensor
        //lightSensorPin = 102;
        float v = (float)val3/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueL    = irisMinL + v * irisRange; //amw18 changed to irisMinL works?
        break;
      }
      case LEFT:{
        //use val2 sensor
        //lightSensorPin = 102;
        float v = (float)val2/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueL    = irisMinL + v * irisRange;
        break;
      }
    } //amw18 end switch for left eye

//handle the right eye value
    switch(config.rLight){
      case BOTH:{
        //use val (largest of either sensor) to calculate
        //lightSensorPin = 102;
        float v = (float)val/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueR    = irisMinR + v * irisRange;
        break;
      }
      case NONE:{
        //lightSensorPin = 0;
        lastLightValueR = irisMinR;
        break;
      }
      case RIGHT:{
        //use the val3 sensor value
        //lightSensorPin = 102;
        float v = (float)val3/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueR    = irisMinR + v * irisRange;
        break;
      }
      case LEFT:{
        //use the val2 sensor value
        //lightSensorPin = 102;
        float v = (float)val2/1000; // 0.0 to 1.0 based on sensor output of 1-1000(ish)
        v = pow(v, lightSensorCurve);
        lastLightValueR    = irisMinR + v * irisRange;
        break;
      }
    } //amw18 end switch for left eye

/*// amw18 original code for the eye/light response:
// Fun fact: eyes have a "consensual response" to light -- both
          // pupils will react even if the opposite eye is stimulated.
          // Meaning we can get away with using a single light sensor for
          // both eyes. This comment has nothing to do with the code.
          uint16_t rawReading = arcada.readLightSensor();
          if(rawReading <= 1023) {
            if(rawReading < lightSensorMin)      rawReading = lightSensorMin; // Clamp light sensor range
            else if(rawReading > lightSensorMax) rawReading = lightSensorMax; // to within usable range
            float v = (float)(rawReading - lightSensorMin) / (float)(lightSensorMax - lightSensorMin); // 0.0 to 1.0
            v = pow(v, lightSensorCurve);
            lastLightValue    = irisMin + v * irisRange;
            lastLightReadTime = t;
            lightSensorFailCount = 0;
          } else { // I2C error
            if(++lightSensorFailCount >= 25) { // If repeated errors in succession...
              lightSensorPin = -1; // Stop trying to use the light sensor
            } else {
              lastLightReadTime = t - LIGHT_INTERVAL + 30000; // Try again in 30 ms
            } }
            break;
  */ //end case ORIG
} //end handleLightSensor

static uint16_t readWaveData(uint8_t *dst) {
  if(remainingBytesInChunk <= 0) {
    // Read next chunk
    struct {
      char     id[4];
      uint32_t size;
    } header;
    for(;;) {
      if(wavFile.read(&header, 8) != 8) return 0;
      if(!strncmp(header.id, "data", 4)) {
        remainingBytesInChunk = header.size;
        break;
      }
      if(!wavFile.seekCur(header.size)) { // If not "data" then skip
        return 0; // Seek failed, return invalid count
      }
    }
  }

  int16_t bytesRead = wavFile.read(dst, min(WAV_BUFFER_SIZE, remainingBytesInChunk));
  if(bytesRead > 0) remainingBytesInChunk -= bytesRead;
  return bytesRead;
}

// Partially swiped from Wave Shield code.
// Is pared-down, handles 8-bit mono only to keep it simple.
static bool startWav(char *filename) {
  wavFile = arcada.open(filename);
  if(!wavFile) {
    Serial.println("Failed to open WAV file");
    return false;
  }

  union {
    struct {
      char     id[4];
      uint32_t size;
      char     data[4];
    } riff;  // riff chunk
    struct {
      uint16_t compress;
      uint16_t channels;
      uint32_t sampleRate;
      uint32_t bytesPerSecond;
      uint16_t blockAlign;
      uint16_t bitsPerSample;
      uint16_t extraBytes;
    } fmt; // fmt data
  } buf;

  uint16_t size;
  if((wavFile.read(&buf, 12) == 12)
    && !strncmp(buf.riff.id, "RIFF", 4)
    && !strncmp(buf.riff.data, "WAVE", 4)) {
    // next chunk must be fmt, fmt chunk size must be 16 or 18
    if((wavFile.read(&buf, 8) == 8)
      && !strncmp(buf.riff.id, "fmt ", 4)
      && (((size = buf.riff.size) == 16) || (size == 18))
      && (wavFile.read(&buf, size) == size)
      && ((size != 18) || (buf.fmt.extraBytes == 0))) {
      if((buf.fmt.channels == 1) && (buf.fmt.bitsPerSample == 8)) {
        Serial.printf("Samples/sec: %d\n", buf.fmt.sampleRate);
        bufEnd = readWaveData(wavBuf[0]);
        if(bufEnd > 0) {
          // Initialize A/D, speaker and start timer
          analogWriteResolution(8);
          analogWrite(A0, 128);
          analogWrite(A1, 128);
          arcada.enableSpeaker(true);
          wavEventTime = millis(); // WAV starting time
          bufIdx       = 0;
          playing      = true;
          arcada.timerCallback(buf.fmt.sampleRate, wavOutCallback);
          nextBufEnd   = readWaveData(wavBuf[1]);
        }
        return true;
      } else {
        Serial.println("Only 8-bit mono WAVs are supported");
      }
    } else {
      Serial.println("WAV uses compression or other unrecognized setting");
    }
  } else {
    Serial.println("Not WAV file");
  }

  wavFile.close();
  return false;
}

static void wavOutCallback(void) {
  uint8_t n = wavBuf[activeBuf][bufIdx];
  analogWrite(A0, n);
  analogWrite(A1, n);

  if(++bufIdx >= bufEnd) {
    if(nextBufEnd <= 0) {
      arcada.timerStop();
      arcada.enableSpeaker(false);
      playing      = false;
      wavEventTime = millis(); // Same var now holds WAV end time
      return;
    }
    bufIdx     = 0;
    bufEnd     = nextBufEnd;
    nextBufEnd = readWaveData(wavBuf[activeBuf]);
    activeBuf  = 1 - activeBuf;
  }
}

/*
void getWavFiles(){
  File            entry;
  //struct wavlist *wptr;
  char            filename[SD_MAX_FILENAME_SIZE+1];
  // Scan wav_path for .wav files:
  for(int i=0; i<MAX_WAV_FILES; i++) {
    entry = arcada.openFileByIndex(wav_path, i, O_READ, "wav");
    if(!entry) break;
    // Found one, alloc new wavlist struct, try duplicating filename
    if((wptr = (struct wavlist *)malloc(sizeof(struct wavlist)))) {
      entry.getName(filename, SD_MAX_FILENAME_SIZE);
      if((wptr->filename = strdup(filename))) {
        // Alloc'd OK, add to linked list...
        if(wavListPtr) {           // List already started?
          wavListPtr->next = wptr; // Point prior last item to new one
        } else {
          wavListStart = wptr;     // Point list head to new item
        }
        wavListPtr = wptr;         // Update last item to new one
      } else {
        free(wptr);                // Alloc failed, delete interim stuff
      }
    }
    entry.close();
  }
  if(wavListPtr) {                   // Any items in WAV list?
    wavListPtr->next = wavListStart; // Point last item's next to list head (list is looped)
    wavListPtr       = wavListStart; // Update list pointer to head
  }
}*/ //don't think we need this bc not looping sounds etc.

void splashScreen(){
  // SPLASH SCREEN (IF FILE PRESENT) ----amw18 update this to Sim Lab image!---------------------------------
  yield();
  if (showSplashScreen) {
    showSplashScreen = ((arcada.drawBMP((char *)"/splash.bmp",
                         0, 0, eye[0].display)) == IMAGE_SUCCESS);
    if (showSplashScreen) { // Loaded OK?
      Serial.println("Splashing");
      yield();
      arcada.drawBMP((char *)"/splash.bmp", 0, 0, eye[1].display);
      // Ramp up backlight over 1/2 sec duration
      startTime = millis();
      while ((elapsed = (millis() - startTime)) <= 500) {
        yield();
        arcada.setBacklight(255 * elapsed / 500);
      }
      arcada.setBacklight(255); // To the max
      startTime = millis();     // Note current time for backlight hold later
    }
  }
  // If no splash, or load failed, turn backlight on early so user gets a
  // little feedback, that the board is not locked up, just thinking.
  if (!showSplashScreen) arcada.setBacklight(255);
}

void loadEyelidGraphics(){
  yield();
  ImageReturnCode status;

  status = loadEyelid(upperEyelidFilename ?
    upperEyelidFilename : (char *)"upper.bmp",
    upperClosed, upperOpen, DISPLAY_SIZE-1, maxRam);

  status = loadEyelid(lowerEyelidFilename ?
    lowerEyelidFilename : (char *)"lower.bmp",
    lowerOpen, lowerClosed, 0, maxRam);

  // Filenames are no longer needed...
  for(int e=0; e<NUM_EYES; e++) {
    if(eye[e].sclera.filename) free(eye[e].sclera.filename);
    if(eye[e].iris.filename)   free(eye[e].iris.filename);
  }
  if(lowerEyelidFilename) free(lowerEyelidFilename);
  if(upperEyelidFilename) free(upperEyelidFilename);

  // Note that calls to availableRAM() at this point will return something
  // close to reserveSpace, suggesting very little RAM...but that function
  // really just returns the space between the heap and stack, and we've
  // established above that the top of the heap is something of a mirage.
  // Large allocations CAN still take place in the lower heap!

  calcMap();
  calcDisplacement();
  Serial.printf("Free RAM: %d\n", availableRAM());

  randomSeed(SysTick->VAL + analogRead(A2));
  eyeOldX = eyeNewX = eyeOldY = eyeNewY = mapRadius; // Start in center
  for(int e=0; e<NUM_EYES; e++) { // For each eye...
    eye[e].display->setRotation(eye[e].rotation);
    eye[e].eyeX = eyeOldX; // Set up initial position
    eye[e].eyeY = eyeOldY;
  }

  if (showSplashScreen) { // Image(s) loaded above?
    // Hold backlight on for up to 2 seconds (minus other initialization time)
    if ((elapsed = (millis() - startTime)) < 2000) {
      delay(2000 - elapsed);
    }
    // Ramp down backlight over 1/2 sec duration
    startTime = millis();
    while ((elapsed = (millis() - startTime)) <= 500) {
      yield();
      arcada.setBacklight(255 - (255 * elapsed / 500));
    }
    arcada.setBacklight(0);
    for(int e=0; e<NUM_EYES; e++) {
      eye[e].display->fillScreen(0);
    }
  }
/* amw18 comment out the voice modulation stuff
#if defined(ADAFRUIT_MONSTER_M4SK_EXPRESS)
  if(voiceOn) {
    if(!voiceSetup((waveform > 0))) {
      Serial.println("Voice init fail, continuing without");
      voiceOn = false;
    } else {
      voiceGain(gain);
      currentPitch = voicePitch(currentPitch);
      if(waveform) voiceMod(modulate, waveform);
      arcada.enableSpeaker(true);
    }
  }
#endif
*/
  arcada.setBacklight(255); // Back on, impending graphics

  yield();
}

void loadTextureMaps(){
  maxRam = availableRAM() - stackReserve;
  uint8_t e2;
  for(int e=0; e<NUM_EYES; e++) { // For each eye...
    yield();
    for(e2=0; e2<e; e2++) {    // Compare against each prior eye...
      // If both eyes have the same iris filename...
      if((eye[e].iris.filename && eye[e2].iris.filename) &&
         (!strcmp(eye[e].iris.filename, eye[e2].iris.filename))) {
        // Then eye 'e' can share the iris graphics from 'e2'
        // rotate & mirror are kept distinct, just share image
        eye[e].iris.data   = eye[e2].iris.data;
        eye[e].iris.width  = eye[e2].iris.width;
        eye[e].iris.height = eye[e2].iris.height;
        break;
      }
    }
    if((!e) || (e2 >= e)) { // If first eye, or no match found...
      // If no iris filename was specified, or if file fails to load...
      if((eye[e].iris.filename == NULL) || (loadTexture(eye[e].iris.filename,
        &eye[e].iris.data, &eye[e].iris.width, &eye[e].iris.height,
        maxRam) != IMAGE_SUCCESS)) {
        // Point iris data at the color variable and set image size to 1px
        eye[e].iris.data  = &eye[e].iris.color;
        eye[e].iris.width = eye[e].iris.height = 1;
      }
      // Huh. The booster seat idea STILL doesn't always work right,
      // something leaking in upper memory. Keep shrinking down the
      // booster seat size a bit each time we load a texture. Feh.
      maxRam -= 20;
    }
    // Repeat for sclera...
    for(e2=0; e2<e; e2++) {    // Compare against each prior eye...
      // If both eyes have the same sclera filename...
      if((eye[e].sclera.filename && eye[e2].sclera.filename) &&
         (!strcmp(eye[e].sclera.filename, eye[e2].sclera.filename))) {
        // Then eye 'e' can share the sclera graphics from 'e2'
        // rotate & mirror are kept distinct, just share image
        eye[e].sclera.data   = eye[e2].sclera.data;
        eye[e].sclera.width  = eye[e2].sclera.width;
        eye[e].sclera.height = eye[e2].sclera.height;
        break;
      }
    }
    if((!e) || (e2 >= e)) { // If first eye, or no match found...
      // If no sclera filename was specified, or if file fails to load...
      if((eye[e].sclera.filename == NULL) || (loadTexture(eye[e].sclera.filename,
        &eye[e].sclera.data, &eye[e].sclera.width, &eye[e].sclera.height,
        maxRam) != IMAGE_SUCCESS)) {
        // Point sclera data at the color variable and set image size to 1px
        eye[e].sclera.data  = &eye[e].sclera.color;
        eye[e].sclera.width = eye[e].sclera.height = 1;
      }
      maxRam -= 20; // See note above
    }
  }
}

void initializeDMAS(){
  // Initialize DMAs
  yield();
  for(uint8_t e=0; e<NUM_EYES; e++) {
  #if (ARCADA_TFT_WIDTH != 160) && (ARCADA_TFT_HEIGHT != 128) // 160x128 is ST7735 which isn't able to deal
    eye[e].spi->setClockSource(DISPLAY_CLKSRC); // Accelerate SPI!
  #endif
    eye[e].display->fillScreen(0);
    eye[e].dma.allocate();
    eye[e].dma.setTrigger(eye[e].spi->getDMAC_ID_TX());
    eye[e].dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    eye[e].dptr = eye[e].dma.addDescriptor(NULL, NULL, 42, DMA_BEAT_SIZE_BYTE, false, false);
    eye[e].dma.setCallback(dma_callback);
    eye[e].dma.setPriority(DMA_PRIORITY_0);
    uint32_t spi_data_reg = (uint32_t)eye[e].spi->getDataRegister();
    for(int i=0; i<2; i++) {   // For each of 2 scanlines...
      for(int j=0; j<NUM_DESCRIPTORS; j++) { // For each descriptor on scanline...
        eye[e].column[i].descriptor[j].BTCTRL.bit.VALID    = true;
        eye[e].column[i].descriptor[j].BTCTRL.bit.EVOSEL   = DMA_EVENT_OUTPUT_DISABLE;
        eye[e].column[i].descriptor[j].BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
        eye[e].column[i].descriptor[j].BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
        eye[e].column[i].descriptor[j].BTCTRL.bit.DSTINC   = 0;
        eye[e].column[i].descriptor[j].BTCTRL.bit.STEPSEL  = DMA_STEPSEL_SRC;
        eye[e].column[i].descriptor[j].BTCTRL.bit.STEPSIZE = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
        eye[e].column[i].descriptor[j].DSTADDR.reg         = spi_data_reg;
      }
    }
    eye[e].colNum       = DISPLAY_SIZE; // Force initial wraparound to first column
    eye[e].colIdx       = 0;
    eye[e].dma_busy     = false;
    eye[e].column_ready = false;
    eye[e].dmaStartTime = 0;
    // Default settings that can be overridden in config file
    eye[e].pupilColor        = 0x0000;
    eye[e].backColor         = 0xFFFF;
    eye[e].iris.color        = 0xFF01;
    eye[e].iris.data         = NULL;
    eye[e].iris.filename     = NULL;
    eye[e].iris.startAngle   = (e & 1) ? 512 : 0; // Rotate alternate eyes 180 degrees
    eye[e].iris.angle        = eye[e].iris.startAngle;
    eye[e].iris.mirror       = 0;
    eye[e].iris.spin         = 0.0;
    eye[e].iris.iSpin        = 0;
    eye[e].sclera.color      = 0xFFFF;
    eye[e].sclera.data       = NULL;
    eye[e].sclera.filename   = NULL;
    eye[e].sclera.startAngle = (e & 1) ? 512 : 0; // Rotate alternate eyes 180 degrees
    eye[e].sclera.angle      = eye[e].sclera.startAngle;
    eye[e].sclera.mirror     = 0;
    eye[e].sclera.spin       = 0.0;
    eye[e].sclera.iSpin      = 0;
    eye[e].rotation          = 3;

    // Uncanny eyes carryover stuff for now, all messy:
    eye[e].blink.state = NOBLINK;
    eye[e].blinkFactor = 0.0;
  } //end DMA initialize for both eyes
}

