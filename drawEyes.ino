void drawEyes(){   //amw18 pulled the drawing code out into it's own function

//this was the original draw function...for variable # eyes
  if(++eyeNum >= NUM_EYES) eyeNum = 0; // Cycle through eyes...

  uint8_t  x = eye[eyeNum].colNum;
  uint32_t t = micros();

  // If next column for this eye is not yet rendered...
  if(!eye[eyeNum].column_ready) {
    if(!x) { // If it's the first column...

      // ONCE-PER-FRAME EYE ANIMATION LOGIC HAPPENS HERE -------------------

      // Eye movement
      float eyeX, eyeY;
      if(moveEyesRandomly) {
        int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event
        if(eyeInMotion) {                       // Eye currently moving?
          if(dt >= eyeMoveDuration) {           // Time up?  Destination reached.
            eyeInMotion = false;                // Stop moving
            // The "move" duration temporarily becomes a hold duration...
            // Normally this is 35 ms to 1 sec, but don't exceed gazeMax setting
            uint32_t limit = min(1000000, gazeMax);
            eyeMoveDuration = random(35000, limit); // Time between microsaccades
            if(!saccadeInterval) {              // Cleared when "big" saccade finishes
              lastSaccadeStop = t;              // Time when saccade stopped
              saccadeInterval = random(eyeMoveDuration, gazeMax); // Next in 30ms to 3sec
            }
            // Similarly, the "move" start time becomes the "stop" starting time...
            eyeMoveStartTime = t;               // Save time of event
            eyeX = eyeOldX = eyeNewX;           // Save position
            eyeY = eyeOldY = eyeNewY;
          } else { // Move time's not yet fully elapsed -- interpolate position
            float e  = (float)dt / float(eyeMoveDuration); // 0.0 to 1.0 during move
            e = 3 * e * e - 2 * e * e * e; // Easing function: 3*e^2-2*e^3 0.0 to 1.0
            eyeX = eyeOldX + (eyeNewX - eyeOldX) * e; // Interp X
            eyeY = eyeOldY + (eyeNewY - eyeOldY) * e; // and Y
            Serial.println("eyeX, eyeOldX, eyeNewX");
            Serial.println(eyeX);
            Serial.println(eyeOldX);
            Serial.println(eyeNewX);
          }
        } 
        else {                       // Eye is currently stopped
          eyeX = eyeOldX;
          eyeY = eyeOldY;
          if(dt > eyeMoveDuration) {   // Time up?  Begin new move.
            if((t - lastSaccadeStop) > saccadeInterval) { // Time for a "big" saccade
              // r is the radius in X and Y that the eye can go, from (0,0) in the center.
              float r = ((float)mapDiameter - (float)DISPLAY_SIZE * M_PI_2) * 0.75;
              eyeNewX = random(-r, r);
              float h = sqrt(r * r - eyeNewX * eyeNewX);
              eyeNewY = random(-h, h);
              // Set the duration for this move, and start it going.
              eyeMoveDuration = random(83000, 166000); // ~1/12 - ~1/6 sec
              saccadeInterval = 0; // Calc next interval when this one stops
            } else { // Microsaccade
              // r is possible radius of motion, ~1/10 size of full saccade.
              // We don't bother with clipping because if it strays just a little,
              // that's okay, it'll get put in-bounds on next full saccade.
              float r = (float)mapDiameter - (float)DISPLAY_SIZE * M_PI_2;
              r *= 0.07;
              float dx = random(-r, r);
              eyeNewX = eyeX - mapRadius + dx;
              float h = sqrt(r * r - dx * dx);
              eyeNewY = eyeY - mapRadius + random(-h, h);
              eyeMoveDuration = random(7000, 25000); // 7-25 ms microsaccade
            }
            eyeNewX += mapRadius;    // Translate new point into map space
            eyeNewY += mapRadius;
            eyeMoveStartTime = t;    // Save initial time of move
            eyeInMotion      = true; // Start move on next frame
          }
        }
      } //end if eye moves randomly
      else { 
        // Allow user code to control eye position (e.g. IR sensor, joystick, etc.)
        float r = ((float)mapDiameter - (float)DISPLAY_SIZE * M_PI_2) * 0.9;
        eyeX = mapRadius + eyeTargetX * r;
        eyeY = mapRadius + eyeTargetY * r;
      }

      // Eyes fixate (are slightly crossed) -- amount is filtered for boops
      int nufix = booped ? 90 : 7;
      fixate = ((fixate * 15) + nufix) / 16;
      // save eye position to this eye's struct so it's same throughout render
      if(eyeNum & 1) eyeX += fixate; // Eyes converge slightly toward center
      else           eyeX -= fixate;
      eye[eyeNum].eyeX = eyeX;
      eye[eyeNum].eyeY = eyeY;

      if (eyeNum == 1) eye[eyeNum].pupilFactor = irisValueL; //if left eye, use left value 
      else eye[eyeNum].pupilFactor = irisValueR; //if right eye use right value
      
      // Also note - irisValue is calculated at the END of this function
      // for the next frame (because the sensor must be read when there's
      // no SPI traffic to the left eye)

      // Similar to the autonomous eye movement above -- blink start times
      // and durations are random (within ranges).
      if((t - timeOfLastBlink) >= timeToNextBlink) { // Start new blink?
        timeOfLastBlink = t;
        uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec
        //amw18 - blinkDuration is how long it takes to blink, not time in between blinks, haha
        // Set up durations for both eyes (if not already winking)
        for(uint8_t e=0; e<NUM_EYES; e++) {
          if(eye[e].blink.state == NOBLINK) {
            eye[e].blink.state     = ENBLINK;
            eye[e].blink.startTime = t;
            eye[e].blink.duration  = blinkDuration;
          }
        }
        //timeToNextBlink = blinkDuration * 3 + random(4000000); //amw18 this was the original blink line/timing
        if (config.Blink == NORMAL) timeToNextBlink = blinkDuration * 3 + random(4000000); //amw18 this is the original timing
        else if (config.Blink == SLOW) timeToNextBlink = random(8000000, 12000000); //amw18 this is a slow blink for the 'abnormal' eye states
      }

      float uq, lq; // So many sloppy temp vars in here for now, sorry
      if(tracking) {
        // Eyelids naturally "track" the pupils (move up or down automatically)
        int ix = (int)map2screen(mapRadius - eye[eyeNum].eyeX) + (DISPLAY_SIZE/2), // Pupil position
            iy = (int)map2screen(mapRadius - eye[eyeNum].eyeY) + (DISPLAY_SIZE/2); // on screen
        iy += irisRadius * trackFactor;
        if(eyeNum & 1) ix = DISPLAY_SIZE - 1 - ix; // Flip for right eye
        if(iy > upperOpen[ix]) {
          uq = 1.0;
        } else if(iy < upperClosed[ix]) {
          uq = 0.0;
        } else {
          uq = (float)(iy - upperClosed[ix]) / (float)(upperOpen[ix] - upperClosed[ix]);
        }
        if(booped) {
          uq = 0.9;
          lq = 0.7;
        } else {
          lq = 1.0 - uq;
        }
      } else {
        // If no tracking, eye is FULLY OPEN when not blinking
        uq = 1.0;
        lq = 1.0;
      }
      // Dampen eyelid movements slightly
      // SAVE upper & lower lid factors per eye,
      // they need to stay consistent across frame
      eye[eyeNum].upperLidFactor = (eye[eyeNum].upperLidFactor * 0.6) + (uq * 0.4);
      eye[eyeNum].lowerLidFactor = (eye[eyeNum].lowerLidFactor * 0.6) + (lq * 0.4);

      // Process blinks
      if(eye[eyeNum].blink.state) { // Eye currently blinking?
        // Check if current blink state time has elapsed
        if((t - eye[eyeNum].blink.startTime) >= eye[eyeNum].blink.duration) {
          if(++eye[eyeNum].blink.state > DEBLINK) { // Deblinking finished?
            eye[eyeNum].blink.state = NOBLINK;      // No longer blinking
            eye[eyeNum].blinkFactor = 0.0;
          } else { // Advancing from ENBLINK to DEBLINK mode
            eye[eyeNum].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
            eye[eyeNum].blink.startTime = t;
            eye[eyeNum].blinkFactor = 1.0;
          }
        } else {
          eye[eyeNum].blinkFactor = (float)(t - eye[eyeNum].blink.startTime) / (float)eye[eyeNum].blink.duration;
          if(eye[eyeNum].blink.state == DEBLINK) eye[eyeNum].blinkFactor = 1.0 - eye[eyeNum].blinkFactor;
        }
      }

      // Periodically report frame rate. Really this is "total number of
      // eyeballs drawn." If there are two eyes, the overall refresh rate
      // of both screens is about 1/2 this.
      frames++;
      if(((t - lastFrameRateReportTime) >= 1000000) && t) { // Once per sec.
        //Serial.println((frames * 1000) / (t / 1000)); //amw18 commented this out for testing
        lastFrameRateReportTime = t;
      }

      // Once per frame (of eye #0), reset boopSum...
      //amw18 see if we can comment this out bc not using boops
      /*if((eyeNum == 0) && (boopPin >= 0)) {
        boopSumFiltered = ((boopSumFiltered * 3) + boopSum) / 4;
        if(boopSumFiltered > boopThreshold) {
          if(!booped) {
            Serial.println("BOOP!");
          }
          booped = true;
        } else {
          booped = false;
        }
        boopSum = 0;
      }*/

      float mins = (float)millis() / 60000.0;
      if(eye[eyeNum].iris.iSpin) {
        // Spin works in fixed amount per frame (eyes may lose sync, but "wagon wheel" tricks work)
        eye[eyeNum].iris.angle   += eye[eyeNum].iris.iSpin;
      } else {
        // Keep consistent timing in spin animation (eyes stay in sync, no "wagon wheel" effects)
        eye[eyeNum].iris.angle    = (int)((float)eye[eyeNum].iris.startAngle   + eye[eyeNum].iris.spin   * mins + 0.5);
      }
      if(eye[eyeNum].sclera.iSpin) {
        eye[eyeNum].sclera.angle += eye[eyeNum].sclera.iSpin;
      } else {
        eye[eyeNum].sclera.angle  = (int)((float)eye[eyeNum].sclera.startAngle + eye[eyeNum].sclera.spin * mins + 0.5);
      }

      // END ONCE-PER-FRAME EYE ANIMATION ----------------------------------

    } // end first-scanline check

    // PER-COLUMN RENDERING ------------------------------------------------

    // Should be possible for these to be local vars,
    // but the animation becomes super chunky then, what gives?
    xPositionOverMap = (int)(eye[eyeNum].eyeX - (DISPLAY_SIZE/2.0));
    yPositionOverMap = (int)(eye[eyeNum].eyeY - (DISPLAY_SIZE/2.0));

    // These are constant across frame and could be stored in eye struct
    float upperLidFactor = (1.0 - eye[eyeNum].blinkFactor) * eye[eyeNum].upperLidFactor,
          lowerLidFactor = (1.0 - eye[eyeNum].blinkFactor) * eye[eyeNum].lowerLidFactor;
    if (eyeNum == 1) iPupilFactorL = (int)((float)eye[eyeNum].iris.height * 256 * (1.0 / eye[eyeNum].pupilFactor)); //amw18
    else if (eyeNum == 0) iPupilFactorR = (int)((float)eye[eyeNum].iris.height * 256 * (1.0 / eye[eyeNum].pupilFactor)); //amw18
    //amw18 updated the above for L and R. was just the first line, no if, with iPupilFactor no r/l specification

    int y1, y2;
    int lidColumn = (eyeNum & 1) ? (DISPLAY_SIZE - 1 - x) : x; // Reverse eyelid columns for left eye

    DmacDescriptor *d = &eye[eyeNum].column[eye[eyeNum].colIdx].descriptor[0];

    if(upperOpen[lidColumn] == 255) {
      // No eyelid data for this line; eyelid image is smaller than screen.
      // Great! Make a full scanline of nothing, no rendering needed:
      d->BTCTRL.bit.SRCINC = 0;
      d->BTCNT.reg         = DISPLAY_SIZE * 2;
      d->SRCADDR.reg       = (uint32_t)&eyelidIndex;
      d->DESCADDR.reg      = 0; // No linked descriptor
    } else {
      y1 = lowerClosed[lidColumn] + (int)(0.5 + lowerLidFactor *
        (float)((int)lowerOpen[lidColumn] - (int)lowerClosed[lidColumn]));
      y2 = upperClosed[lidColumn] + (int)(0.5 + upperLidFactor *
        (float)((int)upperOpen[lidColumn] - (int)upperClosed[lidColumn]));
      if(y1 > DISPLAY_SIZE-1)    y1 = DISPLAY_SIZE-1; // Clip results in case lidfactor
      else if(y1 < 0) y1 = 0;   // is beyond the usual 0.0 to 1.0 range
      if(y2 > DISPLAY_SIZE-1)    y2 = DISPLAY_SIZE-1;
      else if(y2 < 0) y2 = 0;
      if(y1 >= y2) {
        // Eyelid is fully or partially closed, enough that there are no
        // pixels to be rendered for this line. Make "nothing," as above.
        d->BTCTRL.bit.SRCINC = 0;
        d->BTCNT.reg         = DISPLAY_SIZE * 2;
        d->SRCADDR.reg       = (uint32_t)&eyelidIndex;
        d->DESCADDR.reg      = 0; // No linked descriptors
      } else {
        // If single eye, dynamically build descriptor list as needed,
        // else use a single descriptor & fully buffer each line.
#if NUM_DESCRIPTORS > 1
        DmacDescriptor *next;
        int             renderlen;
        if(y1 > 0) { // Do upper eyelid unless at top of image
          d->BTCTRL.bit.SRCINC = 0;
          d->BTCNT.reg         = y1 * 2;
          d->SRCADDR.reg       = (uint32_t)&eyelidIndex;
          next                 = &eye[eyeNum].column[eye[eyeNum].colIdx].descriptor[1];
          d->DESCADDR.reg      = (uint32_t)next; // Link to next descriptor
          d                    = next;           // Advance to next descriptor
        }
        // Partial column will be rendered
        renderlen            = y2 - y1 + 1;
        d->BTCTRL.bit.SRCINC = 1;
        d->BTCNT.reg         = renderlen * 2;
        d->SRCADDR.reg       = (uint32_t)eye[eyeNum].column[eye[eyeNum].colIdx].renderBuf + renderlen * 2; // Point to END of data!
#else
        // Full column will be rendered; DISPLAY_SIZE pixels, point source to end of
        // renderBuf and enable source increment.
        d->BTCTRL.bit.SRCINC = 1;
        d->BTCNT.reg         = DISPLAY_SIZE * 2;
        d->SRCADDR.reg       = (uint32_t)eye[eyeNum].column[eye[eyeNum].colIdx].renderBuf + DISPLAY_SIZE * 2;
        d->DESCADDR.reg      = 0; // No linked descriptors
#endif
        // Render column 'x' into eye's next available renderBuf
        uint16_t *ptr = eye[eyeNum].column[eye[eyeNum].colIdx].renderBuf;
        int xx = xPositionOverMap + x;
        int y;

#if NUM_DESCRIPTORS == 1
        // Render lower eyelid if needed
        for(y=0; y<y1; y++) *ptr++ = eyelidColor;
#else
        y = y1;
#endif

        // tablegen.cpp explains a bit of the displacement mapping trick.
        uint8_t *displaceX, *displaceY;
        int8_t   xmul; // Sign of X displacement: +1 or -1
        int      doff; // Offset into displacement arrays
        if(x < (DISPLAY_SIZE/2)) {  // Left half of screen (quadrants 2, 3)
          displaceX = &displace[ (DISPLAY_SIZE/2 - 1) - x       ];
          displaceY = &displace[((DISPLAY_SIZE/2 - 1) - x) * (DISPLAY_SIZE/2)];
          xmul      = -1; // X displacement is always negative
        } else {       // Right half of screen( quadrants 1, 4)
          displaceX = &displace[ x - (DISPLAY_SIZE/2)       ];
          displaceY = &displace[(x - (DISPLAY_SIZE/2)) * (DISPLAY_SIZE/2)];
          xmul      =  1; // X displacement is always positive
        }

        for(; y<=y2; y++) { // For each pixel of open eye in this column...
          int yy = yPositionOverMap + y;
          int dx, dy;

          if(y < (DISPLAY_SIZE/2)) { // Lower half of screen (quadrants 3, 4)
            doff = (DISPLAY_SIZE/2 - 1) - y;
            dy   = -displaceY[doff];
          } else {      // Upper half of screen (quadrants 1, 2)
            doff = y - (DISPLAY_SIZE/2);
            dy   =  displaceY[doff];
          }
          dx = displaceX[doff * (DISPLAY_SIZE/2)];
          if(dx < 255) {      // Inside eyeball area
            dx *= xmul;       // Flip sign of x offset if in quadrants 2 or 3
            int mx = xx + dx; // Polar angle/dist map coords
            int my = yy + dy;
            if((mx >= 0) && (mx < mapDiameter) && (my >= 0) && (my < mapDiameter)) {
              // Inside polar angle/dist map
              int angle, dist, moff;
              if(my >= mapRadius) {
                if(mx >= mapRadius) { // Quadrant 1
                  // Use angle & dist directly
                  mx   -= mapRadius;
                  my   -= mapRadius;
                  moff  = my * mapRadius + mx; // Offset into map arrays
                  angle = polarAngle[moff];
                  dist  = polarDist[moff];
                } else {                // Quadrant 2
                  // ROTATE angle by 90 degrees (270 degrees clockwise; 768)
                  // MIRROR dist on X axis
                  mx    = mapRadius - 1 - mx;
                  my   -= mapRadius;
                  angle = polarAngle[mx * mapRadius + my] + 768;
                  dist  = polarDist[ my * mapRadius + mx];
                }
              } else {
                if(mx < mapRadius) {  // Quadrant 3
                  // ROTATE angle by 180 degrees
                  // MIRROR dist on X & Y axes
                  mx    = mapRadius - 1 - mx;
                  my    = mapRadius - 1 - my;
                  moff  = my * mapRadius + mx;
                  angle = polarAngle[moff] + 512;
                  dist  = polarDist[ moff];
                } else {                // Quadrant 4
                  // ROTATE angle by 270 degrees (90 degrees clockwise; 256)
                  // MIRROR dist on Y axis
                  mx   -= mapRadius;
                  my    = mapRadius - 1 - my;
                  angle = polarAngle[mx * mapRadius + my] + 256;
                  dist  = polarDist[ my * mapRadius + mx];
                }
              }
              // Convert angle/dist to texture map coords
              if(dist >= 0) { // Sclera
                angle = ((angle + eye[eyeNum].sclera.angle) & 1023) ^ eye[eyeNum].sclera.mirror;
                int tx = angle * eye[eyeNum].sclera.width  / 1024; // Texture map x/y
                int ty = dist  * eye[eyeNum].sclera.height / 128;
                *ptr++ = eye[eyeNum].sclera.data[ty * eye[eyeNum].sclera.width + tx];
              } else if(dist > -128) { // Iris or pupil
                  int ty;
                  if (eyeNum == 1) ty = dist * iPupilFactorL / -32768; //amw18 added the if and eyeNum check for L
                  else ty = dist * iPupilFactorR / -32768; //amw18 added the if and eyeNum check for R
                if(ty >= eye[eyeNum].iris.height) { // Pupil
                  *ptr++ = eye[eyeNum].pupilColor;
                } else { // Iris
                  angle = ((angle + eye[eyeNum].iris.angle) & 1023) ^ eye[eyeNum].iris.mirror;
                  int tx = angle * eye[eyeNum].iris.width / 1024;
                  *ptr++ = eye[eyeNum].iris.data[ty * eye[eyeNum].iris.width + tx];
                }
              } else {
                *ptr++ = eye[eyeNum].backColor; // Back of eye
              }
            } else {
              *ptr++ = eye[eyeNum].backColor; // Off map, use back-of-eye color
            }
          } else { // Outside eyeball area
            *ptr++ = eyelidColor;
          }
        }

#if NUM_DESCRIPTORS == 1
        // Render upper eyelid if needed
        for(; y<DISPLAY_SIZE; y++) *ptr++ = eyelidColor;
#else
        if(y2 >= (DISPLAY_SIZE-1)) {
          // No third descriptor; close it off
          d->DESCADDR.reg      = 0;
        } else {
          next                 = &eye[eyeNum].column[eye[eyeNum].colIdx].descriptor[(y1 > 0) ? 2 : 1];
          d->DESCADDR.reg      = (uint32_t)next; // link to next descriptor
          d                    = next; // Increment descriptor
          d->BTCTRL.bit.SRCINC = 0;
          d->BTCNT.reg         = ((DISPLAY_SIZE-1) - y2) * 2;
          d->SRCADDR.reg       = (uint32_t)&eyelidIndex;
          d->DESCADDR.reg      = 0; // end of descriptor list
        }
#endif
      }
    }
    eye[eyeNum].column_ready = true; // Line is rendered!
  }

  // If DMA for this eye is currently busy, don't block, try next eye...
  if(eye[eyeNum].dma_busy) {
    if((micros() - eye[eyeNum].dmaStartTime) < DMA_TIMEOUT) return;
    // If we reach this point in the code, an SPI DMA transfer has taken
    // noticably longer than expected and is probably stalled (see comments
    // in the DMAbuddy.h file and above the DMA_TIMEOUT declaration earlier
    // in this code). Take action!
    // digitalWrite(13, HIGH);
    Serial.printf("Eye #%d stalled, resetting DMA channel...\n", eyeNum);
    eye[eyeNum].dma.fix();
    // If this somehow proves to be inadequate, we still have the Nuclear
    // Option of just completely restarting the sketch from the beginning,
    // though this stalls animation for several seconds during startup.
    // DO NOT enable this line unless the fix() function isn't fixing!
    //NVIC_SystemReset();
  }

  // At this point, above checks confirm that column is ready and DMA is free
  if(!x) { // If it's the first column...
    // End prior SPI transaction...
    digitalWrite(eye[eyeNum].cs, HIGH); // Deselect
    eye[eyeNum].spi->endTransaction();
    // Initialize new SPI transaction & address window...
    eye[eyeNum].spi->beginTransaction(settings);
    digitalWrite(eye[eyeNum].cs, LOW);  // Chip select
    eye[eyeNum].display->setAddrWindow((eye[eyeNum].display->width() - DISPLAY_SIZE) / 2, (eye[eyeNum].display->height() - DISPLAY_SIZE) / 2, DISPLAY_SIZE, DISPLAY_SIZE);
    delayMicroseconds(1);
    digitalWrite(eye[eyeNum].dc, HIGH); // Data mode
    if(eyeNum == (NUM_EYES-1)) {
      // Handle pupil scaling
      if(lightSensorPin >= 0) {
        // Read light sensor, but not too often (Seesaw hates that)
        #define LIGHT_INTERVAL (1000000 / 10) // 10 Hz, don't poll Seesaw too often
        if((t - lastLightReadTime) >= LIGHT_INTERVAL) { //amw18 if it's time to read the sensor again
          handleLightSensor(config, t); //amw18 call method to calculate lastLightValue L/R
          
        } //amw18 end of if time to read the sensor
        else lastLightReadTime = t - LIGHT_INTERVAL + 30000; // Try again in 30 ms amw18 (trying to get this to work)
        irisValueL = (irisValueL * 0.97) + (lastLightValueL * 0.03); // Filter response for smooth reaction //amw18 was just irisValue = iV
        irisValueR = (irisValueR * 0.97) + (lastLightValueR * 0.03); //amw18 added this for R/L
        //make sure they don't get smaller than irisMin
        if (irisValueL > (irisMinL + irisRange)) irisValueL = irisMinL + irisRange;
        if (irisValueR > (irisMinR + irisRange)) irisValueR = irisMinR + irisRange;
      } 
      else {  //amw18 lightSensorPin = -1 (like for none)
        irisValueR = irisMinR + (.5 * irisRange); //amw18 was just irisValue but updated for both R/L
        irisValueL = irisMinL + (.5 * irisRange); //amw18 added R/L
         
      } 
      /* amw18 comment the voice modulator pitch stuff out bc not using
#if defined(ADAFRUIT_MONSTER_M4SK_EXPRESS)
      if(voiceOn) {
        // Read buttons, change pitch
        arcada.readButtons();
        uint32_t buttonState = arcada.justPressedButtons();
        if(       buttonState & ARCADA_BUTTONMASK_UP) {
          currentPitch *= 1.05;
        } else if(buttonState & ARCADA_BUTTONMASK_A) {
          currentPitch = defaultPitch;
        } else if(buttonState & ARCADA_BUTTONMASK_DOWN) {
          currentPitch *= 0.95;
        }
        if(buttonState & (ARCADA_BUTTONMASK_UP | ARCADA_BUTTONMASK_A | ARCADA_BUTTONMASK_DOWN)) {
          currentPitch = voicePitch(currentPitch);
          if(waveform) voiceMod(modulate, waveform);
          Serial.print("Voice pitch: ");
          Serial.println(currentPitch);
        }
      }
#endif   */
      config = user_loop(config); //amw18 go to the loop to check for state switch
    }
  } // end first-column check

  // MUST read the booper when there’s no SPI traffic across the nose!
  /*if((eyeNum == (NUM_EYES-1)) && (boopPin >= 0)) {
    boopSum += readBoop();
  }*/ //amw18 remove boop functionality

  memcpy(eye[eyeNum].dptr, &eye[eyeNum].column[eye[eyeNum].colIdx].descriptor[0], sizeof(DmacDescriptor));
  eye[eyeNum].dma_busy       = true;
  eye[eyeNum].dma.startJob();
  eye[eyeNum].dmaStartTime   = micros();
  if(++eye[eyeNum].colNum >= DISPLAY_SIZE) { // If last line sent...
    eye[eyeNum].colNum      = 0;    // Wrap to beginning
  }
  eye[eyeNum].colIdx       ^= 1;    // Alternate 0/1 line structs
  eye[eyeNum].column_ready = false; // OK to render next line
}