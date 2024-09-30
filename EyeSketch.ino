//amw18 attempt to pull out Monster M4sk eye code to use for Sim Lab Mannekins
#if !defined(USE_TINYUSB)
  #error "Please select Tools->USB Stack->TinyUSB before compiling"
#endif

#define GLOBAL_VAR
#include "globals.h"

// Global eye state that applies to all eyes (not per-eye):
myConfig config; //amw18 creating a config to hold all the eye states
bool     eyeInMotion = false;
float    eyeOldX, eyeOldY, eyeNewX, eyeNewY;
uint32_t eyeMoveStartTime = 0L;
int32_t  eyeMoveDuration  = 0L;
uint32_t lastSaccadeStop  = 0L;
int32_t  saccadeInterval  = 0L;

// Some sloppy eye state stuff, some carried over from old eye code...
// kinda messy and badly named and will get cleaned up/moved/etc.
uint32_t timeOfLastBlink         = 0L,
         timeToNextBlink         = 0L;
int      xPositionOverMap        = 0;
int      yPositionOverMap        = 0;
uint8_t  eyeNum                  = 0;
uint32_t frames                  = 0;
uint32_t lastFrameRateReportTime = 0;
uint32_t lastLightReadTime       = 0;
float    lastLightValue          = 0.5; //amw18 adding in a R and L
float    lastLightValueL         = 0.5; //amw18 added
float    lastLightValueR         = 0.5; //amw18
double   irisValue               = 0.5; //amw18 adding in values for R and L
double   irisValueR              = 0.5; //amw18 added
double   irisValueL              = 0.5; //amw18 added
int      iPupilFactor            = 42; //amw18 adding in a R and L
int      iPupilFactorL           = 42; //amw18
int      iPupilFactorR           = 42; //amw18 added 
uint32_t boopSum                 = 0,
         boopSumFiltered         = 0;
bool     booped                  = false;
int      fixate                  = 7;
uint8_t  lightSensorFailCount    = 0;
uint32_t startTime, elapsed; //amw18 added

// WAV player stuff amw18 pulled from fizzgig.cpp
#define WAV_BUFFER_SIZE    256
static uint8_t     wavBuf[2][WAV_BUFFER_SIZE];
static File        wavFile;
static bool        playing = false;
static int         remainingBytesInChunk;
static uint8_t     activeBuf;
static uint16_t    bufIdx, bufEnd, nextBufEnd;
static bool        startWav(char *filename);
static void        wavOutCallback(void);
static uint32_t    wavEventTime; // WAV start or end time, in ms
static const char *wav_path = "wav files";
/*static struct wavlist { // Linked list of WAV filenames
  char           *filename;
  struct wavlist *next;
} *wavListStart = NULL, *wavListPtr = NULL;*/ //amw18 I don't think I need this since no looping sounds
#define MAX_WAV_FILES 20
//amw18 end of wav stuff pulled from fizzgig.cpp

// Callback invoked after each SPI DMA transfer - sets a flag indicating
// the next line of graphics can be issued as soon as its ready.
static void dma_callback(Adafruit_ZeroDMA *dma) {
  // It's possible to assign each DMA channel its own callback function
  // (freeing up a few cycles vs. this channel-to-eye lookup), but it's
  // written this way to scale to as many eyes as needed (up to one per
  // SERCOM if this is ported to something like Grand Central).
  //amw18 update this to accommodate two eyes only (0, 1 = right, left)
  if (dma == &eye[0].dma) {
    eye[0].dma_busy = false;
    return;
  }
  if (dma == &eye[1].dma){
    eye[1].dma_busy = false;
    return;
  }
}

#define DISPLAY_FREQ   50000000
#define DISPLAY_CLKSRC SERCOM_CLOCK_SOURCE_100M

SPISettings settings(DISPLAY_FREQ, MSBFIRST, SPI_MODE0);

#define DMA_TIMEOUT (uint32_t)((DISPLAY_SIZE * 16 * 4000) / (DISPLAY_FREQ / 1000))

// Crude error handler. Prints message to Serial Monitor, blinks LED.
void fatal(const char *message, uint16_t blinkDelay) {
  Serial.begin(9600);
  Serial.println(message);
  for(bool ledState = HIGH;; ledState = !ledState) {
    digitalWrite(LED_BUILTIN, ledState);
    delay(blinkDelay);
  }
}

#include <unistd.h> // sbrk() function

uint32_t availableRAM(void) {
  char top;                      // Local variable pushed on stack
  return &top - (char *)sbrk(0); // Top of stack minus end of heap
}


void setup() {
  //check arduino settings
  if(!arcada.arcadaBegin())     fatal("Arcada init fail!", 100);
  #if defined(USE_TINYUSB)
  if(!arcada.filesysBeginMSD()) fatal("No filesystem found!", 250);
  #else
  if(!arcada.filesysBegin())    fatal("No filesystem found!", 250);
  #endif
  //turn on the display
  arcada.displayBegin();
  // Backlight(s) off ASAP, they'll switch on after screen(s) init & clear
  arcada.setBacklight(0);
  DISPLAY_SIZE = min(ARCADA_TFT_WIDTH, ARCADA_TFT_HEIGHT);
  yield(); // Periodic yield() makes sure mass storage filesystem stays alive
  char *filename = (char *)"config.eye";
  //amw18 set initial/default eye states
  config.Move = MOVING;
  config.Blink = NORMAL;
  config.lSize = MED;
  config.rSize = MED;
  config.lLight = BOTH;
  config.rLight = BOTH;
  lightSensorPin = 0;
  yield();
  // Initialize display(s) amw18 assume two eyes
  eye[0].display = arcada._display;
  eye[1].display = arcada.display2;  
  //amw18 pulled initializations into separate functions for clarity
  initializeDMAS();
  splashScreen();
  loadConfig(filename);
  loadTextureMaps();
  loadEyelidGraphics();
  //getWavFiles(); don't think we need to do this
  lastLightReadTime = micros() + 2000000; // Delay initial light reading
  irisRange = .3; //amw18 see if this prevents eyes from constricting to nothing
}

void loop() {
  handleMoveSize();
  drawEyes(); //amw18 this calls user_loop which checks for serial input
}

myConfig user_loop(myConfig config) {  //amw18 this checks for serial input and changes the config values accordingly
  Serial.flush();
  Serial.begin(9600);
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    //wav file state
    if (command == "Bark"){ //play the bark wav file
      startWav("wav files/BigDogBark.wav");
      return config;
    }
    if (command == "Agg"){ //play the aggressive bark file
      startWav("wav files/AggressiveDogGrowlBark.wav");
      return config;
    }
    if (command == "Snarl"){ //play the snarl file
      startWav("wav files/DogBarkSnarl.wav");
      return config;
    }
    //movement states
    if (command == "MoveM"){ //both moving
      config.Move = MOVING; 
      return config;
    }
    else if (command == "MoveF"){ //both fixed
      config.Move = FIXED;
      return config;
    }
    // blink states
    else if (command == "BlinkN"){ //both blink normally
      config.Blink = NORMAL;
      return config;
    }
    else if (command == "BlinkS"){ //both blink slowly
      config.Blink = SLOW;
      return config;
    }
    else if (command == "BlinkU"){
      config.Blink = UNCONSCIOUS;
      return config;
    }
    //pupil size states
    else if (command == "lSizeM"){ //left pupil is medium
      config.lSize = MED;
      return config;
    }
    else if (command == "lSizeB"){ //left pupil is big
      config.lSize = BIG;
      return config;
    }
    else if (command == "lSizeS"){ //left pupil is small
      config.lSize = SMALL;
      return config;
    }
    else if (command == "rSizeM"){ //right pupil is medium
      config.rSize = MED;
      return config;
    }
    else if (command == "rSizeB"){ //right pupil is big
      config.rSize = BIG;
      return config;
    }
    else if (command == "rSizeS"){ //right pupil is small
      config.rSize = SMALL;
      return config;
    }
    //light states
    else if (command == "lLightB") { //left responds to both inputs
        config.lLight = BOTH;
        return config;
      }
    else if (command == "lLightL") { //left responds to left input only
        config.lLight = LEFT;
        return config;
      }
    else if (command == "lLightR"){ //left responds to right input only
        config.lLight = RIGHT;
        return config;
    }
    else if (command == "lLightN"){ //left responds to neither input
        config.lLight = NONE;
        return config;
    }
    else if (command == "rLightB") { //right responds to both inputs
        config.rLight = BOTH;
        return config;
      }
    else if (command == "rLightL") { //right responds to left input only
        config.rLight = LEFT;
        return config;
      }
    else if (command == "rLightR"){ //right responds to right input only
        config.rLight = RIGHT;
        return config;
    }
    else if (command == "rLightN"){ //right responds to right input only
        config.rLight = NONE;
        return config;
    } 
  } 
  return config;
}


void handleMoveSize(){ //handle movement and size of eyes
  switch (config.Move){
    case MOVING:{ 
      moveEyesRandomly = true; //move eyes normally
      break;
    }
    case FIXED:{
      moveEyesRandomly = false; //don't move eyes
      break;
    }
  }
  if (config.Blink == UNCONSCIOUS) {
    arcada.setBacklight(0); //turn off the display, no eyes drawn
    config = user_loop(config); //check for state change in the user_loop (currently a button press)
    if (config.Blink != UNCONSCIOUS) arcada.setBacklight(255); //if state changes from unconscious, turn display back on
    return;
  }
  switch (config.lSize){ //left eye size
    case MED:{
      irisMinL = .6; //iris is regular size
      break;
    }
    case BIG:{
      irisMinL = .4; //make the pupil large
      break;  
    }
    case SMALL:{
      irisMinL = .8; //make the pupil small
      break;
    }
  }
  switch (config.rSize){ //right eye size
    case MED:{
      irisMinR = .6; //iris is regular size
      break;
    }
    case BIG:{
      irisMinR = .4; //iris is big
      break;  
    }
    case SMALL:{
      irisMinR = .8; //iris is small
      break;
    }
  }
  /* amw18 add code for wav file playback here
  arcada.chdir(wav_path);
      startWav(wavListPtr->filename);
      wavListPtr = wavListPtr->next; */
}


