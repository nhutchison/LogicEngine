///
// SERIAL CONFIGURATION
///

#define BAUDRATE 2400 //this is the baudrate that we use to listen for incoming commands over JEDI_SERIAL
//#define DEBUG_SERIAL SerialUSB
//#define JEDI_SERIAL Serial1
#define DEBUG  // uncomment this line to output debug messages on Debug serial

// TEECES STUFF ....
// TODO :  TEST THIS!!!!!!!
// It compiles if set to 1, the rest is just a cut/paste from Pauls Sketch ... Zero testing!
#define TEECESPSI 0

//Teeces PSI related settings...
#define TEECES_D_PIN 12
#define TEECES_C_PIN 10
#define TEECES_L_PIN 6
#define RPSIbright 15 //rear PSI
#define FPSIbright 15 //front PSI
#define PSIstuck 5 //odds (in 100) that a PSI will get partially stuck between 2 colors

// handle to the Serial object
Stream* serialPort;
Stream* debugSerialPort;


///////////////////////////////////////////////////
//////////// Assign IC2 Address Below ////////////
/////////////////////////////////////////////////

byte I2CAdress = 21;


///////////////////////////////////////////////////
/////////////////////////////////////////////////


///////////////////////////////////////////////////
///////////////// Timer Settings /////////////////
/////////////////////////////////////////////////

bool alwaysOn = false;


///////////////////////////////////////////////////
////////////// SET DEFAULT PATTERN ///////////////
/////////////////////////////////////////////////

// Any display Mode can be the default Mode the PSI returns to
// after completing a command initiated Mode.  The standard default Mode
// is Swipe.  Use this to set the default Mode number.

uint8_t defaultPattern = 1; //Mode 1 is Random Blinkies


///////////////////////////////////////////////////
////////////// Define Display addresses ///////////////
/////////////////////////////////////////////////
#define ALL_LD 0
#define FLD_TOP 1
#define FLD_BOTTOM 2
#define RLD 3



///
// LED CONFIGURATION
///

#define NUM_FRONT_LEDS 80
#define NUM_REAR_LEDS 96

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define FRONT_PIN 21
  #define REAR_PIN 22
  #define STATUSLED_PIN 10
#elif defined(__SAMD21G18A__)
  #define FRONT_PIN 5
  #define REAR_PIN 3
  #define STATUSLED_PIN 8
#else
  // No Board settings known for LED pins ... I'm going to barf this!
  #error UNRECOGNIZED PINOUT.  Sorry.
#endif

#define FRONT_COL 8
#define FRONT_ROW 5
#define FRONT_ALL_ROW (FRONT_ROW * 2)
#define REAR_COL 24
#define REAR_ROW 4

///
// Palette and stuff config
///

DEFINE_GRADIENT_PALETTE( front_gp ) {
  0,   0,     0,   0,   //black
110,   0,     0, 255,   //blue
170,   0,     0, 200,   //blue, marginally less bright (used to reduce the total amount of "white"
230,   255, 255, 255,   //white
255,   0,    0,    0 }; //black

DEFINE_GRADIENT_PALETTE( rear_gp ) {
  0,   0,     0,   0,   //black
 50,   0,     255, 0,   //bright green
140,   180, 255, 1,   //yellow green
210,   246, 172, 0,   //orangey yellow
290,   255,   0,   0,   //red
255,   0,    0,    0};  //black

DEFINE_GRADIENT_PALETTE( rear_gp2 ) {
 30,   0,     0,   0,   //black
 70,   246, 172, 0,   //orangey yellow
 80,   255,   0,   0,   //red
100,   0,     255, 0,   //bright green
210,   180, 255, 1,   //yellow green
255,   0,    0,    0};  //black

CRGBPalette16 frontTargetPalette( front_gp);
CRGBPalette16 frontCurrentPalette( front_gp);
CRGBPalette16 rearCurrentPalette( rear_gp);
CRGBPalette16 rearTargetPalette( rear_gp);

#define UPDATES_PER_SECOND 140 // The bigger this number the faster the blinkies blink.  Value between 1 and 200
#define FRONT_COLOR_STEP 6  // Playing with this really changes how the blinkies look, smaller is "smoother"
#define REAR_COLOR_STEP 3  // Playing with this really changes how the blinkies look, smaller is "smoother"

// Chance percentage of an LED changing
#define PER_CHANGE_CHANCE 60

uint8_t frontColorIndex[NUM_FRONT_LEDS];
uint8_t rearColorIndex[NUM_REAR_LEDS];

// Addressible LED Array
// -1 = no LED in that grid space

uint8_t frontTopLedMatrix[FRONT_COL][FRONT_ROW] = {
  { 0, 15, 16, 31, 32,},
  { 1, 14, 17, 30, 33,},
  { 2, 13, 18, 29, 34,},
  { 3, 12, 19, 28, 35,},
  { 4, 11, 20, 27, 36,},
  { 5, 10, 21, 26, 37,},
  { 6,  9, 22, 25, 38,},
  { 7,  8, 23, 24, 39,},
};

uint8_t frontBottomLedMatrix[FRONT_COL][FRONT_ROW] = {
  { 40, 55, 56, 71, 72,},
  { 41, 54, 57, 70, 73,},
  { 42, 53, 58, 69, 74,},
  { 43, 52, 59, 68, 75,},
  { 44, 51, 60, 67, 76,},
  { 45, 50, 61, 66, 77,},
  { 46, 49, 62, 65, 78,},
  { 47, 48, 63, 64, 79,},
};

uint8_t bothFrontLedMatrix[FRONT_COL][FRONT_ALL_ROW] = {
  {  0, 15, 16, 31, 32, 40, 55, 56, 71, 72,},
  {  1, 14, 17, 30, 33, 41, 54, 57, 70, 73,},
  {  2, 13, 18, 29, 34, 42, 53, 58, 69, 74,},
  {  3, 12, 19, 28, 35, 43, 52, 59, 68, 75,},
  {  4, 11, 20, 27, 36, 44, 51, 60, 67, 76,},
  {  5, 10, 21, 26, 37, 45, 50, 61, 66, 77,},
  {  6,  9, 22, 25, 38, 46, 49, 62, 65, 78,},
  {  7,  8, 23, 24, 39, 47, 48, 63, 64, 79,},
};

uint8_t rearLedMatrix[REAR_COL][REAR_ROW] = {
  {  0, 47, 48, 95,},
  {  1, 46, 49, 94,},
  {  2, 45, 50, 93,},
  {  3, 44, 51, 92,},
  {  4, 43, 52, 91,},
  {  5, 42, 53, 90,},
  {  6, 41, 54, 89,},
  {  7, 40, 55, 88,},
  {  8, 39, 56, 87,},
  {  9, 38, 57, 86,},
  { 10, 37, 58, 85,},
  { 11, 36, 59, 84,},
  { 12, 35, 60, 83,},
  { 13, 34, 61, 82,},
  { 14, 33, 62, 81,},
  { 15, 32, 63, 80,},
  { 16, 31, 64, 79,},
  { 17, 30, 65, 78,},
  { 18, 29, 66, 77,},
  { 19, 28, 67, 76,},
  { 20, 27, 68, 75,},
  { 21, 26, 69, 74,},
  { 22, 25, 70, 73,},
  { 23, 24, 71, 72,},
};


//Setup Debug stuff for Real Arduino Pro Micros
#ifdef DEBUG
    #define DEBUG_PRINT_LN(msg)  debugSerialPort->println(msg)
    #define DEBUG_PRINT(msg)  debugSerialPort->print(msg)
#else
  #define DEBUG_PRINT_LN(msg)
  #define DEBUG_PRINT(msg)
#endif // DEBUG

// Command processing stuff
// maximum number of characters in a command (63 chars since we need the null termination)
#define CMD_MAX_LENGTH 64 

// memory for command string processing
char cmdString[CMD_MAX_LENGTH];
