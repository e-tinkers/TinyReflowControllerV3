/*******************************************************************************
  Title: Tiny Reflow Controller
  Version: 3.00
  Date: 10-Jun-2021
  Author: Henry Cheung
  Website: www.e-tinkers.com
  ------------------------------------------------------------------------------
  Version: 2.00
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX31856 and SSD1306 libraries. Adafruit has been the source
  of tonnes of tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core and megaTinyCore for Arduino:
  https://github.com/SpenceKonde/ATTinyCore
  https://github.com/SpenceKonde/megaTinyCore

  ==========================================
  Bill Greiman (github.com/greiman)
  ==========================================
  Maintainer of SSD1306Ascii library usedin V3.00:
  https://github.com/greiman/SSD1306Ascii

  ==========================================
  Henry Cheung (www.e-tinkers.com/about)
  ==========================================
  Maintainer of Button and MAX31855 libraries used in V3.00:
  https://github.com/e-tinkers/button
  https://github.com/e-tinkers/MAX31855

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  For Version 3.00 only:
  - E-Tinkers Button Library:
    >> https://github.com/e-tinkers/button
  - E-Tinkers MAX31855 Library:
    >> https://github.com/e-tinkers/MAX31855
  - SSD1306Ascii Library:
    >> https://github.com/greiman/SSD1306Ascii
  For Version 2.00 or earlier version only:
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library
  - Adafruit_MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856

  Revision  Description
  ========  ===========
  3.00      Re-write by Henry Cheung
            - Simplified many parts of the code to make it less less busier.
            - Switch from using Adafruit_SSD1306 library to SSD1306Ascii.h for
              supporting cheap SH-1106-based OLED.
            - Replace the code for button deboncing with <button.h> Library
              (https://github.com/e-tinkers/button).
            - Replace Adafruit_MAX31856 library with e-tinkers' MAX31855 library.
            - Tested on both ATMega32U4 Leonardo board (with ArduinoAVR-core)
              and e-tinkers' ATtiny3217 board (with TinyMegaCore).
  2.00      Support V2 of the Tiny Reflow Controller:
            - Based on ATMega328P 3.3V @ 8MHz
            - Uses SSD1306 128x64 OLED
  1.00      Initial public release:
            - Based on ATtiny1634R 3.3V @ 8MHz
            - Uses 8x2 alphanumeric LCD

*******************************************************************************/

// ***** INCLUDES *****
#include <Arduino.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <MAX31855.h>
#include <button.h>

// ***** ENABLE SERIAL PRINTOUT OUTPUT *****
// #define SERIAL_PRINTOUT

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000    // thermocouple reading interval
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** PID PARAMETERS *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20

#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250

#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD DISPLAY *****
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define I2C_ADDRESS 0x3c
#define X_AXIS_START 18 // X-axis starting position for the chart
#define UPDATE_RATE 100

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;
reflowState_t reflowState;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;
reflowStatus_t reflowStatus;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;
reflowProfile_t reflowProfile;

// ***** LCD MESSAGES *****
const char ready_m[] PROGMEM = "Ready ";
const char preheat_m[] PROGMEM = "PreHt ";
const char soak_m[] PROGMEM = "Soak  ";
const char reflow_m[] PROGMEM = "Reflow";
const char coolDn_m[] PROGMEM = "CoolDn";
const char done_m[] PROGMEM = "Done! ";
const char hot_m[] PROGMEM = "Hot!  ";
const char error_m[] PROGMEM = "Error";

PGM_P const lcdMessages[] PROGMEM = {
  ready_m,
  preheat_m,
  soak_m,
  reflow_m,
  coolDn_m,
  done_m,
  hot_m,
  error_m
};

// ***** PIN ASSIGNMENT *****
uint8_t chipSelectPin = A5;
uint8_t ssrPin = 12;
uint8_t fanPin = 11;
uint8_t buzzerPin = 10;
uint8_t ledPin = A2;
uint8_t btn1Pin = A3;
uint8_t btn2Pin = A4;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double thermoReading;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
unsigned long windowSize;
unsigned long windowStartTime;

unsigned long nextRead;    // for keeping track of thermocouple reading interval
unsigned long updateLcd;
unsigned long timerSoak;
uint8_t soakTemperatureMax;
uint8_t reflowTemperatureMax;
unsigned long soakMicroPeriod;

// Seconds timer
unsigned int timerSeconds;
unsigned int temperatureUpdate;
uint8_t temperature[SCREEN_WIDTH - X_AXIS_START];
uint8_t idx;

PID reflowOvenPID(&thermoReading, &output, &setpoint, kp, ki, kd, DIRECT);
SSD1306AsciiWire oled;
MAX31855 thermocouple(chipSelectPin);
Button startBtn;      // For start/stop
Button profileBtn;    // For selection of Lead-Free or Leaded profile

/* A helper function to print the degree symbol on LCD display */
void printDegreeSymbol() {
  const char degree[6] = {0x00, 0x06, 0x09, 0x09, 0x06, 0x00};
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0x040);
  for (uint8_t i=0; i<6; i++) {
    Wire.write(degree[i]);
  }
  Wire.endTransmission();
}

/* A helper function for drawing a pixel on SH1106 display */
void drawPixel() __attribute__((always_inline));
inline void drawPixel(uint8_t x, uint8_t y, bool stackUp=false) {
  oled.setCursor(x, y>>3);
  Wire.beginTransmission(I2C_ADDRESS);
  uint8_t colData = 0;
  if (stackUp)         // if going to be a vertical line, it need to get the existing data
    colData = Wire.read();
  Wire.write(0x040);
  Wire.write( (1 << (y & 7)) | colData);
  Wire.endTransmission();
}

/*
 * update display - re-factor this part of code out of the loop() as an inline
 * function to make the loop() less crowder.
*/
void updateDisplay() __attribute__((always_inline));
inline void updateDisplay()
{
    oled.set2X();
    oled.setCursor(0, 0);
    char buff[7];
    strcpy_P(buff, (PGM_P)pgm_read_word(&lcdMessages[reflowState]));
    oled.print(buff);

    oled.set1X();
    oled.setCursor(80, 0);
    if (reflowStatus == REFLOW_STATUS_OFF) {
      oled.print(F("      "));
    }
    else {
      oled.print((int)setpoint);
      printDegreeSymbol();
      oled.print(F("C "));
    }

    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
      oled.print(F("LF"));
    }
    else
    {
      oled.print(F("PB"));
    }

    if (reflowState == REFLOW_STATE_ERROR) {
      oled.setCursor(115, 1);
      oled.print(F("TC"));
    }

    // Right align temperature reading
    char tempStr[10];
    sprintf(tempStr, "%4d", (int)thermoReading);
    oled.setCursor(74, 1);
    oled.print(tempStr);
    printDegreeSymbol();
    oled.print(F("C"));

    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // We are updating the display faster than sensor reading
      if (timerSeconds > temperatureUpdate)
      {
        // Store temperature reading every 4 s
        if ((timerSeconds % 4) == 0)
        {
          temperatureUpdate = timerSeconds;
          uint8_t averageReading = map(thermoReading, 0, 260, 63, 19);
          if (idx < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[idx++] = averageReading;
          }
        }
      }
    }

    for (uint8_t timeAxis = 0; timeAxis < idx; timeAxis++)
    {
      drawPixel(timeAxis + X_AXIS_START + 1, temperature[timeAxis]);
    }

}

void setup()
{
#ifdef SERIAL_PRINTOUT
  Serial.begin(115200);
  while (!Serial);
#endif

  // Check last-save reflow profile value, if not exist, default to lead-free profile
  reflowProfile_t value = (reflowProfile_t) EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == REFLOW_PROFILE_LEADFREE) || (value == REFLOW_PROFILE_LEADED))
  {
    reflowProfile = value;
  }
  else
  {
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }

  // pin initializations
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  startBtn.begin(btn1Pin);
  profileBtn.begin(btn2Pin);

  // Start-up splash
  digitalWrite(ledPin, HIGH);   // turn on led and buzzer
  digitalWrite(buzzerPin, HIGH);
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&SH1106_128x64, I2C_ADDRESS);
  oled.setFont(font5x7);
  oled.clear();
  oled.println(F("     Tiny Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("     Version 3.00"));
  oled.println();
  oled.println(F("     2021-06-10"));
  delay(500);
  digitalWrite(buzzerPin, LOW);
  delay(3000);
  digitalWrite(ledPin, LOW);

  // Temperature markers and time axis
  oled.clear();
  oled.setCursor(0, 2);
  oled.print(F("250"));
  oled.setCursor(0, 4);
  oled.print(F("150"));
  oled.setCursor(0, 6);
  oled.print(F(" 50"));
  for (uint8_t i = 18; i<SCREEN_HEIGHT-1; i++)
    drawPixel(X_AXIS_START, i, true);    // draw a vertical line
  for (uint8_t i = X_AXIS_START+1; i<SCREEN_WIDTH; i++)
    drawPixel(i, SCREEN_HEIGHT-1);       // draw a horizontal line

  // Initialize thermocouple interface
  if (thermocouple.begin() != 0) {
    reflowState = REFLOW_STATE_ERROR;    //thermocouple connection error
  };


  windowSize = 2000;    // time in ms for PID calculation
  nextRead = millis();
  updateLcd = millis();

}

void loop()
{
  static unsigned long buzzerPeriod;

  // update display every UPDATE_RATE(100ms)
  if (millis() - updateLcd >= UPDATE_RATE)
  {
    updateDisplay();
    updateLcd = millis();
  }

  // read thermocouple every SENSOR_SAMPLING_TIME (1000ms)
  if (millis() - nextRead >= SENSOR_SAMPLING_TIME)
  {
    nextRead = millis();
    thermoReading = thermocouple.thermocoupleTemperature();
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      digitalWrite(ledPin, HIGH);
      timerSeconds++;

#ifdef SERIAL_PRINTOUT
      Serial.print(timerSeconds);
      Serial.print(F(", "));
      Serial.print(setpoint);
      Serial.print(F(", "));
      Serial.print(thermoReading);
      Serial.print(F(", "));
      Serial.println(output);
#endif

    }
    else
    {
      digitalWrite(ledPin, LOW);
    }
  }

  // if Start/Stop button pressed, and current reflow process is on going, turn it off
  if (startBtn.debounce() && (reflowStatus == REFLOW_STATUS_ON))
  {
    reflowStatus = REFLOW_STATUS_OFF;
    reflowState = REFLOW_STATE_IDLE;
  }

  // if LF/RF button is pressed and only reflow process is idle, it allows to toggle
  if (profileBtn.debounce() && (reflowState == REFLOW_STATE_IDLE))
  {
    // toggle the profile state
    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      reflowProfile = REFLOW_PROFILE_LEADED;
    else
      reflowProfile = REFLOW_PROFILE_LEADFREE;
    EEPROM.write(PROFILE_TYPE_ADDRESS, reflowProfile);
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (thermoReading >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (startBtn.debounce())
        {

#ifdef SERIAL_PRINTOUT
          Serial.println(F("Time, Setpoint, Temperature, Output"));
#endif
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;

          // Initialize reflow plot update timer
          temperatureUpdate = 0;

          for (idx = 0; idx < (SCREEN_WIDTH - X_AXIS_START+1); idx++)
          {
            temperature[idx] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          idx = 0;

          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE)
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }
          else
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (thermoReading >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax)
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // Temperature continue to rise by 10 degree when reach reflowTemperatureMax
      // To avoid hovering at peak temperature for too long, switch to CoolDn earlier
      if (thermoReading >= (reflowTemperatureMax - 10))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
      }
      if (thermoReading >= reflowTemperatureMax) {
        // Display only switch to 'CoolDn' when reach to the peak temp
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (thermoReading <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        digitalWrite(fanPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (thermoReading < TEMPERATURE_ROOM)
      {
        digitalWrite(fanPin, LOW);
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    default:
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {

    reflowOvenPID.Compute();

    if ((millis() - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (millis() - windowStartTime))
      digitalWrite(ssrPin, HIGH);
    else
      digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    if (digitalRead(ssrPin)!=LOW) digitalWrite(ssrPin, LOW);
  }

}
