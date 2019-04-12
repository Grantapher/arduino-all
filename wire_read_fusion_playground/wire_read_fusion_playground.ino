#include <Wire.h>
#include <FastLED.h>

// Log all to Serial, comment this line to disable logging
#define LOG Serial
// Include must be placed after LOG definition to work
#include "log.h"

#define NUM_LEDS 300
#define DATA_PIN 2

#define OUTPUT_A 11
#define OUTPUT_B 10

#define ROTARY_MAX 50
#define ROTARY_MIN 0
#define ROTARY_START 0

#define TICK_MIN 5
#define TICK_MAX 500

#define DELTA_HUE 32
#define NUM_SPOKES 8


int SPOKE_LENGTH = NUM_LEDS / NUM_SPOKES;

// I2C input data
byte input[8];

unsigned int tickPr = 0;

int16_t rotaryCount = ROTARY_START;
int aState;
int aLastState;

unsigned long startMs;
unsigned long startMsPr;

uint8_t prevHue = 0;

uint16_t maxDots = NUM_LEDS / 2;
int16_t dots[NUM_LEDS / 2];

// todo make threshold configurable
uint8_t threshold = 3;

CRGBArray <NUM_LEDS> leds;
int hue = 0;
int pix = 0;

void onData(int numBytes) {
  int i = 0;
  while (Wire.available()) {
    input[i++] = Wire.read();
  }
}

int arrayAverage(int8_t ray[], uint8_t startIndex, uint8_t endIndex) {
  float sum = 0;
  for (int i = startIndex; i < endIndex; i++) {
    sum += ray[i];
  }
  return sum / (endIndex - startIndex);
}

void KickFlash() {
  fadeToBlackBy( leds, NUM_LEDS, 64);
  if (input[0] > threshold) {
    fill_rainbow(leds, NUM_LEDS, 0, 255 / NUM_LEDS);
  }
}

uint16_t dotIndex = 0;
void KickAndRun() {
  fadeToBlackBy( leds, NUM_LEDS, rotaryCount ? 255 : 128);

  uint16_t i;
  //increment active dots
  for (i = 0; i < maxDots; i++) {
    if (dots[i] >= 0) dots[i]++;
    if (dots[i] > maxDots) dots[i] = -1;
  }

  if (input[0] > threshold) {
    dots[dotIndex] = 0;
    dotIndex = (dotIndex + 1) % maxDots;
  }

  CRGB color;
  uint16_t ledIndex;
  for (i = 0; i < maxDots; i++) {
    if (dots[i] >= 0) {
      ledIndex = dots[i];
      fill_rainbow(&color, 1, map(ledIndex, 0, maxDots, 255, 0), DELTA_HUE);

      leds[getIndex(maxDots, NUM_LEDS, -ledIndex)] = color;
      leds[getIndex(maxDots, NUM_LEDS, +ledIndex)] = color;
    }
  }
}

// Utility function to wrap array index
int getIndex(int current, int maximum, int add) {
  int ret = current + add;
  if (ret < 0) ret += maximum;
  if (ret > maximum) ret -= maximum;
  return ret;
}

uint8_t Spaceship_i = 0;
uint16_t prevPos = sin8(0);
void Spaceship() {
  fadeToBlackBy( leds, NUM_LEDS, 24);

  uint64_t currentMs = millis();
  if (currentMs - startMs >= 10) {
    CRGB color;
    fill_rainbow(&color, 1, prevHue, DELTA_HUE);

    uint8_t pos = map(sin8(Spaceship_i++), 0, 255, 0, SPOKE_LENGTH * 2);

    for (uint8_t spoke = 0; spoke < NUM_SPOKES; spoke++) {
      leds[(spoke * SPOKE_LENGTH + pos) % NUM_LEDS] = color;
    }

    if (prevPos != pos) {
      prevPos = pos;
      prevHue += DELTA_HUE;
    }

    startMs = currentMs;
  }
}

void glitter() {
  fadeToBlackBy( leds, NUM_LEDS, 3);
  if ( random8() < 20) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = beatsin16(13, 0, NUM_LEDS);
  fill_rainbow(&(leds[pos]), 1, prevHue, DELTA_HUE);
  prevHue += 2;
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 1);
  uint64_t currentMs = millis();
  if (currentMs - startMs >= 25) {
    int pos = random16(NUM_LEDS);
    leds[pos] += CHSV(random8(255), 200, 255);
    startMs = currentMs;
  }
}

int Wheel_i = 0;
bool Wheel_high = false;
void WheelAuto() {
  //todo perhaps add beat detection if possible?
  bool thresholdMet = input[0] >= threshold;

  if (!thresholdMet) {
    fadeToBlackBy(leds, NUM_LEDS, 16);
  } else {
    int startLed = Wheel_i * SPOKE_LENGTH;
    int endLed = startLed + SPOKE_LENGTH;
    fadeToBlackBy(leds, startLed, 16);
    fadeToBlackBy(&(leds[endLed]), NUM_LEDS - endLed, 16);
  }

  uint64_t currentMs = millis();
  if (thresholdMet && !Wheel_high && currentMs - startMs >= 50) {
    Wheel_high = true;

    Wheel_i = (Wheel_i + 1) % NUM_SPOKES;
    fill_rainbow(&(leds[Wheel_i * SPOKE_LENGTH]), SPOKE_LENGTH, 0, 255 / SPOKE_LENGTH);
    startMs = currentMs;
  }

  if (!thresholdMet && Wheel_high) {
    Wheel_high = false;
  }
}

uint16_t state = 0;
void WheelManual() {
  //todo configure speed with potentiometer, add
  fadeToBlackBy(leds, NUM_LEDS, 5);

  uint64_t currentMs = millis();
  if (currentMs - startMs >= 20) {
    fill_rainbow(&(leds[state]), 1, prevHue, DELTA_HUE);
    state = (state + 1) % NUM_LEDS;
    startMs = currentMs;
    prevHue += DELTA_HUE;
  }
}

void EQ() {
  fadeToBlackBy( leds, NUM_LEDS, 32);
  for (size_t x = 0; x < 8; x++) {
    fill_rainbow(&(leds[x * SPOKE_LENGTH]), map(input[x], 0, 10, 0, SPOKE_LENGTH), 0, DELTA_HUE);
  }
}

typedef void (*LedFunctionArray[])(void);
LedFunctionArray gPatterns = { KickAndRun, KickAndRun, KickFlash, Spaceship, glitter, juggle, sinelon, confetti, WheelManual, WheelAuto, EQ };
bool patternRawStatus[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 };
size_t gPatternsSize = sizeof(gPatterns) / sizeof(gPatterns[0]);

int16_t prevRotary = rotaryCount;

void updateLeds() {
  if (prevRotary != rotaryCount) {
    FastLED.clear();
    memset(dots, -1, sizeof(dots));
  }
  prevRotary = rotaryCount;
  gPatterns[rotaryCount / 2]();
  FastLED.show();
}

/* update rotary encoder */
void rotaryStateUpdate(void) {
  //todo encoder moves two spaces at a time, perhaps timebox state changes
  aState = digitalRead(OUTPUT_A); // Reads the "current" state of the outputA

  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    int bState = digitalRead(OUTPUT_B);

    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (bState != aState) {
      rotaryCount++;
      if (rotaryCount >= gPatternsSize * 2) rotaryCount = 0;
    } else {
      rotaryCount--;
      if (rotaryCount < 0) rotaryCount = gPatternsSize * 2 - 1;
    }

  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
}

void printArray(byte ray[], size_t len) {
  log_print("[ ");
  int i;
  for (i = 0; i < len; i++) {
    log_printf("%2d", ray[i]);
    if (i != 7) {
      log_print(", ");
    }
  }
  log_print(" ]. ");
}

/* Handle going from one state to the next. */
void advanceState() {
  updateLeds();

  printArray(input, 8);
  log_printf("Rotary: %2u. ", rotaryCount);
  log_printf("Tick: %3u. ", tickPr);
  log_println();
}

/* Check if a new tick has passed. */
bool updateTick() {
  unsigned long currentMs = millis();
  tickPr = currentMs - startMsPr;
  startMsPr = currentMs;
}


void onReq(int numBytes) {
  Wire.write(patternRawStatus[rotaryCount / 2]);
}

void setup() {
  Serial.begin(115200);
  delay(250);

  FastLED.addLeds <WS2812B, DATA_PIN, GRB> (leds, NUM_LEDS);

  // Setup input pins
  pinMode(OUTPUT_A, INPUT);
  pinMode(OUTPUT_B, INPUT);

  // Reads the initial state of rotary
  aLastState = digitalRead(OUTPUT_A);

  // start clock
  startMs = millis();

  //setup data sending from sound arduino
  Wire.begin(1);
  Wire.onReceive(onData);
  Wire.onRequest(onReq);

  // fill state array with -1s
  memset(dots, -1, sizeof(dots));
}

/* Main loop code */
void loop() {
  rotaryStateUpdate();
  updateTick();
  advanceState();
}
