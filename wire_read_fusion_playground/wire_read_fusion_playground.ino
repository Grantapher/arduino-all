#include <Wire.h>
#include <FastLED.h>
#include <MD_REncoder.h>
// Log all to Serial, comment this line to disable logging
#define LOG Serial
// Include must be placed after LOG definition to work
#include "log.h"

#define NUM_LEDS 250
#define DATA_PIN 2

#define FUNCTION_ROTARY_INPUT_BTN 5
#define FUNCTION_ROTARY_INPUT_A 6
#define FUNCTION_ROTARY_INPUT_B 7
#define FUNCTION_ROTARY_START 0

#define THRESHOLD_ROTARY_INPUT_BTN 8
#define THRESHOLD_ROTARY_INPUT_A 9
#define THRESHOLD_ROTARY_INPUT_B 10
#define THRESHOLD_ROTARY_START 2
#define THRESHOLD_MAX 10

#define TICK_MIN 5
#define TICK_MAX 500

#define DELTA_HUE 32
#define NUM_SPOKES 8

#define BRIGHTNESS 0x80

uint8_t SPOKE_LENGTH = NUM_LEDS / NUM_SPOKES;

// I2C input data
int8_t input[8];

uint16_t tickPr = 0;

uint64_t startMs;
uint64_t startMsPr;

uint8_t prevHue = 0;

uint16_t maxDots = NUM_LEDS / 2;
int16_t dots[NUM_LEDS / 2];


CRGBArray <NUM_LEDS> leds;

MD_REncoder functionRotary = MD_REncoder(FUNCTION_ROTARY_INPUT_A, FUNCTION_ROTARY_INPUT_B);
int8_t functionIndex = FUNCTION_ROTARY_START;

MD_REncoder thresholdRotary = MD_REncoder(THRESHOLD_ROTARY_INPUT_A, THRESHOLD_ROTARY_INPUT_B);
uint8_t threshold = THRESHOLD_ROTARY_START;

void onData(int numBytes) {
  int i = 0;
  while (Wire.available()) {
    input[i++] = Wire.read();
  }
}

void KickFlash() {
  fadeToBlackBy(leds, NUM_LEDS, 32);
  if (input[0] > threshold) {
    fill_rainbow(leds, NUM_LEDS, 0xE0, (255 / NUM_LEDS) + 1);
  }
}

uint16_t dotIndex = 0;
uint8_t modThreshold = 8;
void KickAndRun() {
  fadeToBlackBy( leds, NUM_LEDS, functionIndex ? 255 : 128);

  uint16_t i;
  //increment active dots
  for (i = 0; i < maxDots; i++) {
    if (dots[i] >= 0) dots[i]++;
    if (dots[i] > maxDots) dots[i] = -1;
  }

  //create new dot if necessary
  if (input[0] > threshold) {
    dots[dotIndex] = 0;
    dotIndex = (dotIndex + 1) % maxDots;
  }

  //fill all existing dots
  CRGB color;
  uint16_t ledIndex;
  for (i = 0; i < maxDots; i++) {
    if (dots[i] >= 0) {
      ledIndex = dots[i];
      fill_rainbow(&color, 1, map(i % modThreshold, 0, modThreshold, 0, 255), DELTA_HUE);

      leds[getIndex(maxDots, NUM_LEDS, -ledIndex)] |= color;
      leds[getIndex(maxDots, NUM_LEDS, +ledIndex)] |= color;
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
  //todo consider thresholding logic to speed up the dots
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
  //todo copy glitter's threshold logic
  fadeToBlackBy( leds, NUM_LEDS, 3);
  if ( random8() < 20) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void juggle() {
  //todo also needs previous position taken into account like sinelon
  //todo have threshold controll speed like sinelon  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

uint16_t prevSinPos = 0;
void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, threshold * 8 + 4);
  uint16_t pos = beatsin16(threshold * 6 + 6, 0, NUM_LEDS);

  // positive means increasing, negative means decreasing
  int16_t diff = pos - prevSinPos;
  if (diff == 0) return;

  if (diff > 0) {
    fill_rainbow(&(leds[prevSinPos]), diff, prevHue, 2);
    prevHue += 2 * diff;
  } else {
    diff = -diff;
    fill_rainbow(&(leds[pos]), diff, prevHue + 2 * diff, -2);
    prevHue += 2 * diff;
  }

  prevSinPos = pos;
}


void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, THRESHOLD_MAX + 1 - threshold);

  uint8_t timeThreshold = threshold * 10 + 10;
  uint64_t currentMs = millis();
  if (currentMs - startMs >= timeThreshold) {
    int pos = random16(NUM_LEDS);
    leds[pos] += CHSV(random8(255), 200, 255);
    startMs = currentMs;
  }
}

uint8_t Wheel_i = 0;
void WheelAuto() {
  //todo consider changing the color to do the rainbow in segments rather than full rainbow in each segment
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
  if (thresholdMet) {
    Wheel_i = (Wheel_i + 1) % NUM_SPOKES;
    fill_rainbow(&(leds[Wheel_i * SPOKE_LENGTH]), SPOKE_LENGTH, 0, 255 / SPOKE_LENGTH);
    startMs = currentMs;
  }
}

uint16_t state = 0;
void WheelManual() {
  fadeToBlackBy(leds, NUM_LEDS, (THRESHOLD_MAX * 2) + 1 - (threshold * 2));

  uint8_t timeThreshold = threshold * 5 + 5;
  uint64_t currentMs = millis();
  if (currentMs - startMs >= timeThreshold) {
    fill_rainbow(&(leds[state]), 1, prevHue, DELTA_HUE);
    state = (state + 1) % NUM_LEDS;
    startMs = currentMs;
    prevHue += DELTA_HUE;
  }
}

void EQ() {
  fadeToBlackBy( leds, NUM_LEDS, 0x28);
  for (size_t x = 0; x < 8; x++) {
    fill_rainbow(&(leds[x * SPOKE_LENGTH]), map(input[x], 0, 10, 0, SPOKE_LENGTH), 0, DELTA_HUE);
  }
}

typedef void (*LedFunctionArray[])(void);
LedFunctionArray gPatterns = { KickAndRun, KickAndRun, KickFlash, Spaceship, glitter, juggle, sinelon, confetti, WheelManual, WheelAuto, EQ };
bool patternRawStatus[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
uint8_t gPatternsSize = sizeof(gPatterns) / sizeof(gPatterns[0]);

uint8_t prevFunctionIndex = FUNCTION_ROTARY_START;
void updateLeds() {
  if (prevFunctionIndex != functionIndex) {
    FastLED.clear();
    memset(dots, -1, sizeof(dots));
  }
  prevFunctionIndex = functionIndex;
  gPatterns[functionIndex]();
  FastLED.show();
}

uint8_t updateRotaryState(MD_REncoder* rotary, int8_t rotaryCount, uint8_t upperBound) {
  uint8_t rotaryState = rotary->read();
  if (rotaryState) {
    rotaryState != DIR_CW ? rotaryCount++ : rotaryCount--;
    if (rotaryCount < 0) rotaryCount = upperBound - 1;
    rotaryCount = rotaryCount % upperBound;
  }
  return rotaryCount;
}

void updateRotaries() {
    functionIndex = updateRotaryState(&functionRotary, functionIndex, gPatternsSize);
    threshold = updateRotaryState(&thresholdRotary, threshold, THRESHOLD_MAX);
}

void printArray(int8_t ray[], size_t len) {
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

/* Check if a new tick has passed. */
bool updateTick() {
  unsigned long currentMs = millis();
  tickPr = currentMs - startMsPr;
  startMsPr = currentMs;
}

void onReq() {
  Wire.write(patternRawStatus[functionIndex]);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  FastLED.addLeds <WS2812B, DATA_PIN, GRB> (leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // rotary encoder starts
  functionRotary.begin();
  thresholdRotary.begin();

  // buttons input setup
  pinMode(FUNCTION_ROTARY_INPUT_BTN, INPUT);
  pinMode(THRESHOLD_ROTARY_INPUT_BTN, INPUT);

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
  updateRotaries();
  updateTick();
  if (digitalRead(FUNCTION_ROTARY_INPUT_BTN) == LOW) {
      FastLED.clear();
      fill_rainbow(leds, functionIndex + 1, 0, 255 / gPatternsSize);
      FastLED.show();
  } else if (digitalRead(THRESHOLD_ROTARY_INPUT_BTN) == LOW) {
      FastLED.clear();
      fill_rainbow(leds, threshold + 1, 0, 255 / THRESHOLD_MAX);
      FastLED.show();
  } else {
      updateLeds();

      printArray(input, 8);
      log_printf("threshold: %2u. ", threshold);
      log_printf("functionIndex: %2u. ", functionIndex);
      log_printf("Tick: %3u. ", tickPr);
      log_println();
  }
}
