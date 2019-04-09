#include <Wire.h>
#include <FastLED.h>

#define NUM_LEDS 80
#define DATA_PIN 2

#define OUTPUT_A 11
#define OUTPUT_B 10

#define ROTARY_MAX 50
#define ROTARY_MIN 1
#define ROTARY_START 20

#define TICK_MIN 5
#define TICK_MAX 500

#define DELTA_HUE 32
#define NUM_SPOKES 8
int SPOKE_LENGTH = NUM_LEDS / NUM_SPOKES;

// I2C input data
byte input[8];

unsigned int tickPr = 0;

unsigned int rotaryCount = ROTARY_START;
int aState;
int aLastState;

unsigned long startMs;
unsigned long startMsPr;

char printBuf[16];

uint8_t prevHue = 0;

CRGBArray <NUM_LEDS> leds;
int hue = 0;
int pix = 0;

/* Main setup code. */
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
}

void onData(int numBytes) {
  int i = 0;
  while (Wire.available()) {
    input[i++] = Wire.read();
  }
}

/* update rotary encoder */
void rotaryStateUpdate(void) {
  aState = digitalRead(OUTPUT_A); // Reads the "current" state of the outputA

  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    int bState = digitalRead(OUTPUT_B);

    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (bState != aState) {
      rotaryCount = min(ROTARY_MAX, rotaryCount + 1);
    } else {
      rotaryCount = max(ROTARY_MIN, rotaryCount - 1);
    }

  }
  aLastState = aState; // Updates the previous state of the outputA with the current state

}

int arrayAverage(int8_t ray[], uint8_t startIndex, uint8_t endIndex) {
  float sum = 0;
  for (int i = startIndex; i < endIndex; i++) {
    sum += ray[i];
  }
  return sum / (endIndex - startIndex);
}

void updateLeds() {
  //Kick();
  //Spaceship();
  //glitter();
  //juggle();
  //sinelon();
  //confetti();
  //WheelManual();
  //WheelAuto();
  //flashEQ();
  brightnessEQ();
  FastLED.show();
}

uint8_t setting1index = NUM_LEDS / 2;
uint8_t setting1counter = 0;
bool active = false;
void Kick() {
  //todo 
  int threshold = 3;

  fadeToBlackBy( leds, NUM_LEDS, 16);

  if (input[1] > threshold && !active) {
    active = true;
    setting1counter = 0;
  }

  if (setting1counter <= NUM_LEDS / 2 && active) {
    CRGB color;
    fill_rainbow(&color, 1, prevHue, DELTA_HUE);
    prevHue += DELTA_HUE;
    
    leds[getIndex(setting1index, NUM_LEDS, -setting1counter)] = color;
    leds[getIndex(setting1index, NUM_LEDS, +setting1counter)] = color;
    setting1counter++;
  } else {
    active = false;
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

void sinelon()
{
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
  //todo configure potentiometer to control threshold. perhaps add beat detection if possible?
  bool thresholdMet = arrayAverage(input, 0, 1) >= 8.7;

  if (!thresholdMet) {
    fadeToBlackBy(leds, NUM_LEDS, 16);
  } else {
    int startLed = Wheel_i * SPOKE_LENGTH;
    int endLed = startLed + SPOKE_LENGTH;
    fadeToBlackBy(leds, startLed, 16);
    fadeToBlackBy(&(leds[endLed]), NUM_LEDS - endLed, 16);
  }

  uint64_t currentMs = millis();
  if (thresholdMet && !Wheel_high && currentMs - startMs >= 100) {
    Wheel_high = true;

    Wheel_i = (Wheel_i + 1) % NUM_SPOKES;
    fill_rainbow(&(leds[Wheel_i * SPOKE_LENGTH]), SPOKE_LENGTH, 0, 255 / SPOKE_LENGTH);
    startMs = currentMs;
  }

  if (!thresholdMet && Wheel_high) {
    Wheel_high = false;
  }
}

uint8_t state = 0;

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

void flashEQ() {
  //todo fuck with the constants
  int8_t x, y, i;

  fadeToBlackBy( leds, NUM_LEDS, 64);

  double m = 1;

  CRGB color = CRGB::Orange; //getColor(wheelVal);

  for (x = 0; x < 8; x++) {
    switch (x) {
      case 0:
        y = map(input[0] * m, 7, 10, 0, SPOKE_LENGTH);
        break;
      case 1:
        y = map(input[1] * 0.9 * m, 1, 10, 0, SPOKE_LENGTH);
        break;
      case 2:
        y = map(min(input[2] * 4 * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 3:
        y = map(min(input[3] * 2 * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 4:
        y = map(min(input[4] * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 5:
        y = map(min(input[5] * 2 * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 6:
        y = map(min(input[6] * 2 * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 7:
        y = map(min(input[7] * 4 * m, 10), 0, 10, 0, SPOKE_LENGTH);
        break;
      case 8:
        y = map(input[x] * m, 0, 10, 0, SPOKE_LENGTH);
        break;
    }
    y = min(SPOKE_LENGTH, max(0, y));

    fill_rainbow(&(leds[x * SPOKE_LENGTH]), y, 0, DELTA_HUE);
  }
}

void brightnessEQ() {
  //todo brighten based on change from previous audio state?
  fill_rainbow(leds, NUM_LEDS, 0, DELTA_HUE);
  for (int i = 0; i < NUM_LEDS; i++) {
    int brightness = map(input[i % 8], 0, 10, 0xFF, 0x00);
    leds[i] = leds[i].fadeToBlackBy(brighten8_video(brightness));
  }
}

void printArray(byte ray[]) {
  Serial.print("[ ");
  int i;
  for (i = 0; i < 8; i++) {
    sprintf(printBuf, "%2d", ray[i]);
    Serial.print(printBuf);
    if (i != 7) {
      Serial.print(", ");
    }
  }
  Serial.print(" ]. ");
}

/* Handle going from one state to the next. */
void advanceState() {
  updateLeds();

  printArray(input);

  sprintf(printBuf, "%2u", rotaryCount);
  Serial.print("Counter: ");
  Serial.print(printBuf);
  Serial.print(". ");

  sprintf(printBuf, "%3u", tickPr);
  Serial.print("tick: ");
  Serial.print(printBuf);
  Serial.print(". ");

  Serial.println();
}

/* Check if a new tick has passed. */
bool updateTick() {
  unsigned long currentMs = millis();
  //  bool tickUpdate = currentMs - startMs > tickSize;
  //  if (tickUpdate) {
  tickPr = currentMs - startMsPr;
  startMsPr = currentMs;
  //  }
  //  return tickUpdate;
  return true;
}

/* Main loop code */
void loop() {
  rotaryStateUpdate();
  bool newTick = updateTick();
  if (newTick)
    advanceState();
}
