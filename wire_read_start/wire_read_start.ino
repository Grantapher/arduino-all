#include <Wire.h>
#include <FastLED.h>

#define NUM_LEDS 100
#define DATA_PIN 2

#define OUTPUT_A 11
#define OUTPUT_B 10

#define ROTARY_MAX 50
#define ROTARY_MIN 1
#define ROTARY_START 20

#define TICK_MIN 5
#define TICK_MAX 500

#define DELTA_HUE 32

// I2C input data
byte input[8];

unsigned int tick = 0;

unsigned int rotaryCount = ROTARY_START;
int aState;
int aLastState;

unsigned long startMs;
unsigned long tickSize;
unsigned int tickState = -1;

char printBuf[16];

uint8_t prevHue = 0;

CRGBArray <NUM_LEDS> leds;
int hue = 0;
int pix = 0;

/* Main setup code. */
void setup() {
  Serial.begin(115200);
  delay(2000);

  FastLED.addLeds <WS2812B, DATA_PIN, GRB> (leds, NUM_LEDS);

  // Setup input pins
  pinMode(OUTPUT_A, INPUT);
  pinMode(OUTPUT_B, INPUT);

  // Reads the initial state of rotary
  aLastState = digitalRead(OUTPUT_A);

  // start clock
  startMs = millis();

  // initialize tick size
  updateTickSize(ROTARY_START);

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
    updateTickSize(rotaryCount);

  }
  aLastState = aState; // Updates the previous state of the outputA with the current state

}

/* Update the tick size from the rotary */
void updateTickSize(int rotary) {
  tickSize = 500 / (rotary + 1);
  // tickSize = map(rotary, ROTARY_MIN, ROTARY_MAX, TICK_MAX, TICK_MIN);
}

void updateLeds(int state) {
  // only check previous 20
  //    int startLed = state - 60;
  //    if (startLed < 0) startLed += NUM_LEDS;
  fill_rainbow(leds, NUM_LEDS, 0, DELTA_HUE);
  for (int i = 0; i < NUM_LEDS; i++) {
    //        int led = (startLed + i) % NUM_LEDS;
    //        leds[i] = leds[i].fadeToBlackBy(3);
    int brightness = map(input[i % 8], 0, 10, 0xFF, 0x00);
    leds[i] = leds[i].fadeToBlackBy(brighten8_video(brightness));
  }

  //  fill_rainbow(&(leds[state]), 1, prevHue, DELTA_HUE);
  //  prevHue += DELTA_HUE;
  //    leds[state] = CRGB::Orange;
  FastLED.show();
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
  updateLeds(tickState);

  printArray(input);

  sprintf(printBuf, "%2u", rotaryCount);
  Serial.print("Counter: ");
  Serial.print(printBuf);
  Serial.print(". ");

  sprintf(printBuf, "%3u", tick);
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
  tick = currentMs - startMs;
  startMs = currentMs;
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
