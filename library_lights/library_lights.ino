#include <Wire.h>
#include <FastLED.h>
#include "Constants.h"

//we don't care about speed
#define ENABLE_SPEED 0
#define ENABLE_PULLUPS 0
#include <MD_REncoder.h>

// Log all to Serial, comment this line to disable logging
//#define LOG Serial
// Include must be placed after LOG definition to work
#include "log.h"

#define NUM_LEDS 300
#define DATA_PIN 2

#define MAX_VOLTS 5
#define MAX_MILLIAMPS 2400
#define MAX_BRIGHTNESS 0x60

#define FUNCTION_ROTARY_INPUT_BTN 5
#define FUNCTION_ROTARY_INPUT_A 6
#define FUNCTION_ROTARY_INPUT_B 7
#define FUNCTION_ROTARY_START 9

#define THRESHOLD_ROTARY_INPUT_BTN 8
#define THRESHOLD_ROTARY_INPUT_A 9
#define THRESHOLD_ROTARY_INPUT_B 10
#define THRESHOLD_ROTARY_START 9

#define TICK_MIN 5
#define TICK_MAX 500

#define DELTA_HUE 32
#define NUM_SPOKES 20

#define BIT_BUFFER_SIZE ((NUM_LEDS / 16) + 1)
#define MOD_THRESHOLD 8

#define NUM_SHIPS 20
uint8_t SHIP_LENGTH = NUM_LEDS / NUM_SHIPS;

uint8_t SPOKE_LENGTH = NUM_LEDS / NUM_SPOKES;

// I2C input data
int8_t input[8];

uint16_t tickPr = 0;

uint64_t startMs;
uint64_t startMsPr;

uint8_t prevHue = 0;

uint8_t dotsBitBuffer[BIT_BUFFER_SIZE];

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

uint8_t colorIndex = 0;
void KickAndRun() {
    fadeToBlackBy(leds, NUM_LEDS, functionIndex % 2 ? 255 : 128);

    uint16_t i;

    //increment active dots by rotating them all over
    uint8_t carry = 0;
    for (i = 0; i < BIT_BUFFER_SIZE; i++) {
        uint8_t nextCarry = dotsBitBuffer[i] & 0x01;
        dotsBitBuffer[i] >>= 1;
        if (carry) dotsBitBuffer[i] |= 0x80;
        carry = nextCarry;
    }

    //create new dot if necessary, increment starting color
    if (input[0] > threshold) {
        dotsBitBuffer[0] |= 0x80;
        colorIndex = (colorIndex + 1) % MOD_THRESHOLD;
    }

    //fill all existing dots
    CRGB color;
    uint16_t colorsSeen = 0;
    uint16_t maxDots = NUM_LEDS / 2;
    for (i = 0; i < maxDots; i++) {
        if (dotsBitBuffer[i / 8] & (0x80 >> i % 8)) {
            fill_rainbow(&color, 1, map((colorIndex + colorsSeen) % MOD_THRESHOLD, 0, MOD_THRESHOLD, 0, 255), DELTA_HUE);
            colorsSeen = (colorsSeen + MOD_THRESHOLD - 1) % MOD_THRESHOLD;
            leds[getIndex(maxDots, NUM_LEDS, -i)] |= color;
            leds[getIndex(maxDots, NUM_LEDS, +i)] |= color;
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

uint16_t prevPos = beatsin16(0, 0, SHIP_LENGTH * 2);
void Spaceship() {
    fadeToBlackBy(leds, NUM_LEDS, map(threshold, 0, THRESHOLD_MAX, 16, 80));

    uint16_t pos = beatsin16(map(threshold, 0, THRESHOLD_MAX, 15, 70), 0, SHIP_LENGTH * 2);
    int16_t diff = pos - prevPos;
    for (int16_t spoke = -1; spoke < NUM_SHIPS; spoke++) {
        if (diff == 0) diff = 1;

        int16_t index;
        if (diff >= 0) {
            index = prevPos + spoke * SHIP_LENGTH;
            if (index != constrain(index, 0, NUM_LEDS - 1)) continue;
            fill_rainbow(&(leds[index]), diff, prevHue, DELTA_HUE);
        } else {
            index = pos + spoke * SHIP_LENGTH;
            if (index != constrain(index, 0, NUM_LEDS - 1)) continue;
            fill_rainbow(&(leds[index]), -diff, prevHue + -diff * DELTA_HUE, DELTA_HUE);
        }
    }

    if (prevPos != pos) {
        prevPos = pos;
        prevHue += (diff < 0 ? -diff : diff) * DELTA_HUE; // depends on uint8_t overflows
    }

}

void glitter() {
    // random colored speckles that blink in and fade smoothly
    fadeToBlackBy(leds, NUM_LEDS, map(threshold, 0, THRESHOLD_MAX, 1, 10));

    uint8_t timeThreshold = map(threshold, 0, THRESHOLD_MAX, 200, 8);
    uint64_t currentMs = millis();
    if (currentMs - startMs >= timeThreshold) {
        if (functionIndex % 2) {
            leds[random16(NUM_LEDS)] += CRGB::White;
        } else {
            leds[random16(NUM_LEDS)] += CHSV(random8(255), 200, 255);
        }
        startMs = currentMs;
    }
}

uint16_t prevJugglePos[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
void juggle() {
    // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy(leds, NUM_LEDS, map(threshold, 0, THRESHOLD_MAX, 64, 168));

    byte dothue = 0;
    uint8_t speed = map(threshold, 0, THRESHOLD_MAX, 3, 8);
    uint8_t dotSpeedDiff = map(threshold, 0, THRESHOLD_MAX, 1, 5);
    for (int i = 0; i < 8; i++) {
        //get color for dot
        CHSV color = CHSV(dothue, 200, 255);
        dothue += 32;

        uint16_t pos = beatsin16(speed + i * dotSpeedDiff, 0, NUM_LEDS);
        int16_t diff = pos - prevJugglePos[i];
        int j;
        if (diff > 0) {
            for (j = 0; j < diff; j++) {
                leds[prevJugglePos[i] + j] |= color;
            }
        } else if (diff <= 0) {
            for (j = 0; j < -diff; j++) {
                leds[pos + j] |= color;
            }
        }

        prevJugglePos[i] = pos;

    }
}

uint16_t prevSinPos = 0;
void sinelon() {
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy(leds, NUM_LEDS, map(threshold, 0, THRESHOLD_MAX, 4, 84));
    uint16_t pos = beatsin16(map(threshold, 0, THRESHOLD_MAX, 6, 66), 0, NUM_LEDS);

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

uint8_t Wheel_i = 0;
void WheelAuto() {
    //todo perhaps add beat detection if possible?
    fadeToBlackBy(leds, NUM_LEDS, 16);
    if (input[0] > threshold) {
        Wheel_i = (Wheel_i + 1) % NUM_SPOKES;
        uint16_t startIndex = Wheel_i * SPOKE_LENGTH + Wheel_i;

        if (functionIndex % 2) {
            CRGB color;
            fill_rainbow(&color, 1, map(Wheel_i, 0, NUM_SPOKES, 0, 255), 0);
            fill_solid(&(leds[startIndex]), SPOKE_LENGTH + 1, color);
        } else {
            fill_rainbow(&(leds[startIndex]), SPOKE_LENGTH + 1, 0, 255 / SPOKE_LENGTH);
        }
    }
}

uint16_t state = 0;
void WheelManual() {
    fadeToBlackBy(leds, NUM_LEDS, map(threshold, 0, THRESHOLD_MAX, 2, 24));

    uint8_t timeThreshold = map(threshold, 0, 9, 40, 10);
    uint64_t currentMs = millis();
    if (currentMs - startMs >= timeThreshold) {
        fill_rainbow(&(leds[state]), 1, prevHue, DELTA_HUE);
        state = (state + 1) % NUM_LEDS;
        startMs = currentMs;
        prevHue += DELTA_HUE;
    }
}

void EQ() {
    fadeToBlackBy(leds, NUM_LEDS, 0x28);
    for (size_t x = 0; x < 8; x++) {
        fill_rainbow(&(leds[x * SPOKE_LENGTH]), map(input[x], 0, 10, 0, SPOKE_LENGTH), 0, DELTA_HUE);
    }
}

typedef void LedFunction(void);
LedFunction* gPatterns[] = { KickAndRun, KickAndRun, KickFlash, Spaceship, glitter, glitter, juggle, sinelon,
WheelManual, WheelAuto, WheelAuto, EQ };
bool patternRawStatus[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
uint8_t gPatternsSize = sizeof(gPatterns) / sizeof(gPatterns[0]);

LedFunction* prevFunction = gPatterns[FUNCTION_ROTARY_START];
void updateLeds() {
    LedFunction* currentFunction = gPatterns[functionIndex];
    if (prevFunction != currentFunction) {
        memset(dotsBitBuffer, 0, sizeof(dotsBitBuffer));
    }
    prevFunction = currentFunction;
    currentFunction();
    FastLED.show();
}

uint8_t updateRotaryState(MD_REncoder * rotary, int8_t rotaryCount, uint8_t upperBound) {
    uint8_t rotaryState = rotary->read();
    if (rotaryState) {
        rotaryState != DIR_CW ? rotaryCount-- : rotaryCount++;
        if (rotaryCount < 0) rotaryCount = upperBound - 1;
        rotaryCount = rotaryCount % upperBound;
    }
    return rotaryCount;
}

void updateRotaries() {
    functionIndex = updateRotaryState(&functionRotary, functionIndex, gPatternsSize);
    threshold = updateRotaryState(&thresholdRotary, threshold, THRESHOLD_MAX);
}

#ifdef LOG
void printArray(int8_t ray[], size_t len) {
    log_print("[ ");
    uint8_t i;
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
    return true;
}

#else
void printArray(int8_t ray[], size_t len) {}
bool updateTick() {}
#endif

void onReq() {
    Wire.write(patternRawStatus[functionIndex]);
}

void setup() {
#ifdef LOG
    Serial.begin(115200);
#endif 

    delay(500);

    FastLED.addLeds <WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setMaxPowerInVoltsAndMilliamps(MAX_VOLTS, MAX_MILLIAMPS);
    FastLED.setBrightness(MAX_BRIGHTNESS);

    // rotary encoder starts
    functionRotary.begin();
    thresholdRotary.begin();

    // buttons input setup
    pinMode(FUNCTION_ROTARY_INPUT_BTN, INPUT_PULLUP);
    pinMode(THRESHOLD_ROTARY_INPUT_BTN, INPUT_PULLUP);

    // start clock
    startMs = millis();

    //setup data sending from sound arduino
    Wire.begin(1);
    Wire.onReceive(onData);
    Wire.onRequest(onReq);

    // start dotBitBuffer with zeroes
    memset(dotsBitBuffer, 0, sizeof(dotsBitBuffer));
}

/* Main loop code */
void loop() {
    updateRotaries();
    updateTick();
    if (digitalRead(FUNCTION_ROTARY_INPUT_BTN) == LOW) {
        log_printf("function held");
        fadeToBlackBy(leds, NUM_LEDS, 64);
        fill_rainbow(leds, (functionIndex + 1) * NUM_LEDS / gPatternsSize, 0, 255 / gPatternsSize);
        FastLED.show();
    } else if (digitalRead(THRESHOLD_ROTARY_INPUT_BTN) == LOW) {
        log_printf("threshold held");
        fadeToBlackBy(leds, NUM_LEDS, 64);
        fill_rainbow(leds, (threshold + 1) * NUM_LEDS / THRESHOLD_MAX, 0, 255 / THRESHOLD_MAX);
        FastLED.show();
    } else {
        updateLeds();

        printArray(input, 8);
        log_printf("threshold: %2u. ", threshold);
        log_printf("functionIndex: %2u. ", functionIndex);
        log_printf("Tick: %3u. ", tickPr);

    }
    log_println();
}
