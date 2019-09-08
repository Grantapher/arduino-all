#include "Glisten.h"
#include "Constants.h"

template<int SIZE>
void Glisten<SIZE>::updateLeds(uint8_t threshold) {
    // random colored speckles that blink in and fade smoothly
    fadeToBlackBy(get_leds(), SIZE, map(threshold, 0, THRESHOLD_MAX, 1, 10));

    uint8_t timeThreshold = map(threshold, 0, THRESHOLD_MAX, 200, 8);
    uint64_t currentMs = millis();
    if (currentMs - _prevTime >= timeThreshold) {
        get_leds()[random16(SIZE)] += getColor();
        _prevTime = currentMs;
    }
}
