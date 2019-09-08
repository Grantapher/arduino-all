#include "KickAndFlash.h"


template <int SIZE>
KickAndFlash<SIZE>::KickAndFlash(CRGBArray<SIZE>* leds, int8_t* input) : Pattern<SIZE>(leds, input) {}

template <int SIZE>
void KickAndFlash<SIZE>::updateLeds(uint8_t threshold) {
    fadeToBlackBy(m_leds, NUM_LEDS, 32);
    if (m_input[0] > threshold) {
        fill_rainbow(leds, NUM_LEDS, 0xE0, (255 / NUM_LEDS) + 1);
    }
}
