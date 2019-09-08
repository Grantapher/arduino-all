#include "KickAndFlash.h"

template<int SIZE>
void KickAndFlash<SIZE>::updateLeds(uint8_t threshold) {
    fadeToBlackBy(get_leds(), SIZE, 32);
    if (get_input()[0] > threshold) {
        fill_rainbow(get_leds(), SIZE, 0xE0, (255 / SIZE) + 1);
    }
}
