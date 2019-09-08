#pragma once
#include "Glisten.h"

template<int SIZE>
class Confetti : public Glisten<SIZE> {
protected:
    inline CRGB getColor() override { return CRGB(CHSV(random8(255), 200, 255)); }
public:
    Confetti(CRGBArray<SIZE>* leds, int8_t* input) : Glisten<SIZE>(leds, input) {}
    void updateLeds(uint8_t threshold) : Glisten::updateLeds(threshold) {}
};

