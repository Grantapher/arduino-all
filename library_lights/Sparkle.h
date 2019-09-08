#pragma once
#include "Glisten.h"

template<int SIZE>
class Sparkle : public Glisten<SIZE> {
protected:
    CRGB getColor() const override { return CRGS::White; }
public:
    Confetti(CRGBArray<SIZE>* leds, int8_t* input) : Glisten<SIZE>(leds, input) {}
    void updateLeds(uint8_t threshold) : Glisten::updateLeds(threshold) {}
};
