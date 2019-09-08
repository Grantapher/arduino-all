#pragma once
#include "Pattern.h"

template<int SIZE>
class KickAndFlash : public Pattern<SIZE> {
public:
    KickAndFlash(CRGBArray<SIZE>* leds, int8_t* input) : Pattern<SIZE>(leds, input) {}
    void updateLeds(uint8_t threshold) override;
};
