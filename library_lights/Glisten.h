#pragma once
#include "Pattern.h"

template<int SIZE>
class Glisten : public Pattern<SIZE> {
private:
    uint64_t _prevTime;
protected:
    virtual CRGB getColor(uint8_t threshold);
public:
    Glisten(CRGBArray<SIZE>* leds, int8_t* input) : Pattern<SIZE>(leds, input), _prevTime(millis()) {}
    void updateLeds(uint8_t threshold) override;
};
