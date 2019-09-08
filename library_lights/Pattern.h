#pragma once

#include <FastLED.h>

template<int SIZE>
class Pattern {
private:
    const CRGBArray<SIZE>* _leds;
    const int8_t* _input;
protected:
    inline CRGBArray<SIZE>* get_leds() const { return _leds; }
    inline int8_t* get_input() const { return _input; }
public:
    Pattern(CRGBArray<SIZE>* leds, int8_t* input);
    virtual void updateLeds(uint8_t threshold);
};
