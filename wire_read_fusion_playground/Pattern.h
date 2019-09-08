#pragma once
template<int SIZE>
class Pattern {
protected:
    const CRGBArray<SIZE>* leds;
    const int8_t* input;
public:
    Pattern(CRGBArray<SIZE>* leds, int8_t* input);
    virtual void updateLeds(uint8_t threshold);
};
