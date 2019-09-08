#include "Pattern.h"
template <int SIZE>

Pattern<SIZE>::Pattern(CRGBArray<SIZE>* leds, int8_t* input) : _leds(leds), _input(input) {}
