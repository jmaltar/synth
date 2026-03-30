#include "voice.h"

#include <cmath>

adsr_table_t adsr_arr{};
adsr_table_t adsr_arr_2{};

sample_t note_to_hz(int note) {
    return static_cast<sample_t>(440.0) * std::pow(static_cast<sample_t>(2.0), static_cast<sample_t>(note - 69) / static_cast<sample_t>(12.0));
}

bool valid_note(int n) {
    return n >= 0 && n < 128;
}
