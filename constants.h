#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstdint>
#include <cmath>
#include <array>

#ifdef SYNTH_SAMPLE_USE_DOUBLE
using sample_t = double;
#else
using sample_t = float;
#endif

const sample_t two_pi = static_cast<sample_t>(2.0 * M_PI);

const sample_t sample_rate{48000.0};

const uint32_t buffer_length_div_4{16};   // mono samples per synth block
const uint32_t buffer_length_div_2{buffer_length_div_4 * 2};
const uint32_t buffer_length{buffer_length_div_4 * 4};

const uint32_t table_length{1024};
const sample_t table_length_st{static_cast<sample_t>(table_length)};
const sample_t phase_to_index{table_length_st / two_pi};

#ifndef SYNTH_N_VOICES
#define SYNTH_N_VOICES 6
#endif

const uint8_t n_voices{SYNTH_N_VOICES};

#ifdef SYNTH_HEADROOM
const sample_t headroom{SYNTH_HEADROOM};
#else
const sample_t headroom{1.0 / static_cast<sample_t>(n_voices)};
#endif

#define ADSR_CORE_RATE 100
const std::size_t adsr_max_length{2048};
const std::size_t adsr_target_length{1024};
const std::size_t adsr_por_length{512};
using adsr_table_t = std::array<sample_t, adsr_target_length + 1>;

using adsr_por_table_t = std::array<sample_t, adsr_por_length>;

// +2 for linear interpolation
using table_t = std::array<sample_t, table_length + 2>;

enum class Wavetable {
    undefined, sine, square, saw_incr, saw_decr, triangle
};

#endif
