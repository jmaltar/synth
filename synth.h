#ifndef SYNTH_H
#define SYNTH_H

#include <algorithm>
#include <cstdint>
#include <array>
#include <cmath>
#include "voice.h"
#include "utils.h"

#include <limits>

template <typename V>
struct Synth {
    static constexpr size_t BL = V::block_length;

    std::array<sample_t, BL * 2> conversion_buffer{};

    std::array<V, n_voices> voices{};
    sample_t sr;

    sample_t current_norm{1.0};

    Synth(sample_t sample_rate);

    int policy();
    void note_off(int note);
    void note_on(int note, int velocity);

    void process_sample_by_sample(sample_t* out);
    void process_sample_by_sample(int16_t* out);

    void operator()(sample_t* out);
    void operator()(int16_t* out);


    void set_oscillator_1(Wavetable wavetable);
    void set_oscillator_up();
    void set_oscillator_down();
};

template <typename V>
Synth<V>::Synth(sample_t sample_rate):
    sr{sample_rate}
{
    voices[0].adsr_init();
}

template <typename V>
int Synth<V>::policy() {
    int best_free_index{-1};
    uint64_t best_free_started_at{std::numeric_limits<uint64_t>::max()};

    int best_release_index{-1};
    sample_t best_release_adsr_value{1.0};
    uint64_t best_release_started_at{std::numeric_limits<uint64_t>::max()};


    uint16_t voice_i{0};
    for (V& voice: voices) {
        bool is_assigned{voice.current_note != -1};
        bool at_idle{voice.adsr.idle};
        bool at_release{voice.adsr.in_release()};
        uint64_t started_at{voice.started_at};
        sample_t last_value{voice.adsr.current_level};

        if (!is_assigned || at_idle) {
            bool better_candidate{started_at < best_free_started_at};

            if (better_candidate)
            {
                best_free_index = voice_i;
                best_free_started_at = started_at;
            }

        } else if (at_release) {
            bool condition_1{last_value < best_release_adsr_value};
            bool condition_2{last_value == best_release_adsr_value && started_at < best_release_started_at};
            bool better_candidate{condition_1 || condition_2};

            if (better_candidate)
            {
                best_release_index = voice_i;
                best_release_adsr_value = last_value;
                best_release_started_at = started_at;
            }
        }

        ++voice_i;
    }

    if (best_free_index != -1)
        return best_free_index;
    else if (best_release_index != -1)
        return best_release_index;
    else return -1;
}

template <typename V>
void Synth<V>::note_on(int note, int velocity) {
    int voice_i{policy()};
    if (voice_i != -1)
        voices[voice_i].assign(note, velocity);
}

template <typename V>
void Synth<V>::note_off(int note) {
    uint16_t voice_i{0};
    for (V& voice: voices) {
        if (voice.current_note == note)
            voice.adsr_off();
        else if (voice.POR_note == note) {
            // std::cout << "POR OFF\n";
            voice.por_off();
        }

        ++voice_i;
    }
}

template <typename V>
void Synth<V>::set_oscillator_1(Wavetable wavetable) {
    for (V& voice: voices)
        voice.oscillator.set_table(wavetable);
}

template <typename V>
void Synth<V>::set_oscillator_up() {
    uint8_t i{static_cast<uint8_t>(voices[0].oscillator.wavetable)};

    if (--i == 0)
        i = 5;

    Wavetable wavetable_new{static_cast<Wavetable>(i)};
    set_oscillator_1(wavetable_new);
}

template <typename V>
void Synth<V>::set_oscillator_down() {
    uint8_t i{static_cast<uint8_t>(voices[0].oscillator.wavetable)};

    if (++i == 6)
        i = 1;

    Wavetable wavetable_new{static_cast<Wavetable>(i)};
    set_oscillator_1(wavetable_new);
}

template <typename V>
void Synth<V>::process_sample_by_sample(sample_t* out) {

    const sample_t master_gain = 1.0;
    const sample_t alpha = 0.002;

    for (uint32_t sample_i{0}; sample_i < BL; ++sample_i) {

        sample_t s{0.0};
        sample_t target_norm {1.0};
        int n_active_voices = 0;

        for (V& voice: voices) {
            if (!voice.should_be_evaluated())
                continue;

            s += voice.process_single();
            ++n_active_voices;

            if (voice.adsr.idle) {
                voice.current_note = -1;
            }
        }

        if (n_active_voices > 1)
            target_norm /= std::sqrt(static_cast<sample_t>(n_active_voices));

        // ---- 2) smooth norm toward target_norm (per sample) ----
        current_norm += (target_norm - current_norm) * alpha;

        // ---- 3) apply smoothed norm and master gain ----
        sample_t sample_out = s * current_norm * master_gain;

        // write stereo
        out[2 * sample_i + 0] = sample_out;
        out[2 * sample_i + 1] = sample_out;
    }
}

template <typename V>
void Synth<V>::process_sample_by_sample(int16_t* out) {

    const sample_t master_gain = 1.0;
    const sample_t alpha = 0.002;

    for (uint32_t sample_i{0}; sample_i < BL; ++sample_i) {
        sample_t s{0.0};
        sample_t target_norm {1.0};
        int n_active_voices = 0;

        for (V& voice: voices) {
            if (!voice.should_be_evaluated()) continue;

            s += voice();
            ++n_active_voices;

            if (voice.adsr.idle) {
                voice.current_note = -1;
            }
        }

        if (n_active_voices > 1)
            target_norm /= std::sqrt(static_cast<sample_t>(n_active_voices));

        // ---- 2) smooth norm toward target_norm (per sample) ----
        current_norm += (target_norm - current_norm) * alpha;

        // ---- 3) apply smoothed norm and master gain ----
        sample_t sample_out = s * current_norm * master_gain;

        out[2 * sample_i + 0] = sample_t_to_s16(sample_out);
        out[2 * sample_i + 1] = sample_t_to_s16(sample_out);
    }
}

template <typename V>
void Synth<V>::operator()(sample_t* out) {
    /*
    const sample_t master_gain = 1.0;
    const sample_t alpha = 0.002;
    sample_t target_norm {1.0};
    int n_active_voices = 0;
    */

    std::fill(out, out + BL * 2, 0.0);

    for (V& voice: voices) {
        if (!voice.should_be_evaluated())
            continue;

        voice();

        for (uint32_t sample_i{0}; sample_i < BL; ++sample_i) {
            out[2 * sample_i] += voice.buffer[2 * sample_i];
            out[2 * sample_i + 1] += voice.buffer[2 * sample_i + 1];
        }

        if (voice.adsr.idle) {
            voice.current_note = -1;
        }

    }
}

template <typename V>
void Synth<V>::operator()(int16_t* out) {
    this->operator()(conversion_buffer.data());
    sample_t_array_to_s16_array(conversion_buffer.data(), out, BL * 2);
}



#endif