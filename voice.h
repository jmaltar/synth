#ifndef VOICE_H
#define VOICE_H

#include <array>
#include <cstdint>

#include "constants.h"
#include "oscillator.h"
#include "adsr.h"
#include "utils.h"

extern adsr_table_t adsr_arr;
extern adsr_table_t adsr_arr_2;

sample_t note_to_hz(int note);
bool valid_note(int n);

template <size_t BL>
struct Voice {
    static constexpr size_t block_length = BL;

    std::array<sample_t, BL * 2> buffer{};
    ADSR_LI<adsr_target_length, adsr_max_length> adsr{&adsr_arr, 0.1, 0.1, 0.8, 1.5, sample_rate};

    sample_t freq{0.0};
    sample_t gain{0.0};

    int current_note{-1};
    uint64_t started_at{0};

    int POR_note{-1}, POR_velocity{0};

    Oscillator oscillator{sample_rate};

    bool should_be_evaluated() const;
    virtual void set_frequency(sample_t freq) = 0;
    virtual void adsr_on() = 0;
    virtual void adsr_off() = 0;
    void por_off();
    virtual void adsr_init() = 0;
    void assign(int note, int velocity);
    void handle_por();

    virtual sample_t process_single() = 0;
    virtual void operator()() = 0;
};

template <size_t BL>
struct SimpleVoice: Voice<BL> {
    void adsr_on();
    void adsr_off();
    void por_off();
    void set_frequency(sample_t freq);
    sample_t process_single();
    void operator()();
    void adsr_init();
};

template <size_t BL>
struct FMVoice : Voice<BL> {
    sample_t ratio{1.0};

    sample_t freq_modulator{0.0};
    sample_t gain_modulator{1.0};

    sample_t fm_index{1.0};

    ADSR_LI<adsr_target_length, adsr_max_length> adsr_modulator{&adsr_arr_2, 0.2, 0.5, 0.7, 0.3, sample_rate};

    std::array<sample_t, BL * 2> modulator_buffer{};
    Oscillator modulator{sample_rate, 0.0, Wavetable::sine};

    void adsr_on();
    void adsr_off();
    void por_off();
    void adsr_init();
    void set_frequency(sample_t f);
    sample_t process_single();
    void operator()();
};

// ============ Voice<BL> implementations ============

template <size_t BL>
inline bool Voice<BL>::should_be_evaluated() const {
    return (current_note != -1) && freq > 0.0 && gain > 0.0;
}

template <size_t BL>
void Voice<BL>::por_off() {
    POR_note = -1;
    POR_velocity = 0;
    adsr.por_off();
}

template <size_t BL>
void Voice<BL>::assign(int note, int velocity) {
    bool por{adsr.in_release()};
    if (!por) {
        current_note = note;
        set_frequency(note_to_hz(note));
        gain = velocity / 127.0 * headroom;
        started_at = ticks_by_far();
        adsr_on();
    } else {
        POR_note = note;
        POR_velocity = velocity;
        adsr.por_on();
    }
}

template <size_t BL>
void Voice<BL>::handle_por() {
    if (adsr.por && adsr.idle) {
        assign(POR_note, POR_velocity);
    }
}

// ============ SimpleVoice<BL> implementations ============

template <size_t BL>
void SimpleVoice<BL>::adsr_on() {
    Voice<BL>::adsr.gate_on(Voice<BL>::adsr.current_level);
}

template <size_t BL>
void SimpleVoice<BL>::adsr_off() {
    Voice<BL>::adsr.gate_off();
}

template <size_t BL>
void SimpleVoice<BL>::por_off() {
    Voice<BL>::POR_note = -1;
    Voice<BL>::POR_velocity = 0;
    Voice<BL>::adsr.por_off();
}

template <size_t BL>
void SimpleVoice<BL>::set_frequency(sample_t freq) {
    this->freq = freq;
}

template <size_t BL>
sample_t SimpleVoice<BL>::process_single() {
    sample_t env = Voice<BL>::adsr();
    sample_t to_return{env * Voice<BL>::oscillator(Voice<BL>::freq, Voice<BL>::gain)};
    this->handle_por();
    return to_return;
}

template <size_t BL>
void SimpleVoice<BL>::operator()() {
    Voice<BL>::oscillator(Voice<BL>::buffer.data(), BL, Voice<BL>::freq, Voice<BL>::gain);
    Voice<BL>::adsr(Voice<BL>::buffer.data(), BL);
    this->handle_por();
}

template <size_t BL>
void SimpleVoice<BL>::adsr_init() {
    Voice<BL>::adsr.evaluate_array();
}

// ============ FMVoice<BL> implementations ============

template <size_t BL>
void FMVoice<BL>::adsr_on() {
    Voice<BL>::adsr.gate_on(Voice<BL>::adsr.current_level);
    adsr_modulator.gate_on(adsr_modulator.current_level);
}

template <size_t BL>
void FMVoice<BL>::adsr_off() {
    Voice<BL>::adsr.gate_off();
    adsr_modulator.gate_off();
}

template <size_t BL>
void FMVoice<BL>::por_off() {
    Voice<BL>::POR_note = -1;
    Voice<BL>::POR_velocity = 0;
    Voice<BL>::adsr.por_off();
}

template <size_t BL>
void FMVoice<BL>::adsr_init() {
    Voice<BL>::adsr.evaluate_array();
    adsr_modulator.evaluate_array();
}

template <size_t BL>
void FMVoice<BL>::set_frequency(sample_t f) {
    this->freq = f;
    freq_modulator = ratio * f;
}

template <size_t BL>
sample_t FMVoice<BL>::process_single() {
    sample_t env = Voice<BL>::adsr();
    sample_t env_mod = adsr_modulator();

    sample_t mod = modulator(freq_modulator, gain_modulator);

    sample_t I = fm_index * env_mod;
    sample_t df = I * freq_modulator;
    sample_t inst_freq = Voice<BL>::freq + df * mod;

    const sample_t nyq = sample_rate * 0.49;
    if (inst_freq < 0.0f) inst_freq = 0.0f;
    if (inst_freq > nyq) inst_freq = nyq;

    sample_t to_return{env * Voice<BL>::oscillator(inst_freq, Voice<BL>::gain)};
    this->handle_por();
    return to_return;
}

template <size_t BL>
void FMVoice<BL>::operator()() {
    const sample_t nyq = sample_rate * sample_t{0.49};
    const sample_t fm_index_times_freq_modulator = fm_index * freq_modulator;

    modulator(modulator_buffer.data(), BL, freq_modulator, gain_modulator);
    adsr_modulator(modulator_buffer.data(), BL);

    for (uint32_t sample_i{0}; sample_i < BL; ++sample_i) {
        modulator_buffer[2 * sample_i] *= fm_index_times_freq_modulator;
        modulator_buffer[2 * sample_i + 1] *= fm_index_times_freq_modulator;

        modulator_buffer[2 * sample_i] += Voice<BL>::freq;
        modulator_buffer[2 * sample_i + 1] += Voice<BL>::freq;

        if (modulator_buffer[2 * sample_i] < 0.0) {
            modulator_buffer[2 * sample_i] = 0.0;
        }

        if (modulator_buffer[2 * sample_i + 1] < 0.0) {
            modulator_buffer[2 * sample_i + 1] = 0.0;
        }

        if (modulator_buffer[2 * sample_i] > nyq)
            modulator_buffer[2 * sample_i] = nyq;

        if (modulator_buffer[2 * sample_i + 1] > nyq)
            modulator_buffer[2 * sample_i + 1] = nyq;
    }

    Voice<BL>::oscillator(Voice<BL>::buffer.data(), BL, modulator_buffer.data(), Voice<BL>::gain);
    Voice<BL>::adsr(Voice<BL>::buffer.data(), BL);
    this->handle_por();
}


#endif
