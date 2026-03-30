#ifndef ADSR_H
#define ADSR_H

#include <algorithm>
#include <cstdint>
#include <cmath>
#include "constants.h"
#include "linked_list.h"
#include "utils.h"

extern bool curves_initialized;
extern table_t exp_decay;
extern adsr_por_table_t adsr_por_table;
extern table_t exp_decay_inv;

void init_curves();

struct ADSR {
    enum class State : uint8_t { Attack, Decay, Sustain, Release, Idle };

    // --- user parameters (seconds / level) ---
    sample_t sustain_level  {0.5f};
    sample_t attack_time_s  {0.01f};
    sample_t decay_time_s   {0.05f};
    sample_t release_time_s {0.10f};

    // --- control-rate (ticks per second) ---
    sample_t control_rate   {1000.0f};  // set to sample_rate / CR if you use the CR wrapper

    // --- internal: durations in ticks (float, but only updated on parameter change) ---
    sample_t atk_ticks {0.0f};
    sample_t dcy_ticks {0.0f};
    sample_t rel_ticks {0.0f};

    // --- fixed-point progress per tick (Q16.16) ---
    static constexpr uint32_t FP  = 16;
    static constexpr uint32_t ONE = 1u << FP;

    // We read [idx] and [idx+1], so last valid base index is (N-2)
    static constexpr uint32_t LUT_N            = table_length;
    static constexpr uint32_t LUT_LAST_VALID   = (LUT_N >= 2) ? (LUT_N - 2) : 0;
    static constexpr uint32_t ACC_LIMIT        = (LUT_LAST_VALID << FP); // stop threshold

    // Stage step sizes (advance in Q16.16 each control tick)
    uint32_t atk_step {ACC_LIMIT};
    uint32_t dcy_step {ACC_LIMIT};
    uint32_t rel_step {ACC_LIMIT};

    // --- runtime state ---
    State    state {State::Idle};
    uint32_t acc   {0};          // Q16.16 accumulator (0..ACC_LIMIT)
    sample_t level {0.0f};       // current output (0..1)
    sample_t atk_start_level {0.0f};
    sample_t rel_start_level {0.0f};

    ADSR(sample_t a_s, sample_t d_s, sample_t sus, sample_t r_s, sample_t ctrl_rate);

    void set_attack(sample_t seconds);
    void set_decay(sample_t seconds);
    void set_release(sample_t seconds);
    void set_sustain(sample_t s);
    void set_control_rate(sample_t hz);

    void gate_on(sample_t current_level = 0.0f);
    void gate_off();

    sample_t operator()();

    bool active() const;

private:
    void recompute_ticks();
    void recompute_steps();
    static uint32_t ticks_to_step(sample_t ticks);
    static sample_t clamp01(sample_t x);
};


template <size_t len_ll>
LinkedList<sample_t, len_ll> adsr_to_ll(ADSR& adsr) {
    LinkedList<sample_t, len_ll> ll{};

    adsr.gate_on();

    while (true) {
        ll.append(adsr());

        if (adsr.state == ADSR::State::Sustain)
            adsr.gate_off();

        if (adsr.state == ADSR::State::Idle)
            break;

    }

    return ll;
}

inline uint32_t count(ADSR& adsr) {
    adsr.gate_on();
    uint32_t c = 0;
    while (true) {
        adsr();
        if (adsr.state == ADSR::State::Sustain)
            adsr.gate_off();

        if (adsr.state == ADSR::State::Idle)
            break;

        ++c;
    }
    return c;
}

template <size_t len_array, size_t len_ll>
struct ADSR_LI {
    ADSR core;
    sample_t sample_rate;
    adsr_table_t* adsr_arr_ptr;
    uint32_t a_n_samples{}, d_n_samples{}, r_n_samples{}, ad_n_samples{}, adr_n_samples{};
    sample_t delta_len{0.0};

    uint32_t counter, counter_por;
    sample_t current_level;
    sample_t a_initial, one_min_a_initial, r_scaling, r_por_scaling;
    bool idle, por;

    void calculate_params();
    void evaluate_array();

    ADSR_LI(adsr_table_t* adsr_arr_ptr, sample_t a_s, sample_t d_s, sample_t sus, sample_t r_s, sample_t sample_rate);

    sample_t operator()();
    void operator()(sample_t* out, uint32_t buffer_length);

    inline void set_attack (sample_t seconds) { core.set_attack(seconds);  calculate_params(); }
    inline void set_decay  (sample_t seconds) { core.set_decay(seconds);   calculate_params(); }
    inline void set_release(sample_t seconds) { core.set_release(seconds); calculate_params(); }
    inline void set_sustain(sample_t s)       { core.set_sustain(s);       calculate_params(); }

    void gate_on(sample_t a_initial=0.0) {
        this->a_initial = a_initial;
        one_min_a_initial = sample_t{1.0} - a_initial;
        idle = false;
        counter = 0u;
        por = false;
    }

    void gate_off() {
        r_scaling = current_level / core.sustain_level;
        counter = ad_n_samples + 1;
    }

    void por_on() {
        r_por_scaling = current_level;
        counter = adr_n_samples;
        counter_por = 0u;
        por = true;
    }

    void por_off() {
        por = false;
    }

    bool in_attack() const;
    bool in_decay() const;
    bool last_step_in_decay() const;
    bool in_sustain() const;
    bool in_release() const;

};

template<size_t len_array, size_t len_ll>
ADSR_LI<len_array, len_ll>::ADSR_LI(adsr_table_t* adsr_arr_ptr, sample_t a_s, sample_t d_s, sample_t sus, sample_t r_s, sample_t sample_rate):
    core{a_s, d_s, sus, r_s, ADSR_CORE_RATE}, sample_rate{sample_rate}, adsr_arr_ptr{adsr_arr_ptr},
    counter{0u}, counter_por{0u}, current_level{0.0},
    a_initial{0.0}, one_min_a_initial{1.0}, r_scaling{1.0}, r_por_scaling{1.0},
    idle{true}, por{false}
{
    calculate_params();
}

template<size_t len_array, size_t len_ll>
void ADSR_LI<len_array, len_ll>::evaluate_array() {
    std::copy_n(resample_linear<len_array>(adsr_to_ll<len_ll>(core)).begin(), len_array, adsr_arr_ptr->begin());
    (*adsr_arr_ptr)[len_array] = 0;
}


template<size_t len_array, size_t len_ll>
void ADSR_LI<len_array, len_ll>::calculate_params() {
    a_n_samples = static_cast<uint32_t>(core.attack_time_s * sample_rate);
    d_n_samples = static_cast<uint32_t>(core.decay_time_s * sample_rate);
    r_n_samples = static_cast<uint32_t>(core.release_time_s * sample_rate);
    ad_n_samples = a_n_samples + d_n_samples;
    adr_n_samples = ad_n_samples + r_n_samples;

    delta_len = static_cast<sample_t>(len_array) / static_cast<sample_t>(adr_n_samples);
    counter = 0u;
}

template<size_t len_array, size_t len_ll>
bool ADSR_LI<len_array, len_ll>::in_attack() const {
    return counter < a_n_samples;
}

template<size_t len_array, size_t len_ll>
bool ADSR_LI<len_array, len_ll>::in_decay() const {
    return a_n_samples <= counter && counter < ad_n_samples;
}

template<size_t len_array, size_t len_ll>
bool ADSR_LI<len_array, len_ll>::last_step_in_decay() const {
    return counter == ad_n_samples - 1;
}


template<size_t len_array, size_t len_ll>
bool ADSR_LI<len_array, len_ll>::in_sustain() const {
    return ad_n_samples == counter;
}

template<size_t len_array, size_t len_ll>
bool ADSR_LI<len_array, len_ll>::in_release() const {
    return ad_n_samples < counter && counter < adr_n_samples;
}

template<size_t len_array, size_t len_ll>
sample_t ADSR_LI<len_array, len_ll>::operator()() {
    idle = por ? (counter_por == adsr_por_length) : (counter == adr_n_samples);

    if (idle) {
        current_level = 0.0;
        return current_level;
    }

    if (in_sustain()) {
        current_level = core.sustain_level;
        return current_level;
    }

    if (!por) {
        const sample_t base_st{static_cast<sample_t>(counter) * delta_len};
        int base = static_cast<int>(base_st);
        const sample_t frac = base_st - static_cast<sample_t>(base);
        const sample_t a{(*adsr_arr_ptr)[base]};
        const sample_t b{(*adsr_arr_ptr)[base + 1]};
        const sample_t x{a + frac * (b - a)};

        if (in_attack())
            current_level = one_min_a_initial * x + a_initial;

        if (in_decay()) {
            current_level = x;
        }


        if (in_release()) {
            current_level = r_scaling * x;
        }

        ++counter;
        return current_level;

    } else {
        current_level = r_por_scaling * adsr_por_table[counter_por++];
        return current_level;
    }
}

template<size_t len_array, size_t len_ll>
void ADSR_LI<len_array, len_ll>::operator()(sample_t* out, uint32_t buffer_length) {

    for (uint32_t sample_i{0}; sample_i < buffer_length; ++sample_i) {
        idle = por ? (counter_por == adsr_por_length) : (counter == adr_n_samples);

        if (idle) {
            current_level = 0.0;
        } else if (ad_n_samples == counter) {
            current_level = core.sustain_level;
        } else {
            if (!por) {
                // counter is incremented here
                sample_t base_st{static_cast<sample_t>(counter) * delta_len};
                int base = static_cast<int>(base_st);
                sample_t frac = base_st - static_cast<sample_t>(base);
                sample_t a{(*adsr_arr_ptr)[base]};
                sample_t b{(*adsr_arr_ptr)[base + 1]};
                sample_t x{a + frac * (b - a)};

                if (counter < a_n_samples) {
                    // in attack
                    current_level = one_min_a_initial * x + a_initial;
                } else if (a_n_samples <= counter && counter < ad_n_samples) {
                    // in decay
                    current_level = x;
                } else if (ad_n_samples < counter && counter < adr_n_samples) {
                    // in release
                    current_level = r_scaling * x;
                }
                ++counter;
            } else {
                current_level = r_por_scaling * adsr_por_table[counter_por++];
            }
        }

        out[2 * sample_i] *= current_level;
        out[2 * sample_i + 1] *= current_level;
    }
}

#endif
