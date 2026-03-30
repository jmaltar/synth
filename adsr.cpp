#include "adsr.h"

void initialize_exponential_curve(table_t& table, sample_t y_0, sample_t y_1, sample_t t) {
    constexpr sample_t k = 1.0 / static_cast<sample_t>(table_length);
    const sample_t a = std::exp(-k / t);

    sample_t max_value = y_0;
    sample_t min_value = y_0;

    table[0] = y_0;
    for (unsigned int i = 1; i < table_length; ++i) {
        table[i] = a * table[i - 1] + (1.0 - a) * y_1;
        if (table[i] > max_value) max_value = table[i];
        if (table[i] < min_value) min_value = table[i];
    }

    sample_t range = max_value - min_value;
    if (std::fabs(range) < 1e-12) range = 1.0; // avoid divide-by-zero

    for (unsigned int i = 0; i < table_length; ++i)
        table[i] = (table[i] - min_value) / range;

    // padding for safe interpolation (idx+1)
    table[table_length]     = table[table_length - 1];
    table[table_length + 1] = table[table_length - 1];
}


bool curves_initialized{false};
table_t exp_decay{};
table_t exp_decay_inv{};
adsr_por_table_t adsr_por_table{};

void initialize_adsr_por_table() {
    std::copy_n(
        resample_linear<adsr_por_length, table_length>(slice<table_length, table_length + 2>(exp_decay, 0)).begin(),
        adsr_por_length,
        adsr_por_table.begin()
    );
}

void init_curves() {
    if (!curves_initialized) {
        initialize_exponential_curve(exp_decay,     1.0, 0.0, 0.5);
        initialize_exponential_curve(exp_decay_inv, 0.0, 1.0, 0.5);
        initialize_adsr_por_table();
        curves_initialized = true;
    }
}

ADSR::ADSR(sample_t a_s, sample_t d_s, sample_t sus, sample_t r_s, sample_t ctrl_rate)
    : sustain_level{sus}, attack_time_s{a_s}, decay_time_s{d_s}, release_time_s{r_s}, control_rate{ctrl_rate} {
    if (!curves_initialized) init_curves();
    recompute_ticks();
    recompute_steps();
    state = State::Idle;
    acc = 0;
    level = 0.0f;
    atk_start_level = 0.0f;
    rel_start_level = 0.0f;
}

void ADSR::set_attack(sample_t seconds) {
    attack_time_s = seconds;
    recompute_ticks();
    recompute_steps();
}

void ADSR::set_decay(sample_t seconds) {
    decay_time_s = seconds;
    recompute_ticks();
    recompute_steps();
}

void ADSR::set_release(sample_t seconds) {
    release_time_s = seconds;
    recompute_ticks();
    recompute_steps();
}

void ADSR::set_sustain(sample_t s) {
    sustain_level = clamp01(s);
}

void ADSR::set_control_rate(sample_t hz) {
    control_rate = (hz <= 0.0f ? 1.0f : hz);
    recompute_ticks();
    recompute_steps();
}

void ADSR::gate_on(sample_t current_level) {
    atk_start_level = clamp01(current_level);
    acc = 0;
    if (attack_time_s <= 0.0f) {
        level = 1.0f;
        state = (decay_time_s <= 0.0f) ? State::Sustain : State::Decay;
    } else {
        state = State::Attack;
    }
}

void ADSR::gate_off() {
    rel_start_level = clamp01(level);
    acc = 0;
    state = (release_time_s <= 0.0f) ? State::Idle : State::Release;
    if (state == State::Idle) level = 0.0f;
}

sample_t ADSR::operator()() {
    switch (state) {
        case State::Idle:
            level = 0.0f;
            return level;

        case State::Sustain:
            level = sustain_level;
            return level;

        case State::Attack: {
            if (atk_ticks <= 0.0f) {
                level = (decay_time_s <= 0.0f) ? sustain_level : 1.0f;
                state = (decay_time_s <= 0.0f) ? State::Sustain : State::Decay;
                acc = 0;
                return level;
            }
            const uint32_t idx = (acc >> FP);
            const sample_t frac = static_cast<sample_t>(acc & (ONE - 1)) * (1.0f / static_cast<sample_t>(ONE));
            const sample_t a = exp_decay_inv[idx];
            const sample_t b = exp_decay_inv[idx + 1];
            const sample_t shaped = a + frac * (b - a);
            level = atk_start_level + (1.0f - atk_start_level) * shaped;

            acc += atk_step;
            if (acc >= ACC_LIMIT) {
                acc = 0;
                if (decay_time_s <= 0.0f) {
                    state = State::Sustain;
                    level = sustain_level;
                } else {
                    state = State::Decay;
                }
            }
            return level;
        }

        case State::Decay: {
            if (dcy_ticks <= 0.0f) {
                state = State::Sustain;
                level = sustain_level;
                acc = 0;
                return level;
            }
            const uint32_t idx = (acc >> FP);
            const sample_t frac = static_cast<sample_t>(acc & (ONE - 1)) * (1.0f / static_cast<sample_t>(ONE));
            const sample_t a = exp_decay[idx];
            const sample_t b = exp_decay[idx + 1];
            const sample_t shaped = a + frac * (b - a);
            level = sustain_level + (1.0f - sustain_level) * shaped;

            acc += dcy_step;
            if (acc >= ACC_LIMIT) {
                acc = 0;
                state = State::Sustain;
                level = sustain_level;
            }
            return level;
        }

        case State::Release: {
            if (rel_ticks <= 0.0f) {
                state = State::Idle;
                level = 0.0f;
                acc = 0;
                return level;
            }
            const uint32_t idx = (acc >> FP);
            const sample_t frac = static_cast<sample_t>(acc & (ONE - 1)) * (1.0f / static_cast<sample_t>(ONE));
            const sample_t a = exp_decay[idx];
            const sample_t b = exp_decay[idx + 1];
            const sample_t shaped = a + frac * (b - a);
            level = rel_start_level * shaped;

            acc += rel_step;
            if (acc >= ACC_LIMIT || level < 1e-8f) {
                state = State::Idle;
                level = 0.0f;
                acc = 0;
            }
            return level;
        }
    }
    return level;
}

bool ADSR::active() const {
    return state != State::Idle;
}

void ADSR::recompute_ticks() {
    atk_ticks = attack_time_s * control_rate;
    dcy_ticks = decay_time_s * control_rate;
    rel_ticks = release_time_s * control_rate;
}

void ADSR::recompute_steps() {
    atk_step = ticks_to_step(atk_ticks);
    dcy_step = ticks_to_step(dcy_ticks);
    rel_step = ticks_to_step(rel_ticks);
}

uint32_t ADSR::ticks_to_step(sample_t ticks) {
    if (ticks <= 0.0f) return ACC_LIMIT;
    const sample_t s = static_cast<sample_t>(ACC_LIMIT) / ticks;
    return static_cast<uint32_t>(s + 0.5f);
}

sample_t ADSR::clamp01(sample_t x) {
    return (x < 0.0f) ? 0.0f : (x > 1.0f ? 1.0f : x);
}
