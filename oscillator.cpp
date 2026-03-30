#include "oscillator.h"

bool tables_initialized{false};
table_t sine_table{};
table_t square_table{};
table_t saw_incr_table{};
table_t saw_decr_table{};
table_t triangle_table{};

void init_sine() {
    for (uint32_t i{0}; i < table_length + 2; ++i)
        sine_table[i] = static_cast<sample_t>(std::sin(static_cast<sample_t>(i) / table_length_st * two_pi));
}

void init_square() {
    for (uint32_t i{0}; i < table_length; ++i) {
        if (i < table_length / 2)
            square_table[i] = static_cast<sample_t>(1.0);
        else square_table[i] = static_cast<sample_t>(-1.0);
    }

    square_table[table_length] = square_table[0];
    square_table[table_length + 1] = square_table[1];
}

void init_saw_incr() {

    for (uint32_t i{0}; i < table_length; ++i)
        saw_incr_table[i] = 2.0 * static_cast<sample_t>(i) / table_length_st - 1.0;

    saw_incr_table[table_length] = saw_incr_table[0];
    saw_incr_table[table_length + 1] = saw_incr_table[1];
}

void init_saw_decr() {
    for (uint32_t i{0}; i < table_length; ++i)
        saw_decr_table[i] = 1.0 - 2.0 * static_cast<sample_t>(i) / table_length_st;

    saw_decr_table[table_length] = saw_decr_table[0];
    saw_decr_table[table_length + 1] = saw_decr_table[1];
}

void init_triangle() {

    for (uint32_t i{0}; i < table_length; ++i) {
        sample_t phase = static_cast<sample_t>(i) / table_length_st;
        triangle_table[i] = std::fabs((std::floor(phase + 0.75) - phase) * 4.0 - 1.0) - 1.0;
    }

    triangle_table[table_length] = triangle_table[0];
    triangle_table[table_length + 1] = triangle_table[1];
}

void init_tables() {
    if (!tables_initialized) {
        init_sine();
        init_square();
        init_saw_incr();
        init_saw_decr();
        init_triangle();

        tables_initialized = true;
    }
}

table_t* wavetable_addr(const Wavetable wavetable) {
    switch (wavetable) {
        case Wavetable::sine:
            return &sine_table;

        case Wavetable::square:
            return &square_table;

        case Wavetable::saw_incr:
            return &saw_incr_table;

        case Wavetable::saw_decr:
            return &saw_decr_table;

        case Wavetable::triangle:
            return &triangle_table;

        default:
            return nullptr;
    }
}

void Oscillator::set_table(Wavetable wavetable) {
    this->wavetable = wavetable;
    table = wavetable_addr(wavetable);
}

Oscillator::Oscillator(sample_t sample_rate, sample_t initial_phase, Wavetable wavetable):
    sr{sample_rate}, phase{initial_phase}, initial_phase{initial_phase}, last_output{0.0}
{
    init_tables();
    set_table(wavetable);
}

sample_t Oscillator::lookup(sample_t phase) {
    // if you multiply the phase increment with two_pi
    // sample_t base_st = phase * phase_to_index;
    // otherwise
    sample_t base_st = phase * table_length_st;
    int base = static_cast<int>(base_st);

    sample_t frac = base_st - static_cast<sample_t>(base);
    sample_t a{(*table)[base]};
    sample_t b{(*table)[base + 1]};
    sample_t value = a + frac * (b - a);
    return value;
}

// https://www.martin-finke.de/articles/audio-plugins-018-polyblep-oscillator/
inline sample_t poly_blep(sample_t t, sample_t dt) {
    // t in [0, 1)
    if (t < dt) {
        t /= dt;
        return t + t - t * t - static_cast<sample_t>(1.0);        // 2t - t^2 - 1
    } else if (t > static_cast<sample_t>(1.0) - dt) {
        t = (t - static_cast<sample_t>(1.0)) / dt;
        return t * t + t + t + static_cast<sample_t>(1.0);       // t^2 + 2t + 1
    }
    return static_cast<sample_t>(0.0);
}

static inline sample_t blamp(sample_t t, sample_t dt)
{
    // -------- BOTTOM (falling edge at t = 0.75) --------

    // AFTER BOTTOM
    if (t >= static_cast<sample_t>(0.75) && t <= static_cast<sample_t>(0.75) + dt) {
        sample_t p = (t - static_cast<sample_t>(0.75)) / dt - static_cast<sample_t>(1.0);
        return -(p * p * p * static_cast<sample_t>(4.0 / 3.0) * dt);
    }

    // BEFORE BOTTOM
    if (t >= sample_t(0.75) - dt && t <= static_cast<sample_t>(0.75)) {
        sample_t p = (t - static_cast<sample_t>(0.75)) / dt + static_cast<sample_t>(1.0);
        return  (p * p * p * static_cast<sample_t>(4.0 / 3.0) * dt);
    }

    // -------- PEAK (rising edge at t = 0.25) --------

    // AFTER PEAK
    if (t >= static_cast<sample_t>(0.25) && t <= static_cast<sample_t>(0.25) + dt) {
        sample_t p = (t - static_cast<sample_t>(0.25)) / dt - static_cast<sample_t>(1.0);
        return  (p * p * p * static_cast<sample_t>(4.0 / 3.0) * dt);
    }

    // BEFORE PEAK
    if (t >= static_cast<sample_t>(0.25) - dt && t <= static_cast<sample_t>(0.25)) {
        sample_t p = (t - static_cast<sample_t>(0.25)) / dt + static_cast<sample_t>(1.0);
        return -(p * p * p * static_cast<sample_t>(4.0 / 3.0) * dt);
    }

    return 0.0;
}

sample_t Oscillator::operator()(sample_t freq, sample_t gain) {
    // mathematically, the following is correct:
    // phase += two_pi * freq / sr;
    // if (phase >= two_pi) phase -= two_pi;
    // because the phase should go from [0, 2π)
    // however, in table lookup we will have to divide by two_pi then
    // therefore we omit to multipy by two_pi here
    // and divide by two_pi in lookup

    sample_t y{0.0};
    sample_t phase_incr = freq / sr;

    if (wavetable == Wavetable::sine) {
        y = lookup(phase);
    }


    if (wavetable == Wavetable::square) {
        if (phase < static_cast<sample_t>(0.5)) y = static_cast<sample_t>(1.0);
        else                                     y = static_cast<sample_t>(-1.0);
        y += poly_blep(phase, phase_incr); // Discontinuity at wrap-around (t = 0)
        sample_t t = phase + static_cast<sample_t>(0.5);
        if (t >= static_cast<sample_t>(1.0)) t -= static_cast<sample_t>(1.0);
        y -= poly_blep(t, phase_incr); // Discontinuity at half-cycle (t = 0.5)

    }

    if (wavetable == Wavetable::triangle) {
        if (true) {
            y = std::fabs((std::floor(phase + static_cast<sample_t>(0.75)) - phase) * static_cast<sample_t>(4.0) - static_cast<sample_t>(1.0)) - static_cast<sample_t>(1.0);
            y += blamp(phase, phase_incr);
        } else {
            if (phase < static_cast<sample_t>(0.5)) y = static_cast<sample_t>(1.0);
            else                                     y = static_cast<sample_t>(-1.0);
            y += poly_blep(phase, phase_incr); // Discontinuity at wrap-around (t = 0)
            sample_t t = phase + static_cast<sample_t>(0.5);
            if (t >= static_cast<sample_t>(1.0)) t -= static_cast<sample_t>(1.0);
            y -= poly_blep(t, phase_incr); // Discontinuity at half-cycle (t = 0.5)

            y = phase_incr * y + (static_cast<sample_t>(1.0) - phase_incr) * last_output;
            last_output = y;
        }

    }

    if (wavetable == Wavetable::saw_incr) {
        y = static_cast<sample_t>(2.0) * phase - static_cast<sample_t>(1.0);
        y -= poly_blep(phase, phase_incr);
    }


    if (wavetable == Wavetable::saw_decr) {
        y = static_cast<sample_t>(1.0) - static_cast<sample_t>(2.0) * phase;
        y += poly_blep(phase, phase_incr);

    }

    phase += phase_incr;
    if (phase >= static_cast<sample_t>(1.0)) phase -= static_cast<sample_t>(1.0);

    return y * gain;
}

void Oscillator::operator()(sample_t* out, uint32_t buffer_length, sample_t freq, sample_t gain) {

    sample_t phase_incr = freq / sr;
    sample_t y{0};

    for (uint32_t sample_i{0}; sample_i < buffer_length; ++sample_i) {
        switch (wavetable) {
            case Wavetable::sine: {
                const sample_t base_st = phase * table_length_st;
                const int base = static_cast<int>(base_st);
                const sample_t frac = base_st - static_cast<sample_t>(base);
                const sample_t a{(*table)[base]};
                const sample_t b{(*table)[base + 1]};
                y = a + frac * (b - a);

            }
            break;
            case Wavetable::square:
                y = (phase < static_cast<sample_t>(0.5)) ? static_cast<sample_t>(1.0) : static_cast<sample_t>(-1.0);
                y += poly_blep(phase, phase_incr); // Discontinuity at wrap-around (t = 0)
                {
                    sample_t t = phase + static_cast<sample_t>(0.5);
                    if (t >= static_cast<sample_t>(1.0)) t -= static_cast<sample_t>(1.0);
                    y -= poly_blep(t, phase_incr); // Discontinuity at half-cycle (t = 0.5)
                }
                break;
            case Wavetable::triangle:
                y = std::fabs((std::floor(phase + static_cast<sample_t>(0.75)) - phase) * static_cast<sample_t>(4.0) - static_cast<sample_t>(1.0)) - static_cast<sample_t>(1.0);
                y += blamp(phase, phase_incr);
                break;
            case Wavetable::saw_incr:
                y = static_cast<sample_t>(2.0) * phase - static_cast<sample_t>(1.0);
                y -= poly_blep(phase, phase_incr);
                break;
            case Wavetable::saw_decr:
                y = static_cast<sample_t>(1.0) - static_cast<sample_t>(2.0) * phase;
                y += poly_blep(phase, phase_incr);
            default:
                break;
        }

        y *= gain;

        out[2 * sample_i] = y;
        out[2 * sample_i + 1] = y;

        phase += phase_incr;
        if (phase >= static_cast<sample_t>(1.0)) phase -= static_cast<sample_t>(1.0);

    }
}

void Oscillator::operator()(sample_t* out, uint32_t buffer_length, sample_t* freqs, sample_t gain) {


    sample_t y{0.0};

    for (uint32_t sample_i{0}; sample_i < buffer_length; ++sample_i) {

        // IMPORTANT
        // EVERY SECOND FREQUENCY SHOULD BE FETCHED
        // TODO: EXPLAIN IN A MORE DETAIL
        sample_t phase_incr = freqs[2 * sample_i] / sr;

        switch (wavetable) {
            case Wavetable::sine: {
                const sample_t base_st = phase * table_length_st;
                const int base = static_cast<int>(base_st);
                const sample_t frac = base_st - static_cast<sample_t>(base);
                const sample_t a{(*table)[base]};
                const sample_t b{(*table)[base + 1]};
                y = a + frac * (b - a);
                break;
            }
            case Wavetable::square:
                y = (phase < static_cast<sample_t>(0.5)) ? static_cast<sample_t>(1.0) : static_cast<sample_t>(-1.0);
                y += poly_blep(phase, phase_incr); // Discontinuity at wrap-around (t = 0)
                {
                    sample_t t = phase + static_cast<sample_t>(0.5);
                    if (t >= static_cast<sample_t>(1.0)) t -= static_cast<sample_t>(1.0);
                    y -= poly_blep(t, phase_incr); // Discontinuity at half-cycle (t = 0.5)
                }
                break;
            case Wavetable::triangle:
                y = std::fabs((std::floor(phase + static_cast<sample_t>(0.75)) - phase) * static_cast<sample_t>(4.0) - static_cast<sample_t>(1.0)) - static_cast<sample_t>(1.0);
                y += blamp(phase, phase_incr);
                break;
            case Wavetable::saw_incr:
                y = static_cast<sample_t>(2.0) * phase - static_cast<sample_t>(1.0);
                y -= poly_blep(phase, phase_incr);
                break;
            case Wavetable::saw_decr:
                y = static_cast<sample_t>(1.0) - static_cast<sample_t>(2.0) * phase;
                y += poly_blep(phase, phase_incr);
            default:
                break;
        }

        y *= gain;

        out[2 * sample_i] = y;
        out[2 * sample_i + 1] = y;

        phase += phase_incr;
        if (phase >= static_cast<sample_t>(1.0)) phase -= static_cast<sample_t>(1.0);

    }
}