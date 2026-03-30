#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include "constants.h"

#include <array>

extern bool tables_initialized;

extern table_t sine_table;
extern table_t square_table;
extern table_t saw_incr_table;
extern table_t saw_decr_table;
extern table_t triangle_table;

void init_sine();
void init_square();
void init_saw_incr();
void init_saw_decr();
void init_triangle();
void init_tables();

struct Oscillator {
    Wavetable wavetable;
    table_t* table;
    sample_t sr;
    sample_t phase, initial_phase;
    sample_t last_output; // for PolyBLEP
    
    Oscillator(sample_t sample_rate, sample_t initial_phase=0.0, Wavetable wavetable=Wavetable::sine);

    void set_table(Wavetable wavetable);

    sample_t lookup(sample_t phase);
    sample_t operator()(sample_t freq, sample_t gain);
    void operator()(sample_t* out, uint32_t buffer_length, sample_t freq, sample_t gain);
    void operator()(sample_t* out, uint32_t buffer_length, sample_t* freqs, sample_t gain);
};

table_t* wavetable_addr(const Wavetable wavetable);

#endif