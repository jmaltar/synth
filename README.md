# synth

A portable, zero-dependency C++17 synthesis engine. No heap allocations, no external libraries -- everything is statically allocated at compile time, making it suitable for bare-metal microcontrollers and real-time audio alike. The same codebase runs on STM32 Cortex-M4/M7, ESP32-S3, Raspberry Pi Pico, and desktop Linux/Windows with no platform-specific #ifdefs in the core engine.

## Architecture

```
   ┌──────────────────┐  ┌──────────────────┐
   │      Audio       │  │       MIDI       │
   │ (platform-spec.) │  │ (platform-spec.) │
   └────────┬─────────┘  └────────┬─────────┘
            : callback            : callback
            :                     :
   ┌────────────────────────────────────────┐
   │                  Synth                 │
   │                 synth.h                │
   └───────────────────▲───────────────────┘
                       │
                       │
   ┌────────────────────────────────────────┐
   │                 Voice                  │
   │                voice.h                 │
   └────────▲───────────────────▲─────────┘
            │                     │
            │                     │
   ┌──────────────────┐  ┌──────────────────┐
   │    Oscillator    │  │       ADSR       │
   │  oscillator.h    │  │     adsr.h       │
   └────────▲────────┘  └───────▲─────────┘
            │                     │
            │                     │
   ┌──────────────────────────────────────┐
   │                Constants             │
   │               constants.h            │
   └──────────────────────────────────────┘
```

## Files

| File | Description |
|------|-------------|
| `synth.h` / `synth.cpp` | Top-level `Synth<V>` template: voice allocation policy, block rendering, note on/off |
| `voice.h` / `voice.cpp` | `Voice<BL>` base, `SimpleVoice` (single oscillator), `FMVoice` (two-operator FM) with ADSR envelope |
| `adsr.h` / `adsr.cpp` | ADSR envelope generator |
| `oscillator.h` / `oscillator.cpp` | Oscillator with sine wavetable and PolyBLEP/BLAMP square/saw/triangle |
| `constants.h` | Sample rate, buffer sizes, voice count, wavetable types |
| `utils.h` | Helper functions -- platform-abstracted timing, sample conversion, resampling |
| `linked_list.h` | Helper data structure used by ADSR envelope tables |

## Synth

`Synth<V>` is a template parameterized by the voice type (`FMVoice` or `SimpleVoice`). It manages an array of `n_voices` voices and provides two rendering paths:

- **`operator()(sample_t* out)`** -- block-based: each voice renders its full block into its own buffer, then the synth sums them. Used on embedded targets where per-voice block processing is more cache-friendly.
- **`process_sample_by_sample(sample_t* out)`** -- sample-by-sample: iterates all voices per sample with smoothed normalization (`1/sqrt(n_active)`) to prevent clipping as voices stack. Used on desktop.

Both paths write stereo interleaved output. An `int16_t*` overload is also provided that converts from float automatically.

### Voice allocation

`policy()` selects which voice to assign on note-on:
1. Prefer a free voice (unassigned or idle ADSR), picking the one that has been free the longest.
2. If no free voice, steal a voice that is in its release phase, picking the one with the lowest current ADSR level.
3. If all voices are active and not releasing, the note is dropped.

## Voice

`Voice<BL>` is a base template parameterized by block length. It holds an oscillator, an ADSR envelope, the current note/frequency/gain, and a stereo output buffer. Two concrete voice types inherit from it:

### SimpleVoice

Single-oscillator subtractive voice. The oscillator generates a block of samples, then the ADSR envelope is applied as an amplitude multiplier.

### FMVoice

Two-operator FM voice with a carrier and a modulator oscillator (always sine). The modulator's output is scaled by `fm_index * env_mod * freq_modulator` and added to the carrier frequency, producing frequency modulation. The modulator has its own independent ADSR envelope (`adsr_modulator`) controlling the FM depth over time. Instantaneous carrier frequency is clamped to `[0, 0.49 * sample_rate]` to prevent aliasing.

Parameters: `ratio` (modulator-to-carrier frequency ratio), `fm_index` (modulation depth), plus independent ADSR settings for both carrier and modulator envelopes.

## ADSR

Two-layer envelope design:

- **`ADSR`** (core): a low-rate state machine (default 100 Hz) that walks through Attack, Decay, Sustain, Release, Idle states. Uses exponential curves from a precomputed lookup table (`exp_decay`) with Q16.16 fixed-point accumulator for phase tracking. Each stage advances through the LUT at a rate determined by the stage duration in seconds.

- **`ADSR_LI`** (interpolated wrapper): operates at audio rate. The core ADSR is run once to generate a full A-D-S-R shape, which is resampled into a fixed-size array (`adsr_table_t`, 1025 samples). At runtime, `ADSR_LI` steps through this precomputed array using a sample counter, producing one envelope value per audio sample. This avoids running the state machine at audio rate.

`ADSR_LI` recalculates its internal array whenever a parameter (A/D/S/R) changes. Carrier and modulator envelopes use separate arrays, so changing one doesn't affect the other.

## Oscillator

Supports five waveforms: sine, square, saw (rising), saw (falling), and triangle.

- **Sine** uses a wavetable with linear interpolation between adjacent samples.
- **Square, saw, triangle** are computed analytically per sample with PolyBLEP (square, saw) and BLAMP (triangle) anti-aliasing to suppress discontinuity artifacts near Nyquist.

Three `operator()` overloads:
- Single sample: `operator()(freq, gain)` -- returns one sample, advances phase.
- Block with constant frequency: `operator()(out, length, freq, gain)` -- fills a buffer.
- Block with per-sample frequency array: `operator()(out, length, freqs, gain)` -- used by FMVoice where the modulator produces a per-sample frequency deviation.

## Compile-time configuration

Set these via `target_compile_definitions` in your project's CMakeLists.txt:

| Define | Default | Description |
|--------|---------|-------------|
| `SYNTH_N_VOICES` | 6 | Number of polyphonic voices |
| `SYNTH_HEADROOM` | `1.0 / n_voices` | Master headroom scalar |
| `SYNTH_SAMPLE_USE_DOUBLE` | (not set) | Use `double` instead of `float` for `sample_t` |

## Platform abstraction

`utils.h` auto-detects the platform via compiler-defined macros:

| Macro | Platform | Timing source |
|-------|----------|---------------|
| `USE_HAL_DRIVER` | STM32 (HAL) | `HAL_GetTick()` |
| `ESP_PLATFORM` | ESP32 (ESP-IDF) | `esp_timer_get_time()` |
| *(neither)* | Desktop / POSIX | `std::chrono::steady_clock` |

## Audio pipeline

1. The target creates a `Synth<FMVoice>` (or `Synth<SimpleVoice>`) with the sample rate. The constructor calls `voices[0].adsr_init()`, which runs the core ADSR to generate the envelope shape and resamples it into the shared `adsr_arr` / `adsr_arr_2` arrays used by all voices.
2. In its audio callback (DMA interrupt on embedded, PortAudio callback on desktop), the target calls `synth(buffer)` or `synth.process_sample_by_sample(buffer)`.
3. The synth iterates active voices, sums their output, and writes stereo interleaved samples.
4. Output format: `sample_t*` (float/double) or `int16_t*` (with automatic conversion via `sample_t_to_s16`).

Fixed parameters: 48 kHz sample rate, 64 stereo samples per block (16 mono samples x4).
