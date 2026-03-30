#ifndef UTILS_H
#define UTILS_H

#include <algorithm>

#include "constants.h"
#include "linked_list.h"

#if defined(USE_HAL_DRIVER)
// ---- STM32 bare-metal -------------------------------------------------------
#include "main.h"  // HAL_GetTick()

inline uint64_t ticks_by_far() {
    return static_cast<uint64_t>(HAL_GetTick());
}

#elif defined(ESP_PLATFORM)
// ---- ESP32 / ESP-IDF --------------------------------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

inline uint64_t ticks_by_far() {
    return static_cast<uint64_t>(esp_timer_get_time());  // microseconds
}

inline void sleep_ms(int64_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#else
// ---- Desktop / POSIX --------------------------------------------------------
#include <chrono>
#include <thread>
#include <iostream>

struct Stopwatch {
    enum class Unit { milliseconds, seconds };
    using clock = std::chrono::steady_clock;

    clock::time_point start;

    Stopwatch() : start(clock::now()) {}

    void reset() { start = clock::now(); }

    double operator()(Unit u = Unit::seconds) const {
        auto dt = clock::now() - start;
        double secs = std::chrono::duration<double>(dt).count();
        if (u == Unit::seconds) return secs;
        else                    return secs * 1000.0;
    }
};

inline void sleep_ms(int64_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline uint64_t ticks_by_far() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()
    ).count();
}

#endif

// ---- Shared utilities (all platforms) ---------------------------------------

static int16_t sample_t_to_s16(sample_t x)
{
    x = std::clamp(x, sample_t{-1}, sample_t{1});
    sample_t scaled = x * sample_t{32767};
    scaled = std::nearbyint(scaled);
    int32_t i = static_cast<int32_t>(scaled);
    i = std::clamp(i, int32_t{-32768}, int32_t{32767});
    return static_cast<int16_t>(i);
}

[[maybe_unused]] static void sample_t_array_to_s16_array(const sample_t* in, int16_t* out, size_t n) {
    for (size_t i = 0; i < n; ++i)
        out[i] = sample_t_to_s16(in[i]);
}

template <size_t len_array, size_t len_ll>
std::array<sample_t, len_array> resample_linear(const LinkedList<sample_t, len_ll>& ll) {
    std::size_t original_len = ll.length();
    std::array<sample_t, len_array> output{};
    for (std::size_t i = 0; i < len_array; ++i) {
        sample_t pos = static_cast<sample_t>(i) / (len_array - 1) * (original_len - 1);
        std::size_t idx = static_cast<std::size_t>(pos);
        sample_t frac = pos - idx;
        if (idx + 1 < original_len)
            output[i] = (1.0 - frac) * ll.get(idx) + frac * ll.get(idx + 1);
        else
            output[i] = ll.get(original_len - 1);
    }
    return output;
}

template <std::size_t len_array_out, std::size_t len_array_in>
std::array<sample_t, len_array_out> resample_linear(const std::array<sample_t, len_array_in>& arr) {
    std::array<sample_t, len_array_out> out{};
    for (std::size_t i = 0; i < len_array_out; ++i) {
        sample_t pos = sample_t(i) * sample_t(len_array_in - 1) / sample_t(len_array_out - 1);
        std::size_t idx = static_cast<std::size_t>(pos);
        if (idx >= len_array_in - 1) { out[i] = arr[len_array_in - 1]; continue; }
        sample_t frac = pos - sample_t(idx);
        out[i] = (sample_t(1) - frac) * arr[idx] + frac * arr[idx + 1];
    }
    return out;
}

template <size_t M, size_t N>
std::array<sample_t, M> slice(const std::array<sample_t, N>& a, size_t offset) {
    std::array<sample_t, M> out{};
    std::copy_n(a.begin() + offset, M, out.begin());
    return out;
}

#endif
