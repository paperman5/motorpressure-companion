#include <string.h>
#include <bsp/board_api.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "companion_blink.h"

static bool blink_pattern[5] = {false};
static uint32_t blink_times[5] = { 1000, 0, 0, 0, 0 };
static uint8_t pattern_length = 1;
static uint8_t pattern_index = 0;
static uint32_t next_time_ms = 0;

void blink_loop() {
    while (true) {
        uint32_t now = board_millis();
        if (now >= next_time_ms) {
            pattern_index += 1;
            pattern_index %= pattern_length;
            gpio_put(PICO_DEFAULT_LED_PIN, blink_pattern[pattern_index]);
            next_time_ms = now + blink_times[pattern_index];
        }
    }
}

void change_blink_pattern(void (*bp_func)(void)) {
    multicore_lockout_start_blocking();
    bp_func();
    pattern_index = pattern_length - 1;
    next_time_ms = board_millis();
    multicore_lockout_end_blocking();
}

void bp_companion_connection_wait() {
    bool pattern[2] = {true, false};
    uint32_t times[2] = {1000u, 1000u};
    pattern_length = 2;
    memcpy(blink_pattern, pattern, sizeof(pattern));
    memcpy(blink_times, times, sizeof(times));
}

void bp_companion_connected_ready() {
    bool pattern[1] = {true};
    uint32_t times[1] = {1000u};
    pattern_length = 1;
    memcpy(blink_pattern, pattern, sizeof(pattern));
    memcpy(blink_times, times, sizeof(times));
}

void bp_recording() {
    bool pattern[2] = {true, false};
    uint32_t times[2] = {250u, 250};
    pattern_length = 2;
    memcpy(blink_pattern, pattern, sizeof(pattern));
    memcpy(blink_times, times, sizeof(times));
}

void bp_finished() {
    bool pattern[4] = {false, true, false, true};
    uint32_t times[4] = {1250u, 250u, 250u, 250u};
    pattern_length = 4;
    memcpy(blink_pattern, pattern, sizeof(pattern));
    memcpy(blink_times, times, sizeof(times));
}