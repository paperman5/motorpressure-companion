#pragma once

void blink_loop();
void change_blink_pattern(void (*bp_func)(void));
void bp_companion_connection_wait();
void bp_companion_connected_ready();
void bp_recording();
void bp_finished();