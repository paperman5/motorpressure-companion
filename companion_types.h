#pragma once

// The companion docs are incorrect, the message header is 16 bytes, not 8 bytes.
// There is some extra data after the flight number (from firmware source)
typedef struct {
    uint8_t  command;
    uint8_t  flight_state;
    uint16_t tick;
    uint16_t serial;
    uint16_t flight; // End of companion docs
    int16_t  accel;  // From firmware source
    int16_t  speed;
    int16_t  height;
    uint16_t motor_number;
} altos_header_t;

typedef struct {
    uint16_t board_id;
    uint16_t board_id_inverse;
    uint8_t  update_period;
    uint8_t  channels;
} altos_setup_t;

typedef struct {
    uint16_t data[12];
} altos_fetch_t;

typedef struct {
    uint16_t tick;
    uint16_t value;
} data_t;

enum Command {
    CMD_SETUP = 1,
    CMD_FETCH = 2,
    CMD_NOTIFY = 3
};
enum FlightState { // From firmware kernel source `ao_flight.h`
    FS_STARTUP = 0,
    FS_IDLE = 1,
    FS_PAD = 2,
    // PAD->BOOST: Baro >20m vert OR accel >2g && vel > 5m/s
    FS_BOOST = 3,
    // BOOST->FAST: accel < -1/4g OR boost time > 15s
    FS_FAST = 4,
    FS_COAST = 5,
    FS_DROGUE = 6,
    FS_MAIN = 7,
    FS_LANDED = 8,
    FS_INVALID = 9,
    FS_TEST = 10
};
enum ProgramState {
    PS_DISCONNECTED = 0,
    PS_MONITORING = 1,
    PS_RECORDING = 2,
    PS_DONE = 3,
    PS_ERROR = -1
};