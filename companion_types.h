// The companion docs are incorrect, the message header is 16 bytes, not 8 bytes.
// There is some extra data after the flight number (from firmware source)
typedef struct {
  volatile uint8_t  command;
  volatile uint8_t  flight_state;
  volatile uint16_t tick;
           uint16_t serial;
           uint16_t flight; // End of companion docs
  volatile int16_t  accel;  // From firmware source
	volatile int16_t  speed;
	volatile int16_t  height;
	         uint16_t	motor_number;
} altos_header_t;

typedef struct {
  uint16_t board_id;
  uint16_t board_id_inverse;
  uint8_t update_period;
  uint8_t channels;
} altos_setup_t;

typedef struct {
  volatile uint16_t data[12];
} altos_fetch_t;

typedef struct {
  uint16_t tick;
  uint16_t value;
} data_t;