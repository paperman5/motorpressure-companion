#include <SPI.h>


#define PIN_BUTTON 5
#define PIN_CS 0
#define PIN_EN 1

enum Command {
  NONE = 0,
  SETUP = 1,
  FETCH = 2,
  NOTIFY = 3
};
enum FlightState { // From firmware kernel source `ao_flight.h`
  STARTUP = 0,
  IDLE = 1,
  PAD = 2,
  // PAD->BOOST: Baro >20m vert OR accel >2g && vel > 5m/s
  BOOST = 3,
  // BOOST->FAST: accel < -1/4g OR boost time > 15s
  FAST = 4,
  COAST = 5,
  DROGUE = 6,
  MAIN = 7,
  LANDED = 8,
  INVALID = 9,
  TEST = 10
};

typedef struct {
  volatile uint8_t command;
  volatile uint8_t flight_state;
  volatile uint16_t tick;
  uint16_t         serial;
  uint16_t         flight; // End of companion docs
  volatile int16_t accel;  // From firmware source
	volatile int16_t speed;
	volatile int16_t height;
	uint16_t	       motor_number;
} AltOSMessageHeader;

typedef struct {
  uint16_t board_id;
  uint16_t board_id_inverse;
  uint8_t update_period;
  uint8_t channels;
} AltOSSetupReply;

SPISettings spisettings(200000, MSBFIRST, SPI_MODE0);
volatile uint16_t startTick = 0;
AltOSMessageHeader message;
FlightState prevFlightState = INVALID;
volatile bool connected = false;
volatile bool sendFetch = false;
volatile bool sentThisTick = false;
volatile uint8_t setupsSent = 0;
volatile uint8_t channels = 0;
uint16_t stageTickCounts[11];
uint8_t rxBuffer[40];
uint8_t txBuffer[40];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  init_message_startup();
  setupStages();
  init_clocks();
  init_tick_timer();
  SPI.begin();
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);
  memset(rxBuffer, 0, sizeof(rxBuffer));
  memset(txBuffer, 0, sizeof(txBuffer));
  Serial.println(F("Initialized"));
}

void loop() {
  FlightState nextFlightState = static_cast<FlightState>(message.flight_state);
  bool buttonPressed = !digitalRead(PIN_BUTTON);
  digitalWrite(PIN_LED, buttonPressed);
  uint16_t tdiff = message.tick - startTick;

  if (nextFlightState != prevFlightState) {
    Serial.printf("STATE: %d -> %d (%d)\n", prevFlightState, nextFlightState, message.tick);
    if (nextFlightState < INVALID && prevFlightState < INVALID && nextFlightState > STARTUP && prevFlightState > STARTUP && connected) {
      send_notify();
    }
  }

  if ((nextFlightState == INVALID || nextFlightState == STARTUP) && tdiff < stageTickCounts[STARTUP] && message.tick % 10 == 0 && !sentThisTick) {
    if (message.tick > 400) {
      message.flight_state = STARTUP;
    }
    sentThisTick = true;
    send_startup();
  }

  if ((nextFlightState == INVALID || nextFlightState == STARTUP) && tdiff >= stageTickCounts[STARTUP]) {
    message.flight_state = PAD;
    startTick = message.tick;
    init_message_setup();
  }

  if (nextFlightState == PAD && message.tick % 100 == 0 && !sentThisTick && !connected) {
    sentThisTick = true;
    send_setup();
  }

  if (nextFlightState == PAD && buttonPressed && connected) {
    message.flight_state = BOOST;
    startTick = message.tick;
  }

  if (nextFlightState >= BOOST && nextFlightState < LANDED && tdiff >= stageTickCounts[nextFlightState] && connected) {
    message.flight_state++;
    startTick = message.tick;
  }

  if (sendFetch && connected) {
    sendFetch = false;
    send_fetch();
  }
  
  prevFlightState = nextFlightState;
}

void setupStages() {
  memset(stageTickCounts, 0, sizeof(stageTickCounts));
  // Values are from some old flight data
  stageTickCounts[STARTUP] = 596;
  stageTickCounts[BOOST] = 275;
  stageTickCounts[FAST] = 657;
  stageTickCounts[COAST] = 2117;
  stageTickCounts[DROGUE] = 2688;
  stageTickCounts[MAIN] = 907;
}

void init_message_startup() {
  memset(&message, 0, sizeof(message));
  message.flight_state = INVALID;
}

void init_message_setup() {
  message.command = SETUP;
  message.serial = 0x3713;
  message.flight = 0xADDE;
  message.accel = 0xEFBE;
  message.speed = 0xADDE;
  message.height = 0xEFBE;
  message.motor_number = 0x3713;
}

void init_clocks() {
  // Generic clock 0, is used for companion SPI connection (SERCOM1)
  // GCLK0 is also the main system clock so we don't change it or it will crash
  // GCLK1 is set up as the reference clock for the DFLL48 oscillator, don't change it

  // Load the factory clock calibration data
  uint32_t coarse = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  uint32_t fine = (*((uint32_t *) FUSES_DFLL48M_FINE_CAL_ADDR) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  // Generic clock 2, used for ticker clock
  // Setup with 48Mhz oscillator, divided by 12 for 4Mhz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(12);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(2);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void init_tick_timer() {
  // Clock source is GCLK2, setup with 4Mhz clock (see init_clocks)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN_GCLK2 |
                      GCLK_CLKCTRL_ID(GCM_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY);
  TC5->COUNT16.CTRLA.bit.ENABLE = 0x0;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
  TC5->COUNT16.CTRLA.bit.SWRST = 0x1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  NVIC_EnableIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0); // Highest priority

  TC5->COUNT16.CTRLA.bit.MODE = 0x0;        // 16-bit counter
  TC5->COUNT16.CTRLA.bit.WAVEGEN = 0x1;     // Match frequency generation (CC0 becomes TOP value)
  TC5->COUNT16.CTRLA.bit.PRESCALER = 0x5;   // DIV64 prescaler for 62.5kHz clock
  TC5->COUNT16.CTRLA.bit.PRESCSYNC = 0x0;   // Reset counter on next clock, not next prescaled clock
  TC5->COUNT16.CTRLBSET.bit.ONESHOT = 0x0;  // Continue counting on wraparound
  TC5->COUNT16.CTRLBSET.bit.DIR = 0x0;      // Count Up
  TC5->COUNT16.CC[0].reg = 625; // 10ms

  TC5->COUNT16.INTENSET.bit.OVF = 0x1;      // Enable overflow interrupt

  TC5->COUNT16.CTRLA.bit.ENABLE = 0x1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
}

void TC5_Handler() {
  TC5->COUNT16.INTFLAG.bit.OVF = 0x1; // Clear the overflow flag
  message.tick += 1;
  sentThisTick = false;
  if (message.flight_state >= PAD && message.flight_state < INVALID && connected) {
    sendFetch = true;
  }
}

void send_startup() {
  if (message.tick % 10 == 0){
    Serial.print(F("STARTUP"));
    Serial.println(message.tick);
    send_message(sizeof(message));
  }
}

void send_setup() {
  // Send setup message and try to get answer back
  if (message.tick % 100 != 0) {
    return;
  } 
  Serial.print(F("SETUP"));
  Serial.println(message.tick);
  message.command = SETUP;
  AltOSSetupReply received;
  send_message(sizeof(message) + sizeof(received));
  setupsSent++;
  for (int i = 0; i < sizeof(received); i++) {
    *((uint8_t *)&received + i) = *((uint8_t *)&rxBuffer + i + sizeof(message));
  }
  // print_raw(txBuffer, sizeof(txBuffer));
  // print_raw(rxBuffer, sizeof(rxBuffer));
  bool success = (uint16_t)(~received.board_id) == received.board_id_inverse;
  // if (setupsSent >= 10) {
  //   success = true;
  // }
  if (success) {
    Serial.printf("%d %d %d %d\n", 
                received.board_id, received.board_id_inverse, 
                received.update_period, received.channels);
    connected = true;
    channels = min(received.channels, 12);
  }
}

void send_fetch() {
  // Serial.println(F("FETCH"));
  message.command = FETCH;
  send_message(sizeof(message) + sizeof(uint16_t)*channels);
}

void send_notify() {
  Serial.println(F("NOTIFY"));
  message.command = NOTIFY;
  send_message(sizeof(message));
}

void send_message(size_t size) {
  memset(txBuffer, 0, sizeof(txBuffer));
  memset(rxBuffer, 0, sizeof(rxBuffer));
  for (int i = 0; i < sizeof(message); i++) {
    *((uint8_t *)&txBuffer + i) = *((uint8_t *)&message + i);
  }
  SPI.beginTransaction(spisettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(txBuffer, rxBuffer, size, true);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

// Prints the raw bytes of an object.
// Note that the Cortex M0 uses little-endian byte order.
void print_raw(void* obj, size_t size) {
  uint8_t* buf = reinterpret_cast<uint8_t *>(obj);
  for (uint16_t i = 0; i < size; i++) {
    Serial.printf("%02X%s", *(buf + i), (i + 1) % 8 == 0 ? "\n" : " ");
  }
  Serial.println();
}