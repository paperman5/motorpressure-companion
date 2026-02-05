#include <SPISlave.h>

#define PIN_SCK 14
#define PIN_MOSI 15 // AKA TX
#define PIN_MISO 8  // AKA RX
#define PIN_CS 29
#define SPI_RATE 1000000
#define CHANNELS 10U
#define BOARD_ID 7U
#define UPDATE_RATE 100U // 100Hz ticks

enum Command {
  SETUP = 1,
  FETCH = 2,
  NOTIFY = 3
};
enum State { // From firmware kernel source `ao_flight.h`
  STARTUP = 0,
  IDLE = 1,
  PAD = 2,
  BOOST = 3,
  FAST = 4,
  COAST = 5,
  DROGUE = 6,
  MAIN = 7,
  LANDED = 8,
  INVALID = 9,
  TEST = 10
};

typedef struct {
  uint8_t command;
  uint8_t flight_state;
  uint16_t tick;
  uint16_t serial;
  uint16_t flight;
} AltOSMessageHeader;

typedef struct {
  uint16_t board_id;
  uint16_t board_id_inverse;
  uint8_t update_period;
  uint8_t channels;
} AltOSSetupReply;

typedef struct {
  uint16_t data[CHANNELS];
} AltOSFetchReply;


uint8_t recvBuffer[32]; // From docs: Telemetry messages are 32 bytes
AltOSMessageHeader message;
AltOSSetupReply setupReply;
AltOSFetchReply fetchReply;
uint16_t testReply = 0;

// Called when data has been read from a controller SPI message
void msgReceivedCallback(uint8_t *data, size_t len) {
  State prevState = static_cast<State>(message.flight_state);
  
  // Overwrite `message` with raw bytes from received data
  for (int i = 0; i < min(len, sizeof(message)); i++) {
    *((uint8_t*)&message + i) = data[i]; 
  }
  Command newCommand = static_cast<Command>(message.command);
  State newState = static_cast<State>(message.flight_state);
  
  // SPI send is done immediately after this callback, so
  // fill the send buffer here to send the most up-to-date info.
  switch (newCommand) {
    case SETUP:
      SPISlave.setData((uint8_t *)&setupReply, sizeof(setupReply));
      Serial.printf("Sending setup reply: board_id = %d, update_period = %d, channels = %d\n", \
                      setupReply.board_id, setupReply.update_period, setupReply.channels);
      break;
    case FETCH:
      //TESTING REPLY
      for (int i = 0; i < CHANNELS; i++) {
        fetchReply.data[i] = testReply++;
      }
      SPISlave.setData((uint8_t *)fetchReply.data, sizeof(fetchReply.data));
      break;
    case NOTIFY:
      if (prevState != newState) {
        handleStateChange(prevState, newState);
      }
      break;
    default:
      break;
  }
  memset(recvBuffer, 0, sizeof(recvBuffer));
}

// Called AFTER reply has already been sent to controller
void msgSentCallback() {
  // Nothing to do yet
}

void handleStateChange(State oldState, State newState) {
  Serial.printf("State Change: %d -> %d\n", (int)oldState, (int)newState);
}

void setup() {
  digitalWrite(PIN_CS, HIGH); // TeleMega will not boot unless this pin is high when it is switched on
  delay(3000);

  Serial.println("Message Setup");
  setupReply.board_id = (uint16_t)BOARD_ID;
  setupReply.board_id_inverse = ~setupReply.board_id;
  setupReply.update_period = (uint8_t)UPDATE_RATE;
  setupReply.channels = (uint8_t)CHANNELS;
  memset(recvBuffer, 0, sizeof(recvBuffer));
  memset(fetchReply.data, 0, sizeof(fetchReply.data));

  Serial.println("SPI Setup");
  SPISlave.setRX(PIN_MISO);
  SPISlave.setCS(PIN_CS);
  SPISlave.setSCK(PIN_SCK);
  SPISlave.setTX(PIN_MOSI);
  SPISlave.onDataRecv(msgReceivedCallback);
  SPISlave.onDataSent(msgSentCallback);
  SPISlave.begin(SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0));
  Serial.println("Setup Complete");
  delay(3000);
}

void loop() {

}
