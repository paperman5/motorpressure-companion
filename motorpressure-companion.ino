#include <SercomSPISlave.h>

#define CHANNELS 10
#define BOARD_ID 0x7
#define UPDATE_RATE 100 // 100Hz ticks

// WIRING (Adafruit Feather M0 Adalogger)
// Flight Computer MOSI <-> Feather PA12 (Labelled MISO)
// Flight Computer SCK  <-> Feather PB09 (Labelled A2)
// Flight Computer CS   <-> Feather PB10 (Labelled MOSI)
// Flight Computer MISO <-> Feather PB11 (Labelled SCK)

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

// Docs are incorrect, the message header is 16 bytes, not 8 bytes.
// There is some extra data after the flight number (from firmware source)
typedef struct {
  uint8_t   command;
  uint8_t   flight_state;
  uint16_t  tick;
  uint16_t  serial;
  uint16_t  flight; // End of companion docs
  int16_t		accel;  // From firmware source
	int16_t		speed;
	int16_t		height;
	uint16_t	motor_number;
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

Sercom4SPISlave SPISlave;
AltOSMessageHeader message;
AltOSSetupReply setupReply;
AltOSFetchReply fetchReply;
uint8_t rxIndex = 0;
uint8_t txIndex = 0;
volatile bool newMessage = false;

void handleStateChange(State oldState, State newState) {
  Serial.printf("State Change: %d -> %d\n", (int)oldState, (int)newState);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial started");

  setupReply.board_id = (uint16_t)BOARD_ID;
  setupReply.board_id_inverse = ~setupReply.board_id;
  setupReply.update_period = (uint8_t)UPDATE_RATE;
  setupReply.channels = (uint8_t)CHANNELS;
  message.command = 0;
  for (int i = 0; i < CHANNELS; i++) {
    fetchReply.data[i] = i;
  }
  Serial.printf("Setup reply initialized: %d %d %d %d\n", 
                setupReply.board_id, setupReply.board_id_inverse, 
                setupReply.update_period, setupReply.channels);

  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA12, SPISlave.SCK_Pins::PB09, SPISlave.SS_Pins::PB10, SPISlave.MISO_Pins::PB11);
  Serial.println("SERCOM4 SPI slave initialized");
}

void loop() {
  if (newMessage) {
    Serial.println("Received Message");
    print_raw(&message, sizeof(message));
    Serial.printf("%d %d %d %d %d %d %d %d %d\n\n", message.command, message.flight_state, message.tick, 
                                                    message.serial, message.flight, message.accel, message.speed, 
                                                    message.height, message.motor_number);
    newMessage = false;
  }
}

void SERCOM4_Handler()
/*
Reference: Atmel-42181G-SAM-D21_Datasheet section 26.8.6 on page 503
*/
{
  uint8_t interrupts = SERCOM4->SPI.INTFLAG.reg; // Read SPI interrupt register

  if (interrupts & (1 << 7)) // 10000000 = bit 7 = ERROR
  {
    SERCOM4->SPI.INTFLAG.bit.ERROR = 1; // Clear Error Interrupt
  }
  
  // Slave Select Low interrupt: Start of a new data transmission
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
    rxIndex = 0;
    txIndex = 0;
    SERCOM4->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }
  
  // Data Received Complete interrupt: We have received a single byte
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
  {
    uint8_t data = (uint8_t)SERCOM4->SPI.DATA.reg; // Read data register. This clears the RXC interrupt flag!
    if (rxIndex < sizeof(message)) {
      *((uint8_t *)&message + rxIndex) = data;
    }
    rxIndex++;
  }
  
  // Data Transmit Complete interrupt: Overall data transmission is complete
  if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
  {
    // Serial.printf("%d %d", rxIndex, txIndex);
    rxIndex = 0;
    txIndex = 0;
    newMessage = true;
    SERCOM4->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }
  
  // Data Register Empty interrupt: Ready to transmit a single byte
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
  {
    // Setting the DATA register clears the DRE flag
    // NOTE: The data loaded into the register here will be sent with the next byte read,
    //       so each sent byte needs to be loaded 1 position earlier. (Page 484)
    if (rxIndex >= sizeof(message) - 1) {
      switch (message.command) {
        case 1:
          SERCOM4->SPI.DATA.reg = txIndex < sizeof(setupReply) ? *((uint8_t *)&setupReply + txIndex++) : 0x00;
          break;
        case 2:
          SERCOM4->SPI.DATA.reg = txIndex < sizeof(fetchReply) ? *((uint8_t *)&fetchReply + txIndex++) : 0x00;
          break;
        case 3:
          SERCOM4->SPI.DATA.reg = 0x00;
          break;
        default:
          SERCOM4->SPI.DATA.reg = 0x00;
          break;
      }
    }
    else {
      SERCOM4->SPI.DATA.reg = 0x00;
    }
  }
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
