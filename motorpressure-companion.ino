#include <SercomSPISlave.h>

#define CHANNELS 10
#define BOARD_ID 0x7
#define UPDATE_RATE 100 // 100Hz ticks

// WIRING (Adafruit Feather M0 Adalogger)
// Flight Computer MOSI <-> Feather PA12 (Labelled MISO)
// Flight Computer SCK  <-> Feather PB09 (Labelled A2)
// Flight Computer CS   <-> Feather PB10 (Labelled MOSI)
// Flight Computer MISO <-> Feather PB11 (Labelled SCK)
// Pressure sensor output <-> Feather PA02 (Labelled A0) (3.3V MAX!)

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

// The companion docs are incorrect, the message header is 16 bytes, not 8 bytes.
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
volatile uint16_t adcValue = 0;


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
  init_adc();
  Serial.println("ADC Initialized");
}

void loop() {
  if (newMessage) {
    Serial.println("Received Message");
    print_raw(&message, sizeof(message));
    Serial.printf("%d %d %d %d %d %d %d %d %d\n\n", message.command, message.flight_state, message.tick, 
                                                    message.serial, message.flight, message.accel, message.speed, 
                                                    message.height, message.motor_number);
    Serial.println(adcValue);
    newMessage = false;
  }
}

void init_adc() {
  // Configure I/O port - PA02 is PINCFG[2], PMUX[1].PMUXE
  PORT->Group[PORTA].DIRCLR.reg = PORT_PA02;
  PORT->Group[PORTA].PINCFG[2].bit.PMUXEN = 0x1; // Enable pin PA02
  PORT->Group[PORTA].PMUX[1].bit.PMUXE = 0x1; // Select peripheral function B (ADC input)

  // Reset ADC
  ADC->CTRLA.bit.ENABLE = 0x0;
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->CTRLA.bit.SWRST = 0x1;
  while (ADC->CTRLA.bit.SWRST || ADC->STATUS.bit.SYNCBUSY);

  // Configure ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) | // Enable ADC clock
                      GCLK_CLKCTRL_ID_ADC |
                      GCLK_CLKCTRL_CLKEN;
  NVIC_EnableIRQ(ADC_IRQn); // Enable interrupt handler
  NVIC_SetPriority(ADC_IRQn, 1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronisation

  // Load the factory calibration data
  // Code from https://blog.thea.codes/reading-analog-values-with-the-samd-adc/
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
  ADC->INPUTCTRL.bit.GAIN   = 0xF; // DIV2 gain due to voltage reference being 1/2 VCC
  ADC->INPUTCTRL.bit.MUXNEG = 0x18; // Internal ground as negative input
  ADC->INPUTCTRL.bit.MUXPOS = 0x00; // PA02 (AIN0) as positive input
  ADC->INTENSET.bit.RESRDY  = 0x1; // Enable result ready interrupt
  ADC->REFCTRL.bit.REFCOMP  = 0x1; // Enable voltage reference compensation
  ADC->REFCTRL.bit.REFSEL   = 0x2; // 3.3V Reference (1/2 VCC = 1.65V)
  ADC->CTRLB.bit.PRESCALER  = 0x6; // Prescaler 256
  ADC->CTRLB.bit.RESSEL     = 0x0; // 12-bit resolution mode
                                   // NOTE: Even though there is time for 16 bit oversampling, 
                                   // the signal may change rapidly and might be inaccurate that way.
  ADC->CTRLB.bit.CORREN     = 0x0; // Disable digital correction
  // ADC->CTRLB.bit.FREERUN    = 0x0; // One-shot conversion mode
  ADC->CTRLB.bit.FREERUN    = 0x1; // Freerun conversion mode
  ADC->CTRLB.bit.DIFFMODE   = 0x0; // Single-ended conversion
  ADC->AVGCTRL.bit.SAMPLENUM = 0x1; // Average 2 samples
  ADC->AVGCTRL.bit.ADJRES   = 0x1; // Divisor of 2 (for 2 samples)

  // Re-enable ADC
  ADC->CTRLA.bit.ENABLE = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void ADC_Handler() {
  // With prescaler 256, freerun mode, 2 sample average, conversion takes ~110us
  adcValue = ADC->RESULT.reg; // This will clear the INTFLAG.RESRDY interrupt flag
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
