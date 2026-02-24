// #define REGISTER_DEBUG
// #define ADC_TEST_VALUES // Replaces ADC values with a monotonically increasing value for testing
#define TEST_FLIGHT_TRIGGER

#include <SPI.h>
#include <FreeStack.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SdFatDebugConfig.h>
#include <sdios.h>
#include <SercomSPISlave.h>
#ifdef REGISTER_DEBUG
  #include <ZeroRegs.h>
#endif

// WIRING (Adafruit Feather M0 Adalogger)
// Flight Computer MOSI <-> Feather PA16 (Labelled 11)
// Flight Computer SCK  <-> Feather PA17 (Labelled 13)
// Flight Computer CS   <-> Feather PA18 (Labelled 10)
// Flight Computer MISO <-> Feather PA19 (Labelled 12)
// Pressure sensor output <-> Feather PA02 (Labelled A0) (3.3V MAX!)

#define CHANNELS 12
#define BOARD_ID 0x7
#define UPDATE_RATE 1 // Companion port update rate, 100Hz ticks
#define ADC_RATE 10 // 1kHz increments, min 1, max 12 (1-12kHz)
#define FIFO_SIZE 1024
#define SD_MAX_CLOCK 24 // Mhz
#define SD_MIN_CLOCK 8  // Mhz
#define SD_FAT_TYPE 1 // 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define PIN_SD_CS 4
#define PIN_SD_LED 8
#define ERR_PATTERN_LENGTH 150 // millis between morse code beeps
#define DATA_FILE_PREALLOCATE ADC_RATE * 1000 * 4 * 30  // 4 bytes @ ADC_RATE for 30s
#define SD_WRITE_CHUNK_SIZE 512

#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

enum Command {
  SETUP = 1,
  FETCH = 2,
  NOTIFY = 3
};
enum FlightState { // From firmware kernel source `ao_flight.h`
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
enum Error {
  SD_NOT_FOUND = 0,
  SD_FILE_CREATE_ERR = 1,
  SD_FILE_CLOSE_ERR = 2
};
enum ProgramState {
  DISCONNECTED = 0,
  MONITORING = 1,
  RECORDING = 2,
  DONE = 3,
  ERROR = -1
};

// The companion docs are incorrect, the message header is 16 bytes, not 8 bytes.
// There is some extra data after the flight number (from firmware source)
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

typedef struct {
  volatile uint16_t data[CHANNELS];
} AltOSFetchReply;

typedef struct {
  uint16_t tick;
  uint16_t value;
} data_t;

Sercom1SPISlave SPISlave;
AltOSMessageHeader message;
AltOSSetupReply setupReply;
AltOSFetchReply fetchReply;
volatile ProgramState state = DISCONNECTED;
ProgramState prevState = DISCONNECTED;
sd_t sd;
file_t dataFile;
bool fileCreated = false;
uint8_t rxIndex = 0;
uint8_t txIndex = 0;
uint8_t transmitIndex = 0;
uint16_t dataIndex = 0;
volatile data_t currentData;
// NOTE: `volatile` before pointer type means value at the pointer is volatile,
// `volatile` after pointer type means pointer itself is volatile
volatile data_t dataFifo0[FIFO_SIZE];
volatile data_t dataFifo1[FIFO_SIZE];
volatile uint16_t transmitBuffer[CHANNELS * 2];
volatile data_t* volatile currentDataBuffer = dataFifo0;
volatile data_t* volatile dataBufferToWrite = dataFifo1;
volatile bool doDataWrite = false;
volatile bool sdCardIsWriting = false;
volatile bool sdCardOverrun = false;
volatile bool newMessage = false;
#ifdef ADC_TEST_VALUES
volatile uint16_t adcIndex = 0;
#endif

uint16_t writeCount = 0;
uint32_t loopCount = 0;
uint32_t loopCount2 = 0;


/****** MAIN PROGRAM ******/

void setup() {
  pinMode(PIN_SD_LED, OUTPUT);
  digitalWrite(PIN_SD_LED, LOW);
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Serial started"));

  setupReply.board_id = (uint16_t)BOARD_ID;
  setupReply.board_id_inverse = ~setupReply.board_id;
  setupReply.update_period = (uint8_t)UPDATE_RATE;
  setupReply.channels = (uint8_t)CHANNELS;
  message.command = 0;
  for (int i = 0; i < CHANNELS; i++) {
    fetchReply.data[i] = i;
  }
  memset_volatile(dataFifo0, 0, sizeof(dataFifo0));
  memset_volatile(dataFifo1, 0, sizeof(dataFifo1));
  Serial.print(F("Setup reply initialized: "));
  Serial.printf("%d %d %d %d\n", 
                setupReply.board_id, setupReply.board_id_inverse, 
                setupReply.update_period, setupReply.channels);
  
  init_clocks();
  Serial.println(F("System clocks initialized"));
  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA16, SPISlave.SCK_Pins::PA17, SPISlave.SS_Pins::PA18, SPISlave.MISO_Pins::PA19);
  Serial.println(F("SERCOM1 SPI slave initialized"));
  init_adc();
  Serial.println(F("ADC Initialized"));
  init_adc_timer();
  Serial.println(F("Timer Initialized"));
  sd_card_init();
  Serial.println(F("SD Card Initialized"));
  // TODO: Setup brownout detector?
  Serial.print(F("Free Stack: "));
  Serial.printf("%d bytes\n", FreeStack());
#ifdef REGISTER_DEBUG
  ZeroRegOptions opts = { Serial, false };
  printZeroRegs(opts);
#endif
  char pattern[3];
  memset(pattern, 1, sizeof(pattern));
  err_blink_pattern(pattern, sizeof(pattern));
}

void loop() {
  ProgramState newState = state; // Store state in var so it stays constant through the loop
                                 // If state is to be changed in loop(), change it using `state` not `newState`
  // if (newMessage) {
  //   Serial.println(F("Received Message"));
  //   print_raw(&message, sizeof(message));
  //   Serial.printf("%d %d %d %d %d %d %d %d %d\n\n", message.command, message.flight_state, message.tick, 
  //                                                   message.serial, message.flight, message.accel, message.speed, 
  //                                                   message.height, message.motor_number);
  //   newMessage = false;
  // }
  if (prevState != newState) {
    handle_state_change(prevState, newState);
  }

#ifdef TEST_FLIGHT_TRIGGER
  //TESTING - Trigger flight after 5M loops in MONITORING
  if (newState == MONITORING) {
    loopCount2++;
    if (loopCount2 >= 5000000) {
      state = RECORDING;
    }
  }
#endif

  if (newState == RECORDING && doDataWrite) {
    // Serial.println(F("WRITE"));
    doDataWrite = false;
    sdCardIsWriting = true;
    sd_write_buffer_to_file(&dataFile, dataBufferToWrite, sizeof(dataFifo0));
    sdCardIsWriting = false;
    writeCount++;
    if (writeCount >= 150) {
      state = DONE;
    }
  }
  if (sdCardOverrun) {
    sdCardOverrun = false;
    Serial.println(F("OVERRUN"));
  }
  // if (loopCount >= 300000) {
  //   loopCount = 0;
  //   Serial.println(newState);
  // }
  prevState = newState;
  loopCount++;
}

void handle_state_change(ProgramState oldState, ProgramState newState) {
  Serial.printf("PROGRAM STATE: %d -> %d\n", oldState, newState);
  if (newState == MONITORING && !fileCreated) {
    Serial.println(F("Creating flight file"));
    if (!create_flight_file((uint8_t)message.flight)) {
      crash_with_error(SD_FILE_CREATE_ERR);
    }
    fileCreated = true;
    Serial.println(F("Flight file created on SD card"));
  }
  else if (newState == RECORDING) {

  }
  else if (newState == DONE) {
    if (!finalize_file(&dataFile)) {
      crash_with_error(SD_FILE_CLOSE_ERR);
    }
    Serial.println(F("File Finalized, shutting down SD card"));
    sd.end();
    digitalWrite(PIN_SD_LED, HIGH);
  }
}

/****** PERIPHERAL REGISTER INITIALIZATION ******/

void init_clocks() {
  // Generic clock 0, is used for companion SPI connection (SERCOM1)
  // GCLK0 is also the main system clock so we don't change it or it will crash
  // GCLK1 is set up as the reference clock for the DFLL48 oscillator, don't change it

  // Load the factory clock calibration data
  uint32_t coarse = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  uint32_t fine = (*((uint32_t *) FUSES_DFLL48M_FINE_CAL_ADDR) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  // Generic clock 2, used for ADC clock
  // Setup with 48Mhz oscillator, divided by 3 for 16Mhz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(2);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // Generic clock 3, used for Timer clock (TC5)
  // Setup with 48Mhz oscillator, divided by 4 for 12Mhz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void init_adc() {
  // Configure I/O port - PA02 is PINCFG[2], PMUX[1].PMUXE
  PORT->Group[PORTA].DIRCLR.reg = PORT_PA02;
  PORT->Group[PORTA].PINCFG[2].bit.INEN = 0x1;
  PORT->Group[PORTA].PINCFG[2].bit.PMUXEN = 0x1; // Enable pin PA02
  PORT->Group[PORTA].PMUX[1].bit.PMUXE = 0x1; // Select peripheral function B (ADC input)

  // // Configure I/O port - PB08 is GroupB, PINCFG[8], PMUX[4].PMUXE
  // PORT->Group[PORTB].DIRCLR.reg = PORT_PB08;
  // PORT->Group[PORTB].PINCFG[8].bit.INEN = 0x1;
  // PORT->Group[PORTB].PINCFG[8].bit.PMUXEN = 0x1; // Enable pin PA02
  // PORT->Group[PORTB].PMUX[4].bit.PMUXE = 0x1; // Select peripheral function B (ADC input)

  // Reset ADC
  ADC->CTRLA.bit.ENABLE = 0x0;
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->CTRLA.bit.SWRST = 0x1;
  while (ADC->CTRLA.bit.SWRST || ADC->STATUS.bit.SYNCBUSY);

  // Configure ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) |  // Enable ADC clock
                      GCLK_CLKCTRL_GEN_GCLK2 |    // Use 16MHz GCLK2
                      GCLK_CLKCTRL_CLKEN;
  NVIC_EnableIRQ(ADC_IRQn); // Enable interrupt handler
  NVIC_SetPriority(ADC_IRQn, 1); // 2nd-highest priority
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
  // ADC->INPUTCTRL.bit.MUXPOS = 0x02; // PB08 (AIN2) as positive input
  ADC->INTENSET.bit.RESRDY  = 0x1; // Enable result ready interrupt
  ADC->REFCTRL.bit.REFCOMP  = 0x1; // Enable voltage reference compensation
  ADC->REFCTRL.bit.REFSEL   = 0x2; // 1.65V Reference (1/2 VCC)
  ADC->CTRLB.bit.PRESCALER  = 0x4; // Prescaler 64
                                   // NOTE: Max ADC clock is ~2.1MHz, use DIV64 to get 250kHz (48/3/64)
                                   // ALSO NOTE: Max input impedance is ~60MOhm
                                   // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
  ADC->CTRLB.bit.RESSEL     = 0x0; // 12-bit resolution mode
  ADC->CTRLB.bit.CORREN     = 0x0; // Disable digital correction
  ADC->CTRLB.bit.FREERUN    = 0x0; // One-shot conversion mode (Triggered from timer)
  // ADC->CTRLB.bit.FREERUN    = 0x1; // Freerun conversion mode
  ADC->CTRLB.bit.DIFFMODE   = 0x0; // Single-ended conversion
  // ADC->AVGCTRL.bit.SAMPLENUM = 0x1; // Average 2 samples
  // ADC->AVGCTRL.bit.ADJRES   = 0x1; // Divisor of 2 (for 2 samples)

  // Re-enable ADC
  ADC->CTRLA.bit.ENABLE = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void init_adc_timer() {
  // Clock source is GCLK3, setup with 12Mhz clock (see init_clocks)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN_GCLK3 |
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
  TC5->COUNT16.CTRLA.bit.PRESCALER = 0x0;   // DIV1 prescaler for 12MHz clock
  TC5->COUNT16.CTRLA.bit.PRESCSYNC = 0x0;   // Reset counter on next clock, not next prescaled clock
  TC5->COUNT16.CTRLBSET.bit.ONESHOT = 0x0;  // Continue counting on wraparound
  TC5->COUNT16.CTRLBSET.bit.DIR = 0x0;      // Count Up
  TC5->COUNT16.CC[0].reg = (uint16_t) ((12000 / max(1, min(ADC_RATE, 12))) - 1);

  TC5->COUNT16.INTENSET.bit.OVF = 0x1;      // Enable overflow interrupt

  TC5->COUNT16.CTRLA.bit.ENABLE = 0x1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
}

void sd_card_init() {
  uint8_t clock = SD_MAX_CLOCK;
  SdSpiConfig* config;
  bool success = false;
  while (!success && clock >= SD_MIN_CLOCK) {
    config = new SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(clock));
    success = sd.begin(*config);
    clock--;
  }
  if (clock < SD_MIN_CLOCK) {
    Serial.println(F("Error initializing SD Card"));
    crash_with_error(SD_NOT_FOUND);
  }
  Serial.print(F("SD card initialized at "));
  Serial.print(++clock);
  Serial.println(F("MHz"));
  NVIC_SetPriority(SERCOM4_IRQn, 3); // Lowest Priority
  // NVIC_SetPriority(SERCOM4_IRQn, 0); // Highest Priority

  sd.ls(LS_SIZE);
}

/****** INTERRUPT HANDLERS ******/

void TC5_Handler() {
  TC5->COUNT16.INTFLAG.bit.OVF = 0x1; // Clear the overflow flag
  ADC->SWTRIG.bit.START = 0x1; // Start an ADC conversion
}

void ADC_Handler() {
  uint16_t adcValue = ADC->RESULT.reg; // This will clear the INTFLAG.RESRDY interrupt flag
#ifdef ADC_TEST_VALUES
  adcValue = (adcIndex++ * 3) % 4096;
#endif
  currentData.value = adcValue;
  currentDataBuffer[dataIndex].tick = currentData.tick;
  currentDataBuffer[dataIndex++].value = adcValue;
  if (dataIndex >= FIFO_SIZE) {
    dataIndex = 0;
    if (!doDataWrite && state == RECORDING) {
      // SD card write is handled in main loop so that interrupts can continue
      doDataWrite = true;
      dataBufferToWrite = currentDataBuffer;
    }
    else if (state == RECORDING) {
      sdCardOverrun = true;
    }
    currentDataBuffer = currentDataBuffer == dataFifo0 ? dataFifo1 : dataFifo0;
  }
}

// Reference: Atmel-42181G-SAM-D21_Datasheet section 26.8.6 on page 503
void SERCOM1_Handler() {
  uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; // Read SPI interrupt register

  if (interrupts & (1 << 7)) // 10000000 = bit 7 = ERROR
  {
    SERCOM1->SPI.INTFLAG.bit.ERROR = 1; // Clear Error Interrupt
  }
  
  // Slave Select Low interrupt: Start of a new data transmission
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
    // Serial.println(F("SSL"));
    rxIndex = 0;
    txIndex = 0;
    SERCOM1->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }
  
  // Data Received Complete interrupt: We have received a single byte
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
  {
    // Serial.println(F("DRC"));
    uint8_t data = (uint8_t)SERCOM1->SPI.DATA.reg; // Read data register. This clears the RXC interrupt flag!
    if (rxIndex < sizeof(message)) {
      *((uint8_t *)&message + rxIndex) = data;
    }
    rxIndex++;
  }
  
  // Data Transmit Complete interrupt: Overall data transmission is complete
  if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
  {
    // Serial.println(F("DTC"));
    // Serial.printf("%d %d", rxIndex, txIndex);
    rxIndex = 0;
    txIndex = 0;
    newMessage = true;
    SERCOM1->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt

    if (message.command == SETUP && state == DISCONNECTED) {
      state = MONITORING;
    }

    currentData.tick = message.tick;
  }
  
  // Data Register Empty interrupt: Ready to transmit a single byte
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
  {
    // Serial.println(F("DRE"));
    // Setting the DATA register clears the DRE flag
    // NOTE: The data loaded into the register here will be sent with the next byte read,
    //       so each sent byte needs to be loaded 1 position earlier. (Page 484)
    if (rxIndex >= sizeof(message) - 1) {
      switch (message.command) {
        case SETUP:
          SERCOM1->SPI.DATA.reg = txIndex < sizeof(setupReply) ? *((uint8_t *)&setupReply + txIndex++) : 0x00;
          break;
        case FETCH:
          SERCOM1->SPI.DATA.reg = txIndex < sizeof(fetchReply) ? *((uint8_t *)&fetchReply + txIndex++) : 0x00;
          break;
        case NOTIFY:
          SERCOM1->SPI.DATA.reg = 0x00;
          break;
        default:
          SERCOM1->SPI.DATA.reg = 0x00;
          break;
      }
    }
    else {
      SERCOM1->SPI.DATA.reg = 0x00;
    }
  }
}

/****** UTILITY ******/

// Prints the raw bytes of an object.
// Note that the Cortex M0 uses little-endian byte order.
void print_raw(void* obj, size_t size) {
  uint8_t* buf = reinterpret_cast<uint8_t *>(obj);
  for (uint16_t i = 0; i < size; i++) {
    Serial.printf("%02X%s", *(buf + i), (i + 1) % 8 == 0 ? "\n" : " ");
  }
  Serial.println();
}

// `memset()` doesn't work on volatile arrays
// https://stackoverflow.com/a/43897952
void memset_volatile(volatile void *s, char c, size_t n) {
    volatile char *p = reinterpret_cast<volatile char *>(s);
    while (n-- > 0) {
        *p++ = c;
    }
}

// Stores a 12bit uint (the 12 lower bits of a 16bit number) inside of a larger 8bit array as if
// it were a native array of 12bit uints, such that each 12bit value is big-endian contiguous
// on half-byte boundaries. 8bit array should have size `12bit_size * 3 / 2`.
// Example: numbers `0x123` (little-endian `0x2301`) and `0x456` (LE `0x5604`)
// at 12bit indices 0 and 1 become `0x12 34 56` in the underlying 8-bit array.
inline void store_12bit(volatile uint8_t* arr8, uint16_t index12, uint16_t value12) {
  uint32_t i8 = index12 * 3 / 2;
  uint8_t* p8 = (uint8_t*)&value12;
  if (index12 % 2) { // ODD
    arr8[i8] &= 0xF0;
    arr8[i8] |= *(p8 + 1) & 0x0F;
    arr8[i8+1] = *p8;
  }
  else {
    arr8[i8] = *(p8 + 1) << 4 | *p8 >> 4;
    arr8[i8+1] &= 0x0F;
    arr8[i8+1] |= *p8 << 4;
  }
}

// Read a 12bit uint stored with `store_12bit` into a 16bit uint.
inline uint16_t read_12bit(volatile uint8_t* arr8, uint32_t index12) {
  uint32_t i8 = index12 * 3 / 2;
  if (index12 % 2) {
    return (arr8[i8] & 0x0F) << 8 | arr8[i8+1];
  }
  else {
    return (arr8[i8] << 8 | arr8[i8+1]) >> 4;
  }
}

// Write a buffer to the SD card.
void sd_write_buffer_to_file(file_t* file, volatile void* buffer, size_t size) {
  void* buf = const_cast<void *>(buffer);
  uint8_t* pWriteBuffer = reinterpret_cast<uint8_t *>(buf);
  uint16_t fifoIndex = 0;
  while (sd.card()->isBusy()) {}
  while (fifoIndex < size) {
    uint16_t nb = min(size - fifoIndex, SD_WRITE_CHUNK_SIZE);
    if (nb != file->write(pWriteBuffer + fifoIndex, nb)) {
      break;
    }
    fifoIndex += nb;
  }
}

// Create and preallocate space for the data file for this flight.
bool create_flight_file(uint8_t flight) {
  char dataFileName[40];
  memset(dataFileName, 0, sizeof(dataFileName));
  sprintf(dataFileName, "Flight%02d.bin", flight);
  uint8_t i = 1;
  while (sd.exists(dataFileName) && i < 100) {
    sprintf(dataFileName, "Flight%02d_%02d.bin", flight, i++);
  }
  return create_file(&dataFile, dataFileName, DATA_FILE_PREALLOCATE);
}

inline bool create_file(file_t* fileObj, char* name, uint32_t preallocate) {
  return fileObj->close() && fileObj->open(name, O_WRONLY | O_CREAT) && fileObj->preAllocate(preallocate);
}

inline bool remove_file(file_t* fileObj) {
  return fileObj->remove();
}

inline bool finalize_file(file_t* fileObj) {
  return fileObj->truncate() && fileObj->close();
}

/****** ERRORS ******/

void disable_all_interrupts() {
  ADC->INTENCLR.bit.RESRDY = 0x1;
  TC5->COUNT16.INTENCLR.bit.OVF = 0x1;
  SERCOM1->SPI.INTENCLR.bit.SSL = 0x1;
  SERCOM1->SPI.INTENCLR.bit.RXC = 0x1;
  SERCOM1->SPI.INTENCLR.bit.TXC = 0x1;
  SERCOM1->SPI.INTENCLR.bit.ERROR = 0x1;
  SERCOM1->SPI.INTENCLR.bit.DRE = 0x1;
}

void crash_with_error(Error err) {
  disable_all_interrupts();
  state = ERROR;
  char pattern[5];
  memset(pattern, 0, sizeof(pattern));
  pattern[0] = 1;
  if (err == SD_NOT_FOUND) {
    // 'N' for 'Not Found'
    pattern[0] = 3;
    pattern[1] = 1;
  }
  else if (err == SD_FILE_CREATE_ERR) {
    // 'O' for 'Open'
    pattern[0] = 3;
    pattern[1] = 3;
    pattern[2] = 3;
  }
  else if (err == SD_FILE_CLOSE_ERR) {
    // 'C' for 'Close'
    pattern[0] = 3;
    pattern[1] = 1;
    pattern[2] = 3;
    pattern[3] = 1;
  }
  while (true) {
    err_blink_pattern(pattern, sizeof(pattern));
  }
}

void err_blink_pattern(char* pattern, size_t size) {
  for (int i = 0; i < size; i++) {
    if (pattern[i] == 0) {
      break;
    }
    digitalWrite(PIN_SD_LED, HIGH);
    delay(ERR_PATTERN_LENGTH * pattern[i]);
    digitalWrite(PIN_SD_LED, LOW);
    delay(ERR_PATTERN_LENGTH);
  }
  delay(ERR_PATTERN_LENGTH * 6);
}