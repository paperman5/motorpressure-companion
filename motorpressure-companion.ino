#include <SercomSPISlave.h>

#define CHANNELS 10
#define BOARD_ID 0x7
#define UPDATE_RATE 100 // 100Hz ticks
#define ADC_RATE 4 // 50kHz increments, min 1, max 5

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
volatile AltOSFetchReply fetchReply;
uint8_t rxIndex = 0;
uint8_t txIndex = 0;
volatile unsigned long adcIndex = 0;
volatile uint16_t adcSamples[CHANNELS];
volatile bool newMessage = false;
unsigned long prevTime = 0;
unsigned long diffTime = 0;


/****** MAIN PROGRAM ******/

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
  init_clocks();
  Serial.println("System clocks initialized");
  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA12, SPISlave.SCK_Pins::PB09, SPISlave.SS_Pins::PB10, SPISlave.MISO_Pins::PB11);
  Serial.println("SERCOM4 SPI slave initialized");
  init_adc();
  Serial.println("ADC Initialized");
  init_adc_timer();
  Serial.println("Timer Initialized");
  // TODO: Setup brownout detector?
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
  if (adcIndex >= 50000 * ADC_RATE) {
    unsigned long t = micros();
    Serial.printf("%d: %d\n", adcIndex, t - prevTime);
    prevTime = t;
    adcIndex = 0;
  }
}

/****** PERIPHERAL REGISTER INITIALIZATION ******/

void init_clocks() {
  // Generic clock 0, is used for companion SPI connection (SERCOM4)
  // GCLK0 is also the main system clock so we don't change it or it will crash

  // Generic clock 1, used for ADC clock
  // Setup with 48Mhz oscillator, divided by 3 for 16Mhz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // Generic clock 2, used for Timer clock (TC5)
  // Setup with 48Mhz oscillator, divided by 3 for 16Mhz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(2);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
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
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) |  // Enable ADC clock
                      GCLK_CLKCTRL_GEN_GCLK1 |    // Use 16MHz GCLK1
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
  ADC->REFCTRL.bit.REFSEL   = 0x2; // 1.65V Reference (1/2 VCC)
  ADC->CTRLB.bit.PRESCALER  = 0x1; // Prescaler 8
                                   // NOTE: Max ADC clock is ~2.1MHz, use DIV8 to get 2.0MHz (48/3/8)
                                   // ALSO NOTE: Max measurement impedance is ~4.5MOhm
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
  // Clock source is GCLK2, setup with 8Mhz clock (see init_clocks)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN_GCLK2 | 
                      GCLK_CLKCTRL_ID(GCM_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY);
  TC5->COUNT16.CTRLA.bit.ENABLE = 0x0;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
  TC5->COUNT16.CTRLA.bit.SWRST = 0x1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  NVIC_EnableIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 3);

  TC5->COUNT16.CTRLA.bit.MODE = 0x0;        // 16-bit counter
  TC5->COUNT16.CTRLA.bit.WAVEGEN = 0x1;     // Match frequency generation (CC0 becomes TOP value)
  TC5->COUNT16.CTRLA.bit.PRESCALER = 0x2;   // DIV4 prescaler for 4MHz clock
  TC5->COUNT16.CTRLA.bit.PRESCSYNC = 0x0;   // Reset counter on next clock, not next prescaled clock
  TC5->COUNT16.CTRLBSET.bit.ONESHOT = 0x0;  // Continue counting on wraparound
  TC5->COUNT16.CTRLBSET.bit.DIR = 0x0;      // Count Up
  TC5->COUNT16.CC[0].reg = (uint16_t) ((80 / max(1, min(ADC_RATE, 5))) - 1);

  TC5->COUNT16.INTENSET.bit.OVF = 0x1;      // Enable overflow interrupt

  TC5->COUNT16.CTRLA.bit.ENABLE = 0x1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
}

// void init_timer() {
//   // Clock source is GCLK2, setup with 8Mhz clock (see init_clocks)
//   GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
//                       GCLK_CLKCTRL_GEN_GCLK2 | 
//                       GCLK_CLKCTRL_ID(GCM_RTC);
//   while (GCLK->STATUS.bit.SYNCBUSY);
//   // Reset RTC
//   RTC->MODE0.CTRL.bit.ENABLE = 0x0;
//   while (RTC->MODE0.STATUS.bit.SYNCBUSY);
//   RTC->MODE0.CTRL.bit.SWRST = 0x1;
//   while (RTC->MODE0.STATUS.bit.SYNCBUSY);

//   NVIC_EnableIRQ(RTC_IRQn); // Enable interrupts
//   NVIC_SetPriority(RTC_IRQn, 0);

//   RTC->MODE0.CTRL.bit.MODE = 0x0;       // Mode 0 (32 bit counter)
//   RTC->MODE0.CTRL.bit.PRESCALER = 0x3;  // DIV8
//   RTC->MODE0.CTRL.bit.MATCHCLR = 0x1;   // Clear timer on compare match (reset to 0)
//   RTC->MODE0.COMP[0].reg = (uint32_t) 20 * max(1, min(ADC_RATE, 5)) - 1;
//   RTC->MODE0.INTENSET.bit.CMP0 = 0x1;   // Compare match interrupt enabled
//                                         // Interrupt is raised on the next clock cycle
//   // RTC->MODE0.FREQCORR.bit.SIGN = 0x1;
//   // RTC->MODE0.FREQCORR.bit.VALUE = 0x7F;
//   RTC->MODE0.CTRL.bit.ENABLE = 0x1;
//   while (RTC->MODE0.STATUS.bit.SYNCBUSY);
// }

/****** INTERRUPT HANDLERS ******/

// void RTC_Handler() {
//   RTC->MODE0.INTFLAG.bit.CMP0 = 0x1; // Clear the compare match interrupt
//   // TODO: Make sure the RTC is actually ticking at the correct rate.
//   // w/ 8MHz clock and 1s sample rate, it appears to tick at 1003ms. (x1.00259690)
//   // Not sure if that is an issue with the timer setup or calibration.
// }

void TC5_Handler() {
  TC5->COUNT16.INTFLAG.bit.OVF = 0x1; // Clear the overflow flag
  ADC->SWTRIG.bit.START = 0x1; // Start an ADC conversion
}

void ADC_Handler() {
  // With prescaler 32, freerun mode, 2 sample average, conversion takes ~54us
  uint16_t adcValue = ADC->RESULT.reg; // This will clear the INTFLAG.RESRDY interrupt flag
  adcIndex++;
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

    // // Start ADC sampling for the next fetch request
    // if (message.command == 2 && !rtc_interrupts_enabled()) {
    //   start_rtc();
    // }
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

// void enable_rtc_interrupts() {
//   RTC->MODE0.INTENSET.bit.CMP0 = 0x1;
// }

// void disable_rtc_interrupts() {
//   RTC->MODE0.INTENCLR.bit.CMP0 = 0x1;
// }

// bool rtc_interrupts_enabled() {
//   return RTC->MODE0.INTENSET.bit.CMP0;
// }

// void start_rtc() {
//   RTC->MODE0.COUNT.reg = 0x0;
//   enable_rtc_interrupts();
//   while (RTC->MODE0.STATUS.bit.SYNCBUSY); // COUNT register needs sync
// }
