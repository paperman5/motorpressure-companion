#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SdFatDebugConfig.h>
#include <sdios.h>

#define SD_FAT_TYPE 1

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

sd_t sd;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  uint8_t mhz = 50;
  SdSpiConfig* config = new SdSpiConfig(4, DEDICATED_SPI, SD_SCK_MHZ(mhz));
  Serial.println(mhz);
  while (!sd.begin(*config) && mhz > 0) {
    mhz--;
    config = new SdSpiConfig(4, DEDICATED_SPI, SD_SCK_MHZ(mhz));
    Serial.println(mhz);
  }
  Serial.print(F("Max SD card SPI speed: "));
  Serial.print(mhz);
  Serial.println("MHz");
}

void loop() {}
