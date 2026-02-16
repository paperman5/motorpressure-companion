
uint8_t baseArray8[24];
uint16_t baseArray16[16];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  for (int i = 0; i < 16; i++) {
    baseArray16[i] = 0x111 * i;
  }
  memset(&baseArray8, 0, sizeof(baseArray8));
  print_raw(&baseArray8, sizeof(baseArray8), 8);
  Serial.println();
  print_raw(&baseArray16, sizeof(baseArray16), 16);
  Serial.println();

  for (int i = 0; i < 16; i++) {
    store_12bit(baseArray8, i, baseArray16[i]);
  }
  print_raw(&baseArray8, sizeof(baseArray8), 8);
  Serial.println();

  for (int i = 0; i < 16; i++) {
    int j = 15 - i;
    baseArray16[i] = read_12bit(baseArray8, j);
  }
  print_raw(&baseArray16, sizeof(baseArray16), 16);
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

}

// Stores a 12bit uint (the 12 lower bits of a 16bit number) inside of a larger 8bit array as if
// it were a native array of 12bit uints, such that each 12bit value is big-endian contiguous
// on half-byte boundaries.
// Example: numbers `0x123` (little-endian `0x2301`) and `0x456` (LE `0x5604`)
// at 12bit indices 0 and 1 become `0x12 34 56` in the underlying 8-bit array.
inline void store_12bit(uint8_t arr8[], uint32_t index12, uint16_t value12) {
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
inline uint16_t read_12bit(uint8_t arr8[], uint32_t index12) {
  uint32_t i8 = index12 * 3 / 2;
  if (index12 % 2) {
    return (arr8[i8] & 0x0F) << 8 | arr8[i8+1];
  }
  else {
    return (arr8[i8] << 8 | arr8[i8+1]) >> 4;
  }
}

void print_raw(void* obj, size_t size) {
  print_raw(obj, size, 8);
}

void print_raw(void* obj, size_t size, uint8_t print_width) {
  uint8_t* buf = reinterpret_cast<uint8_t *>(obj);
  for (uint16_t i = 0; i < size; i++) {
    Serial.printf("%02X%s", *(buf + i), (i + 1) % print_width == 0 ? "\n" : " ");
  }
  Serial.println();
}
