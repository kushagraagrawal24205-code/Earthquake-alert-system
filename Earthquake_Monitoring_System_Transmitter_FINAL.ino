/*
  EARTHQUAKE MONITORING SYSTEM - TRANSMITTER
  Board: ESP8266 NodeMCU
  Sensor: MPU6050 (I2C address 0x68)
  Radio: SX1278 LoRa 433 MHz

  Transmitter wiring:
    MPU6050 SDA -> D3, SCL -> D1
    SX1278 NSS -> D8, DIO0 -> D2, RESET -> D0
    SX1278 SCK -> D5, MISO -> D6, MOSI -> D7
    SX1278 VCC -> 3.3V, GND -> GND

  Packet format expected by matching receiver:
    V1,sequence,status,ax,ay,az,rms,peak,frequency

  NOTE: Threshold and sampling settings are prototype parameters.
  Validate/calibrate them for the mounting arrangement before making
  scientific or safety claims.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>

// ---------------- PIN AND SENSOR CONFIG ----------------
#define MPU_SDA D3
#define MPU_SCL D1
#define MPU_ADDR 0x68

#define LORA_NSS D8
#define LORA_DIO0 D2
#define LORA_RESET D0

const float ACCEL_COUNTS_PER_G = 16384.0f; // MPU6050 +/-2 g range
const float STANDARD_GRAVITY = 9.80665f;

// ---------------- SAMPLING / PROCESSING ----------------
const uint16_t SAMPLE_COUNT = 64;      // Must be a power of two for this FFT
const float SAMPLE_RATE_HZ = 50.0f;
const uint32_t SAMPLE_INTERVAL_US = 1000000UL / 50UL;
const float ALERT_PEAK_THRESHOLD = 1.0f; // m/s^2, prototype threshold

float axSamples[SAMPLE_COUNT];
float aySamples[SAMPLE_COUNT];
float azSamples[SAMPLE_COUNT];
float fftReal[SAMPLE_COUNT];
float fftImag[SAMPLE_COUNT];

SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RESET, RADIOLIB_NC);
uint32_t packetSequence = 0;
bool radioReady = false;

// ---------------- MPU6050 LOW-LEVEL FUNCTIONS ----------------
bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPUBlock(uint8_t startRegister, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startRegister);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, length, (uint8_t)true);
  if (received != length) return false;

  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

bool initializeMPU6050() {
  uint8_t whoAmI = 0;
  if (!readMPUBlock(0x75, &whoAmI, 1)) return false;
  if (whoAmI != 0x68 && whoAmI != 0x69) {
    Serial.print(F("Unexpected MPU6050 WHO_AM_I: 0x"));
    Serial.println(whoAmI, HEX);
    return false;
  }

  // Wake device; set accelerometer range to +/-2g (AFS_SEL=0).
  if (!writeMPURegister(0x6B, 0x00)) return false;
  delay(100);
  if (!writeMPURegister(0x1C, 0x00)) return false;
  // Digital low-pass filter configuration (DLPF_CFG=3).
  if (!writeMPURegister(0x1A, 0x03)) return false;
  return true;
}

bool readAcceleration(float &ax, float &ay, float &az) {
  uint8_t data[6];
  if (!readMPUBlock(0x3B, data, 6)) return false;

  int16_t rawX = combineBytes(data[0], data[1]);
  int16_t rawY = combineBytes(data[2], data[3]);
  int16_t rawZ = combineBytes(data[4], data[5]);

  ax = ((float)rawX / ACCEL_COUNTS_PER_G) * STANDARD_GRAVITY;
  ay = ((float)rawY / ACCEL_COUNTS_PER_G) * STANDARD_GRAVITY;
  az = ((float)rawZ / ACCEL_COUNTS_PER_G) * STANDARD_GRAVITY;
  return true;
}

// ---------------- FFT: RADIX-2 COoley-Tukey ----------------
void fft(float *realPart, float *imagPart, uint16_t n) {
  // Bit-reversal permutation.
  uint16_t j = 0;
  for (uint16_t i = 1; i < n; i++) {
    uint16_t bit = n >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      float temp = realPart[i]; realPart[i] = realPart[j]; realPart[j] = temp;
      temp = imagPart[i]; imagPart[i] = imagPart[j]; imagPart[j] = temp;
    }
  }

  // Iterative butterfly stages.
  for (uint16_t len = 2; len <= n; len <<= 1) {
    float angle = -2.0f * PI * (float)len / (float)len;
    float wLenReal = cosf(angle);
    float wLenImag = sinf(angle);

    for (uint16_t start = 0; start < n; start += len) {
      float wReal = 1.0f;
      float wImag = 0.0f;
      for (uint16_t k = 0; k < len / 2; k++) {
        uint16_t evenIndex = start + k;
        uint16_t oddIndex = evenIndex + len / 2;

        float oddReal = realPart[oddIndex] * wReal - imagPart[oddIndex] * wImag;
        float oddImag = realPart[oddIndex] * wImag + imagPart[oddIndex] * wReal;

        float evenReal = realPart[evenIndex];
        float evenImag = imagPart[evenIndex];

        realPart[evenIndex] = evenReal + oddReal;
        imagPart[evenIndex] = evenImag + oddImag;
        realPart[oddIndex] = evenReal - oddReal;
        imagPart[oddIndex] = evenImag - oddImag;

        float nextWReal = wReal * wLenReal - wImag * wLenImag;
        wImag = wReal * wLenImag + wImag * wLenReal;
        wReal = nextWReal;
      }
    }
  }
}

float calculateDominantFrequency(const float *dynamicMagnitude) {
  // Hann window reduces spectral leakage.
  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float hann = 0.5f * (1.0f - cosf(2.0f * PI * i / (SAMPLE_COUNT - 1)));
    fftReal[i] = dynamicMagnitude[i] * hann;
    fftImag[i] = 0.0f;
  }

  fft(fftReal, fftImag, SAMPLE_COUNT);

  float maxMagnitude = 0.0f;
  uint16_t dominantBin = 0;

  // Ignore DC bin (0); Nyquist bin is SAMPLE_COUNT/2.
  for (uint16_t bin = 1; bin <= SAMPLE_COUNT / 2; bin++) {
    float magnitude = sqrtf(fftReal[bin] * fftReal[bin] +
                            fftImag[bin] * fftImag[bin]);
    if (magnitude > maxMagnitude) {
      maxMagnitude = magnitude;
      dominantBin = bin;
    }
  }

  return (float)dominantBin * SAMPLE_RATE_HZ / SAMPLE_COUNT;
}

// ---------------- ACQUIRE ONE ANALYSIS WINDOW ----------------
bool acquireWindow() {
  uint32_t nextSampleTime = micros();

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    while ((int32_t)(micros() - nextSampleTime) < 0) {
      yield();
    }

    if (!readAcceleration(axSamples[i], aySamples[i], azSamples[i])) {
      Serial.println(F("MPU6050 read error during sample window."));
      return false;
    }

    nextSampleTime += SAMPLE_INTERVAL_US;
    yield();
  }
  return true;
}

void processAndTransmitWindow() {
  if (!acquireWindow()) {
    delay(100);
    return;
  }

  float meanX = 0.0f, meanY = 0.0f, meanZ = 0.0f;
  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    meanX += axSamples[i];
    meanY += aySamples[i];
    meanZ += azSamples[i];
  }
  meanX /= SAMPLE_COUNT;
  meanY /= SAMPLE_COUNT;
  meanZ /= SAMPLE_COUNT;

  float dynamicMagnitude[SAMPLE_COUNT];
  float sumSquares = 0.0f;
  float peak = 0.0f;

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float dx = axSamples[i] - meanX;
    float dy = aySamples[i] - meanY;
    float dz = azSamples[i] - meanZ;

    float dynamicValue = sqrtf(dx * dx + dy * dy + dz * dz);
    dynamicMagnitude[i] = dynamicValue;
    sumSquares += dynamicValue * dynamicValue;
    if (dynamicValue > peak) peak = dynamicValue;
  }

  float rms = sqrtf(sumSquares / SAMPLE_COUNT);
  float dominantFrequency = calculateDominantFrequency(dynamicMagnitude);
  const char* status = (peak >= ALERT_PEAK_THRESHOLD) ? "ALERT" : "NORMAL";

  // Versioned packet: exactly 8 parsed fields after the V1 prefix.
  String payload = "V1,";
  payload += String(packetSequence++);
  payload += ",";
  payload += status;
  payload += ",";
  payload += String(meanX, 2);
  payload += ",";
  payload += String(meanY, 2);
  payload += ",";
  payload += String(meanZ, 2);
  payload += ",";
  payload += String(rms, 3);
  payload += ",";
  payload += String(peak, 3);
  payload += ",";
  payload += String(dominantFrequency, 2);

  Serial.println(F("\n--- ANALYSIS WINDOW ---"));
  Serial.print(F("Mean X: ")); Serial.println(meanX, 2);
  Serial.print(F("Mean Y: ")); Serial.println(meanY, 2);
  Serial.print(F("Mean Z: ")); Serial.println(meanZ, 2);
  Serial.print(F("RMS: ")); Serial.println(rms, 3);
  Serial.print(F("Peak: ")); Serial.println(peak, 3);
  Serial.print(F("Dominant frequency: ")); Serial.println(dominantFrequency, 2);
  Serial.print(F("Status: ")); Serial.println(status);
  Serial.print(F("Packet: ")); Serial.println(payload);

  int state = radio.transmit(payload);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("TX SUCCESS"));
  } else {
    Serial.print(F("TX FAILED, code: "));
    Serial.println(state);
  }
  yield();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);

  Serial.println();
  Serial.println(F("=== EARTHQUAKE MONITORING SYSTEM: TRANSMITTER ==="));

  if (!initializeMPU6050()) {
    Serial.println(F("MPU6050 initialization failed. Check SDA/SCL, 3.3V and GND."));
    while (true) { delay(1000); yield(); }
  }
  Serial.println(F("MPU6050 ready."));

  Serial.println(F("Initializing SX1278 at 433 MHz..."));
  int radioState = radio.begin(433.0);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa initialization failed, code: "));
    Serial.println(radioState);
    while (true) { delay(1000); yield(); }
  }

  radio.setOutputPower(10);
  Serial.println(F("LoRa transmitter ready."));
  Serial.println(F("Sampling: 50 Hz; window: 64 samples; duration: about 1.28 s."));
  Serial.print(F("Alert peak threshold: "));
  Serial.print(ALERT_PEAK_THRESHOLD, 2);
  Serial.println(F(" m/s^2"));
  radioReady = true;
}

void loop() {
  if (radioReady) processAndTransmitWindow();
  yield();
}
