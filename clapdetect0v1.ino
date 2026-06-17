#include <Arduino.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

#define SAMPLE_RATE 16000
#define FFT_SIZE 256

// ----- INMP441 wiring -----
// Change to match your wiring.
#define I2S_WS 2   // LRCLK / WS
#define I2S_SCK 3  // BCLK / SCK
#define I2S_SD 4   // DOUT

// ESP32-C3 SuperMini onboard LED.
// Many boards use GPIO8, verify yours.
#define LED_PIN 8

ArduinoFFT<double> FFT;

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

float previousRMS = 100.0f;

uint32_t lastClapTime = 0;
uint32_t ledOffTime = 0;

// ---------- Tune these ----------
const float BAND_ENERGY_THRESHOLD = 5.0e10;
const float RISE_RATIO_THRESHOLD = 3.0;
const float HI_LO_RATIO_THRESHOLD = 1.5;
const int PEAK_THRESHOLD = 4000;
const uint32_t CLAP_COOLDOWN_MS = 333;
const uint32_t LED_OFF_TIME_MS = 333;
// ---------------------------------

void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = FFT_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);

  FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

  setupI2S();

  Serial.println();
  Serial.println("ESP32-C3 Clap Detector");
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static int32_t raw[FFT_SIZE];

  size_t bytesRead = 0;

  i2s_read(
    I2S_NUM_0,
    raw,
    sizeof(raw),
    &bytesRead,
    portMAX_DELAY);

  if (bytesRead != sizeof(raw))
    return;

  float sumSq = 0;
  int peak = 0;

  for (int i = 0; i < FFT_SIZE; i++) {
    // INMP441 useful data is in upper bits.
    int32_t sample = raw[i] >> 14;

    vReal[i] = sample;
    vImag[i] = 0;

    int a = abs(sample);
    if (a > peak)
      peak = a;

    sumSq += (double)sample * (double)sample;
  }

  float rms = sqrt(sumSq / FFT_SIZE);

  float riseRatio = 1.0;

  if (previousRMS > 50.0)
    riseRatio = rms / previousRMS;

  previousRMS = rms;

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  int startBin = (1000 * FFT_SIZE) / SAMPLE_RATE;
  int endBin = (4000 * FFT_SIZE) / SAMPLE_RATE;

  double bandEnergy = 0;

  for (int i = startBin; i <= endBin; i++) {
    bandEnergy += vReal[i] * vReal[i];
  }

  uint32_t now = millis();

  float lowEnergy = 0;
  float highEnergy = 0;

  // Low frequencies: bins 1-10
  for (int i = 1; i <= 10; i++) {
    lowEnergy += vReal[i];
  }

  // Higher frequencies: bins 20-80
  for (int i = 20; i <= 120; i++) {
    highEnergy += vReal[i];
  }

  bool clap = bandEnergy > BAND_ENERGY_THRESHOLD
              && riseRatio > RISE_RATIO_THRESHOLD
              && peak > PEAK_THRESHOLD
              && (highEnergy > lowEnergy * HI_LO_RATIO_THRESHOLD)
              && (now - lastClapTime) > CLAP_COOLDOWN_MS;

  if (clap) {
    lastClapTime = now;

    digitalWrite(LED_PIN, HIGH);
    ledOffTime = now;

    Serial.println("* CLAP *");
    Serial.printf("Clap@(ms)   : %lu\n", now);
    Serial.printf("peak        : %d\n", peak);
    Serial.printf("rms         : %.1f\n", rms);
    Serial.printf("rise_ratio  : %.2f\n", riseRatio);
    Serial.printf("hi_lo_ratio : %.2f\n", highEnergy / lowEnergy);
    Serial.printf("band_energy : %.0f\n", bandEnergy);
  }

  if (now - ledOffTime > LED_OFF_TIME_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOffTime = now;
  }
}
