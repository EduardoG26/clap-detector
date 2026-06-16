#include <Arduino.h>
#include <driver/i2s.h>
#include <kiss_fft.h>
#include <math.h>

// ──────────────────────────────────────────────────────────────────────────
// ESP32-C3 Pin Configuration (IMP I2S Microphone)
// ──────────────────────────────────────────────────────────────────────────
#define I2S_NUM I2S_NUM_0
#define I2S_BCK_PIN 4      // Bit Clock
#define I2S_WS_PIN 5       // Word Select (LRCLK)
#define I2S_DIN_PIN 6      // Data In (from mic)
#define I2S_MCLK_PIN -1    // Not used (optional)

#define BLUE_LED_PIN 10    // GPIO10 for blue LED
#define LED_FLASH_MS 250   // 250ms flash duration

// ──────────────────────────────────────────────────────────────────────────
// Audio Configuration
// ──────────────────────────────────────────────────────────────────────────
#define SAMPLE_RATE 16000
#define CHUNK_SIZE 512
#define FFT_SIZE 512
#define I2S_DMA_BUF_COUNT 4
#define I2S_DMA_BUF_LEN (CHUNK_SIZE / 2)  // 256 samples per DMA buffer

// ──────────────────────────────────────────────────────────────────────────
// Clap Detection Parameters
// ──────────────────────────────────────────────────────────────────────────
#define ENERGY_THRESHOLD_RATIO 5
#define DECAY_THRESHOLD 150        // log(ratio) threshold scaled (log(1.5)*100)
#define CLAP_COOLDOWN_MS 300
#define FREQ_MIN_HZ 1000
#define FREQ_MAX_HZ 8000
#define ENERGY_BUFFER_LEN 50
#define DECAY_BUFFER_LEN 6
#define MIN_ENERGY_HISTORY 10

// ──────────────────────────────────────────────────────────────────────────
// Detector State Structure
// ──────────────────────────────────────────────────────────────────────────
typedef struct {
    uint16_t energy_buf[ENERGY_BUFFER_LEN];
    uint16_t recent_energies[DECAY_BUFFER_LEN];
    uint32_t energy_sum;
    uint32_t last_clap_time_ms;
    uint8_t energy_buf_idx;
    uint8_t energy_count;
    uint8_t led_on;
    uint32_t led_on_time;
} ClapDetector;

ClapDetector detector = {0};

// ──────────────────────────────────────────────────────────────────────────
// FFT and Audio Buffers (preallocated)
// ──────────────────────────────────────────────────────────────────────────
kiss_fft_cfg fft_cfg = NULL;
kiss_fft_cpx fft_in[FFT_SIZE];
kiss_fft_cpx fft_out[FFT_SIZE];
int16_t audio_chunk[CHUNK_SIZE];
int16_t i2s_buffer[CHUNK_SIZE * 2];  // Double buffer for I2S DMA

// ──────────────────────────────────────────────────────────────────────────
// Fast Integer Square Root (Newton's method)
// ──────────────────────────────────────────────────────────────────────────
uint32_t fast_isqrt(uint32_t x) {
    if (x == 0) return 0;
    uint32_t s = x;
    uint32_t u = (s + 1) >> 1;  // (s + 1) / 2
    while (u < s) {
        s = u;
        u = (s + (x / s)) >> 1;
    }
    return s;
}

// ──────────────────────────────────────────────────────────────────────────
// Compute RMS Energy from int16 audio (avoids float)
// ──────────────────────────────────────────────────────────────────────────
uint16_t compute_rms_i16(int16_t *samples, uint16_t len) {
    if (len == 0) return 0;
    
    uint32_t sum_sq = 0;
    
    for (uint16_t i = 0; i < len; i++) {
        int32_t s = samples[i];
        // Avoid overflow: right-shift before squaring
        s = s >> 3;  // Divide by 8 (sacrifices 3 bits)
        sum_sq += (s * s);
    }
    
    uint32_t mean_sq = sum_sq / len;
    uint16_t rms = fast_isqrt(mean_sq);
    
    return rms;
}

// ──────────────────────────────────────────────────────────────────────────
// Update Energy Circular Buffer (O(1) with running sum)
// ──────────────────────────────────────────────────────────────────────────
void update_energy_buffer(uint16_t energy) {
    // Remove oldest value from sum
    detector.energy_sum -= detector.energy_buf[detector.energy_buf_idx];
    
    // Insert new value
    detector.energy_buf[detector.energy_buf_idx] = energy;
    detector.energy_sum += energy;
    
    // Advance circular pointer
    detector.energy_buf_idx = (detector.energy_buf_idx + 1) % ENERGY_BUFFER_LEN;
    
    // Track how many samples we've seen
    if (detector.energy_count < ENERGY_BUFFER_LEN) {
        detector.energy_count++;
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Get Average Energy (O(1))
// ──────────────────────────────────────────────────────────────────────────
uint16_t get_avg_energy(void) {
    if (detector.energy_count == 0) return 1;
    return detector.energy_sum / detector.energy_count;
}

// ──────────────────────────────────────────────────────────────────────────
// Compute FFT and Find Peak in 1-8 kHz Band
// ──────────────────────────────────────────────────────────────────────────
uint16_t compute_fft_peak(int16_t *audio_data) {
    // Convert int16 to complex (real only, imaginary = 0)
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_in[i].r = audio_data[i];
        fft_in[i].i = 0;
    }
    
    // Compute FFT (~1.2ms on ESP32-C3 @ 160MHz)
    kiss_fft(fft_cfg, fft_in, fft_out);
    
    // Frequency bins for 1-8 kHz
    int bin_1k = (FREQ_MIN_HZ * FFT_SIZE) / SAMPLE_RATE;
    int bin_8k = (FREQ_MAX_HZ * FFT_SIZE) / SAMPLE_RATE;
    
    uint32_t max_mag_sq = 0;
    int max_bin = bin_1k;
    
    // Find strongest peak in clap frequency band
    for (int i = bin_1k; i < bin_8k && i < (FFT_SIZE / 2); i++) {
        uint32_t mag_sq = (uint32_t)fft_out[i].r * fft_out[i].r + 
                          (uint32_t)fft_out[i].i * fft_out[i].i;
        if (mag_sq > max_mag_sq) {
            max_mag_sq = mag_sq;
            max_bin = i;
        }
    }
    
    // Return peak frequency in Hz
    if (max_mag_sq > 0) {
        return (max_bin * SAMPLE_RATE) / FFT_SIZE;
    }
    return 0;
}

// ──────────────────────────────────────────────────────────────────────────
// Check Decay Rate (exponential decay signature)
// Returns 1 if decay pattern matches clap, 0 otherwise
// ──────────────────────────────────────────────────────────────────────────
uint8_t check_decay(uint16_t *energies, uint8_t len) {
    if (len < 2) return 0;
    
    uint8_t decay_count = 0;
    
    for (int i = 0; i < len - 1; i++) {
        if (energies[i] > 0 && energies[i+1] > 0 && energies[i] > energies[i+1]) {
            // Compute ratio: energies[i] / energies[i+1]
            // log(ratio) > 1.5 means decay constant > 15/s
            // log(1.5) * 100 ≈ 41, but we use scaled ratio > 450 (exp(1.5)*100)
            uint32_t ratio_scaled = (energies[i] * 100) / energies[i+1];
            
            if (ratio_scaled > 450) {  // exp(1.5) ≈ 4.48
                decay_count++;
            }
        }
    }
    
    // Need at least 3 consecutive decaying steps
    return (decay_count >= 3) ? 1 : 0;
}

// ──────────────────────────────────────────────────────────────────────────
// Main Clap Detection Logic
// ──────────────────────────────────────────────────────────────────────────
uint8_t detect_clap(uint16_t energy) {
    uint32_t now = millis();
    
    // Check cooldown (prevent re-triggering)
    if (now - detector.last_clap_time_ms < CLAP_COOLDOWN_MS) {
        return 0;
    }
    
    // Need minimum history before detection
    if (detector.energy_count < MIN_ENERGY_HISTORY) {
        return 0;
    }
    
    // Stage 1: Energy threshold (5x baseline)
    uint16_t avg_energy = get_avg_energy();
    uint32_t ratio = (energy * 100) / (avg_energy > 0 ? avg_energy : 1);
    
    if (ratio <= ENERGY_THRESHOLD_RATIO * 100) {
        return 0;
    }
    
    // Stage 2: FFT frequency check (1-8 kHz band)
    uint16_t peak_freq = compute_fft_peak(audio_chunk);
    if (peak_freq < FREQ_MIN_HZ || peak_freq > FREQ_MAX_HZ) {
        return 0;
    }
    
    // Stage 3: Decay rate check (exponential damping)
    // Record decay over next 6 chunks (6 * 32ms ≈ 192ms)
    uint16_t decay_energies[DECAY_BUFFER_LEN];
    decay_energies[0] = energy;
    
    for (int i = 1; i < DECAY_BUFFER_LEN; i++) {
        // Read next chunk (32ms)
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_NUM, i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
        
        if (err != ESP_OK) {
            return 0;
        }
        
        // Convert stereo to mono
        for (int j = 0; j < CHUNK_SIZE; j++) {
            audio_chunk[j] = i2s_buffer[j * 2];  // Left channel
        }
        
        decay_energies[i] = compute_rms_i16(audio_chunk, CHUNK_SIZE);
    }
    
    // Check decay signature
    if (!check_decay(decay_energies, DECAY_BUFFER_LEN)) {
        return 0;
    }
    
    // All stages passed: CLAP DETECTED!
    detector.last_clap_time_ms = now;
    return 1;
}

// ──────────────────────────────────────────────────────────────────────────
// LED Flash Control (non-blocking)
// ──────────────────────────────────────────────────────────────────────────
void trigger_led_flash(void) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    detector.led_on = 1;
    detector.led_on_time = millis();
    Serial.println("👏 CLAP DETECTED!");
}

void update_led(void) {
    if (detector.led_on) {
        if (millis() - detector.led_on_time >= LED_FLASH_MS) {
            digitalWrite(BLUE_LED_PIN, LOW);
            detector.led_on = 0;
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// I2S Initialization (IMP Microphone, I2S mode)
// ──────────────────────────────────────────────────────────────────────────
void init_i2s(void) {
    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0,
    };
    
    // Pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = -1,         // Not used (RX only)
        .data_in_num = I2S_DIN_PIN,
        .mck_io_num = I2S_MCLK_PIN,
    };
    
    // Install and configure I2S driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
    
    Serial.println("[I2S] Microphone initialized (16-bit, 16 kHz, I2S mode)");
}

// ──────────────────────────────────────────────────────────────────────────
// Setup
// ──────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║     ESP32-C3 Clap Detector (IMP I2S Microphone)        ║");
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
    
    // Initialize GPIO for LED
    pinMode(BLUE_LED_PIN, OUTPUT);
    digitalWrite(BLUE_LED_PIN, LOW);
    Serial.printf("[GPIO] Blue LED initialized on pin %d\n", BLUE_LED_PIN);
    
    // Initialize I2S microphone
    init_i2s();
    
    // Initialize Kiss FFT
    fft_cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    if (!fft_cfg) {
        Serial.println("[ERROR] Kiss FFT allocation failed!");
        while (1);
    }
    Serial.println("[FFT] Kiss FFT initialized (512-point)");
    
    // Initialize detector state
    memset(&detector, 0, sizeof(detector));
    detector.last_clap_time_ms = 0;
    
    Serial.println("\n[CONFIG]");
    Serial.printf("  Sample Rate:        %d Hz\n", SAMPLE_RATE);
    Serial.printf("  Chunk Size:         %d samples (%.1f ms)\n", CHUNK_SIZE, (float)CHUNK_SIZE * 1000 / SAMPLE_RATE);
    Serial.printf("  Energy Threshold:   %.1fx baseline\n", (float)ENERGY_THRESHOLD_RATIO);
    Serial.printf("  Frequency Band:     %d - %d Hz\n", FREQ_MIN_HZ, FREQ_MAX_HZ);
    Serial.printf("  Cooldown:           %d ms\n", CLAP_COOLDOWN_MS);
    Serial.printf("  LED Flash:          %d ms\n\n", LED_FLASH_MS);
    
    Serial.println("[STATUS] Waiting for claps...\n");
}

// ──────────────────────────────────────────────────────────────────────────
// Main Loop
// ──────────────────────────────────────────────────────────────────────────
void loop() {
    // Read I2S audio chunk
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM, i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
    
    if (err != ESP_OK) {
        Serial.printf("[ERROR] I2S read error: %s\n", esp_err_to_name(err));
        return;
    }
    
    // Convert I2S stereo buffer to mono audio chunk
    for (int i = 0; i < CHUNK_SIZE; i++) {
        audio_chunk[i] = i2s_buffer[i * 2];  // Left channel only
    }
    
    // Compute energy
    uint16_t energy = compute_rms_i16(audio_chunk, CHUNK_SIZE);
    update_energy_buffer(energy);
    
    // Check for clap
    if (detect_clap(energy)) {
        trigger_led_flash();
    }
    
    // Update LED state (non-blocking auto-off after 250ms)
    update_led();
    
    // Debug: Print energy level (every 8 chunks = ~256ms)
    static uint8_t debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 8) {
        debug_counter = 0;
        uint16_t avg = get_avg_energy();
        uint32_t ratio = (energy * 100) / (avg > 0 ? avg : 1);
        Serial.printf("[ENERGY] Current: %5d | Avg: %5d | Ratio: %.2fx\n", 
                      energy, avg, (float)ratio / 100.0);
    }
}