#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"  // ensure Sensor base class is available
#include <string>
#include <utility>

// Forward-declare binary sensor and text sensor to avoid requiring the headers at this point
namespace esphome { namespace binary_sensor { class BinarySensor; } }
namespace esphome { namespace text_sensor { class TextSensor; } }

// ESP-IDF / FreeRTOS headers
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"

// Forward declaration of C ISR wrapper (defined after the namespace)
extern "C" void esp32_spa_isr_wrapper(void* arg);

static const char *TAG = "esp32-spa";

// ===== PIN DEFINITIONS =====
// Using input-only GPIOs on ESP32: CLK=GPIO35, DATA=GPIO34
// Note: GPIO34/35 are input-only and do NOT support internal pull-ups; use external pull resistors (e.g., 10k) and a small series resistor on the clock (~47-220 ohm).
static const uint8_t CLK_PIN  = 35;  // Clock (input-only)
static const uint8_t DATA_PIN = 34;  // Data (input-only)

// Button output pins used to inject presses. These are boot-safe GPIOs and match
// the values in `esp32-spa.yaml` (Warm=25, Cool=26, Lights=27, Pumps=32).
#define PIN_WRITE_BTN1 25  // Warm
#define PIN_WRITE_BTN2 26  // Cool
#define PIN_WRITE_BTN3 27  // Lights
#define PIN_WRITE_PUMP 32  // Pumps

namespace esp32_spa {

class HotTubDisplaySensor : public esphome::Component, public esphome::sensor::Sensor {
 public:
  // ---- Shared with ISR ----
  volatile uint64_t shift_reg = 0;  // in-progress frame bits (up to 64 bits)
  volatile uint8_t bit_count = 0;
  volatile bool frame_ready = false;
  // Frame boundary is detected purely by inter-frame gap (not a fixed bit count),
  // so any frame length up to 64 bits is handled correctly.

  // ---- Publish control ----
  uint32_t last_publish_time = 0;
  uint32_t last_published_value = 0;
  bool first_publish = true;
  volatile bool last_frame_valid = false;  // becomes true when a frame passes the checksum and is published

  // Remember last decoded values for change detection
  int16_t last_measured_temp = -1;  // -1 = unknown
  int16_t last_set_temp = -1;       // -1 = unknown
  int16_t set_temp_potential = -1;  // candidate for set temp
  uint32_t last_zero_seen_time = 0; // last time we saw 0x00 in p2/p3
  uint32_t last_candidate_temp_time = 0; // last time we saw a candidate temp while in set mode
  bool in_set_mode = false;         // true when we've seen 0x00 recently

  // Pending measured temp publish (to avoid misreading brief set-mode flashes as measured temp)
  int16_t pending_measured_temp = -1;        // -1 = none pending
  uint32_t pending_measured_since = 0;       // time when pending started (ms)
  static constexpr uint32_t MEASURE_PUBLISH_DELAY_MS = 500;  // delay before publishing measured temp (ms)

  // Stability tracking (counters and candidates)
  int16_t candidate_temp = -2; uint8_t stable_temp = 0;
  bool candidate_is_zero = false; uint8_t stable_zero = 0;
  // Heater stability (derived from bit5 of p1)
  int8_t candidate_heater = -2; uint8_t stable_heater = 0;
  // Pump & light stability (derived from p4 bits)
  int8_t candidate_pump = -1; uint8_t stable_pump = 0;
  int8_t candidate_light = -1; uint8_t stable_light = 0;

  // Sensors for temperature readings
  esphome::sensor::Sensor *measured_temp_sensor_ = nullptr;
  esphome::sensor::Sensor *set_temp_sensor_ = nullptr;
  // Text sensor for error codes
  esphome::text_sensor::TextSensor *error_text_sensor_ = nullptr;

  // Binary sensors for discrete states
  esphome::binary_sensor::BinarySensor *heater_sensor_ = nullptr;  // derived from p1 bit5
  esphome::binary_sensor::BinarySensor *pump_sensor_ = nullptr;    // derived from p4 bit0
  esphome::binary_sensor::BinarySensor *light_sensor_ = nullptr;   // derived from p4 bit1

  // Last published discrete states
  int8_t last_heater = -1;  // -1=unknown, otherwise 0/1
  int8_t last_pump = -1;    // -1=unknown, otherwise 0/1
  int8_t last_light = -1;   // -1=unknown, otherwise 0/1
  // Last published error code (string)
  std::string last_error_code_ = "";
  // Error code stability tracking
  std::string candidate_error = ""; uint8_t stable_error = 0;

  static constexpr uint32_t HEARTBEAT_MS = 30000;  // heartbeat every 30s (publish if unchanged)
  // Gap threshold (ms) to consider the start of a new frame (use ~15ms to match ~19ms observed gap)
  static constexpr uint32_t FRAME_GAP_MS =5;
  static constexpr uint32_t FRAME_GAP_US = FRAME_GAP_MS * 1000;

  // Stability filtering: require this many consecutive identical decoded frames before publishing
  // Increased to 2 to reduce spurious publishes from brief noise
  static constexpr uint8_t STABLE_THRESHOLD = 3;
  static constexpr uint8_t PUMP_STABLE_THRESHOLD = 3;  // pump requires 3 repeats to be considered stable
  // Error codes are noisier — require more repeats to consider stable
  static constexpr uint8_t ERROR_STABLE_THRESHOLD = 3;
  static constexpr uint32_t SET_MODE_TIMEOUT_MS = 2000;  // 2 seconds without 0x00 = exit set mode
  static constexpr uint32_t HEATER_OFF_TIMEOUT_MS = 1000; // heater must be off for 1s before clearing

  // Timestamp to track when heater bit last went low while heater was on
  uint32_t last_heater_off_time = 0;

  // --- Auto-refresh set-temp logic ---
  // When we capture & publish the set temp, reset this timer. If no set-temp is captured
  // for SET_FORCE_INTERVAL_MS milliseconds we auto-press COOL once to force the tub to
  // display and send the set temperature. We also update the timer when we auto-press.
  uint32_t last_set_sent_time_ms = 0;
  static constexpr uint32_t SET_FORCE_INTERVAL_MS = 5u * 60u * 1000u;  // 5 minutes
  
  // Setters called from Python binding
  void set_measured_temp_sensor(esphome::sensor::Sensor *s) { measured_temp_sensor_ = s; }
  void set_set_temp_sensor(esphome::sensor::Sensor *s) { set_temp_sensor_ = s; }
  void set_error_text_sensor(esphome::text_sensor::TextSensor *s) { error_text_sensor_ = s; }

  // Binary sensor setters
  void set_heater_sensor(esphome::binary_sensor::BinarySensor *s) { heater_sensor_ = s; }
  void set_pump_sensor(esphome::binary_sensor::BinarySensor *s) { pump_sensor_ = s; }
  void set_light_sensor(esphome::binary_sensor::BinarySensor *s) { light_sensor_ = s; }

  // Raw frame text sensor (publishes frame as binary string with bit-count prefix, e.g. "24:011011...")
  esphome::text_sensor::TextSensor *raw_frame_sensor_ = nullptr;
  void set_raw_frame_sensor(esphome::text_sensor::TextSensor *s) { raw_frame_sensor_ = s; }

  struct DiscoverySample {
    uint8_t bit_count;
    uint64_t value;
  };

  struct DiscoveryPattern {
    uint8_t bit_count = 0;
    uint64_t value = 0;
    uint16_t count = 0;
  };

  static constexpr uint8_t DISCOVERY_WINDOW_FRAMES = 100;
  static constexpr uint8_t DISCOVERY_PATTERN_SLOTS = 48;  // enough slots for all unique patterns in a 100-frame window
  static constexpr uint8_t DISCOVERY_LOG_INTERVAL = 100;  // one summary per full window rotation

  DiscoverySample discovery_window_[DISCOVERY_WINDOW_FRAMES] = {};
  DiscoveryPattern discovery_patterns_[DISCOVERY_PATTERN_SLOTS] = {};
  uint16_t discovery_window_size_ = 0;
  uint16_t discovery_window_pos_ = 0;
  uint16_t discovery_length_hist_[65] = {};
  uint32_t discovery_frame_seq_ = 0;
  uint32_t discovery_last_summary_seq_ = 0;

  void discovery_add_pattern(uint8_t bit_count, uint64_t value) {
    int free_slot = -1;
    int weakest_slot = 0;
    uint16_t weakest_count = 0xFFFF;

    for (uint8_t i = 0; i < DISCOVERY_PATTERN_SLOTS; ++i) {
      auto &slot = discovery_patterns_[i];
      if (slot.count == 0) {
        if (free_slot < 0) free_slot = i;
        continue;
      }
      if (slot.bit_count == bit_count && slot.value == value) {
        slot.count++;
        return;
      }
      if (slot.count < weakest_count) {
        weakest_count = slot.count;
        weakest_slot = i;
      }
    }

    int target = free_slot >= 0 ? free_slot : weakest_slot;
    discovery_patterns_[target].bit_count = bit_count;
    discovery_patterns_[target].value = value;
    discovery_patterns_[target].count = 1;
  }

  void discovery_remove_pattern(uint8_t bit_count, uint64_t value) {
    for (uint8_t i = 0; i < DISCOVERY_PATTERN_SLOTS; ++i) {
      auto &slot = discovery_patterns_[i];
      if (slot.count > 0 && slot.bit_count == bit_count && slot.value == value) {
        if (--slot.count == 0) {
          slot.bit_count = 0;
          slot.value = 0;
        }
        return;
      }
    }
  }

  void discovery_log_summary() {
    if (discovery_window_size_ == 0) return;

    uint8_t top_length[3] = {0, 0, 0};
    uint16_t top_length_count[3] = {0, 0, 0};
    for (uint8_t bits = 0; bits <= 64; ++bits) {
      uint16_t count = discovery_length_hist_[bits];
      if (count == 0) continue;
      for (uint8_t i = 0; i < 3; ++i) {
        if (count > top_length_count[i]) {
          for (uint8_t j = 2; j > i; --j) {
            top_length_count[j] = top_length_count[j - 1];
            top_length[j] = top_length[j - 1];
          }
          top_length_count[i] = count;
          top_length[i] = bits;
          break;
        }
      }
    }

    uint8_t top_slot[3] = {255, 255, 255};
    uint16_t top_count[3] = {0, 0, 0};
    for (uint8_t i = 0; i < DISCOVERY_PATTERN_SLOTS; ++i) {
      const auto &slot = discovery_patterns_[i];
      if (slot.count == 0) continue;
      for (uint8_t j = 0; j < 3; ++j) {
        if (slot.count > top_count[j]) {
          for (uint8_t k = 2; k > j; --k) {
            top_count[k] = top_count[k - 1];
            top_slot[k] = top_slot[k - 1];
          }
          top_count[j] = slot.count;
          top_slot[j] = i;
          break;
        }
      }
    }

    char message[512];
    size_t pos = 0;
    pos += snprintf(message + pos, sizeof(message) - pos,
      "Discovery window seq=%u frames=%u lengths:",
      static_cast<unsigned>(discovery_frame_seq_), static_cast<unsigned>(discovery_window_size_));

    for (uint8_t i = 0; i < 3; ++i) {
      if (top_length_count[i] == 0) continue;
      pos += snprintf(message + pos, sizeof(message) - pos,
        " %u=%u", static_cast<unsigned>(top_length[i]), static_cast<unsigned>(top_length_count[i]));
    }

    pos += snprintf(message + pos, sizeof(message) - pos, " patterns:");

    for (uint8_t i = 0; i < 3; ++i) {
      if (top_slot[i] == 255) continue;
      const auto &slot = discovery_patterns_[top_slot[i]];
      char bits[68];  // 64 bits + '\0' + margin
      size_t bit_pos = 0;
      for (int b = slot.bit_count - 1; b >= 0 && bit_pos + 1 < sizeof(bits); --b) {
        bits[bit_pos++] = ((slot.value >> b) & 1ULL) ? '1' : '0';
      }
      bits[bit_pos] = '\0';
      pos += snprintf(message + pos, sizeof(message) - pos,
        " %u:%s x%u", static_cast<unsigned>(slot.bit_count), bits, static_cast<unsigned>(slot.count));
    }

    ESP_LOGI(TAG, "%s", message);
  }

  void discovery_record_frame(uint8_t bit_count, uint64_t value) {
    discovery_frame_seq_++;

    if (bit_count <= 64) {
      discovery_length_hist_[bit_count]++;
    }

    if (discovery_window_size_ == DISCOVERY_WINDOW_FRAMES) {
      const auto &oldest = discovery_window_[discovery_window_pos_];
      if (oldest.bit_count <= 64 && discovery_length_hist_[oldest.bit_count] > 0) {
        discovery_length_hist_[oldest.bit_count]--;
      }
      discovery_remove_pattern(oldest.bit_count, oldest.value);
    } else {
      discovery_window_size_++;
    }

    discovery_window_[discovery_window_pos_] = {bit_count, value};
    discovery_window_pos_ = static_cast<uint16_t>((discovery_window_pos_ + 1) % DISCOVERY_WINDOW_FRAMES);
    discovery_add_pattern(bit_count, value);

    if (discovery_window_size_ == DISCOVERY_WINDOW_FRAMES &&
        (discovery_frame_seq_ - discovery_last_summary_seq_) >= DISCOVERY_LOG_INTERVAL) {
      discovery_log_summary();
      discovery_last_summary_seq_ = discovery_frame_seq_;
    }
  }

  static void append_bits(char *buffer, size_t buffer_size, uint64_t value, uint8_t bit_count) {
    if (buffer_size == 0) return;
    size_t pos = 0;
    for (int bit = bit_count - 1; bit >= 0 && pos + 1 < buffer_size; --bit) {
      buffer[pos++] = ((value >> bit) & 1ULL) ? '1' : '0';
    }
    buffer[pos] = '\0';
  }



  
  // Decode temperature from p1, p2, p3
  // p2 = tens digit, p3 = ones digit, bits 5&4 of p1 both high = add 100
  static int16_t decode_temp(uint8_t p1, int8_t d2, int8_t d3) {
    if (d2 < 0 || d3 < 0) return -1;  // invalid digits
    int16_t temp = d2 * 10 + d3;
    // Check bits 5 and 4 of p1 (0b00110000 = 0x30)
    if ((p1 & 0x30) == 0x30) {
      temp += 100;
    }
    return temp;
  }
  static int8_t decode_7seg(uint8_t seg) {
    // bit ordering: bit6=a(top), bit5=b(upper right), bit4=c(lower right), bit3=d(bottom), bit2=e(lower left), bit1=f(upper left), bit0=g(middle)
    static const uint8_t map[10] = {
      0b1111110, // 0
      0b0110000, // 1
      0b1101101, // 2
      0b1111001, // 3
      0b0110011, // 4
      0b1011011, // 5
      0b1011111, // 6
      0b1110000, // 7
      0b1111111, // 8
      0b1110011  // 9
    };

    // Only accept exact matches to avoid occasional 1-bit misreads causing spurious digits.
    for (uint8_t d = 0; d < 10; ++d) {
      if (seg == map[d]) return static_cast<int8_t>(d);
    }

    // Try reversed bit order (maybe wiring/order is reversed)
    uint8_t rev = 0;
    for (int i = 0; i < 7; ++i) rev |= ((seg >> i) & 0x1) << (6 - i);
    for (uint8_t d = 0; d < 10; ++d) if (rev == map[d]) return static_cast<int8_t>(d);

    // If no exact match, treat as invalid (disregard)
    return -1;
  }

  // Decode a 7-seg pattern into a single character used in error codes.
  // Returns '\0' if unknown.
  static char decode_7seg_char(uint8_t seg) {
    // Known letter/dash patterns (approximate common 7-seg shapes)
    const std::pair<uint8_t,char> letters[] = {
      {0b0000001, '-'}, // dash (g)
      {0b0110111, 'H'}, // H
      {0b1111110, 'O'}, // O 
      {0b0110000, 'I'}, // I 
      {0b1001110, 'C'}, // C
      {0b1110111, 'A'}, // A
      {0b0011111, 'b'}, // b
      {0b0001110, 'L'}, // L
      {0b1000111, 'F'}, // F
      {0b0111011, 'Y'}, // Y
      {0b0111101, 'd'}, // d 
      {0b0000101, 'r'}, // r
      {0b1011011, 'S'}, // S (same pattern as '5')
      {0b0010101, 'n'}  // n (segments c,e,g)
    };

    // Prefer letter matches only (we intentionally avoid returning digits here)
    for (auto &p : letters) {
      if (seg == p.first) return p.second;
    }

    // Try reversed bit order too (for wiring/order mismatches)
    uint8_t rev = 0;
    for (int i = 0; i < 7; ++i) rev |= ((seg >> i) & 0x1) << (6 - i);
    for (auto &p : letters) if (rev == p.first) return p.second;

    return '\0';
  }

  // Translate known error codes to plain English
  static const char* translate_error_code(const std::string &code) {
    if (code == "--") return "unknown temperature (expected after power on)";
    if (code == "HH") return "high overheat (water temp over 118 F)";
    if (code == "OH") return "overheat (water temp over 108 F)";
    if (code == "IC" || code == "1C") return "ice possible";
    if (code == "SA") return "Sensor A out of service";
    if (code == "Sb" || code == "5b") return "Sensor B out of service";
    if (code == "Sn") return "sensors out of sync";
    if (code == "HL") return "Significant difference between sensor values";
    if (code == "LF") return "recurring low flow";
    if (code == "dr") return "low flow";
    if (code == "dY") return "Low water";
    return nullptr;
  }

  void setup() override {
    // Configure both pins as inputs (no internal pull); external pull resistors expected
    gpio_config_t io_conf{};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CLK_PIN) | (1ULL << DATA_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Install ISR service and attach to clock pin (rising edge)
    gpio_install_isr_service(0);
    // Use plain C ISR wrapper function to avoid linker relocation issues with C++ static member wrappers
    gpio_isr_handler_add((gpio_num_t)CLK_PIN, &esp32_spa_isr_wrapper, this);
    gpio_set_intr_type((gpio_num_t)CLK_PIN, GPIO_INTR_POSEDGE);

    // Initialize auto-refresh timer to avoid an immediate forced press on boot
    last_set_sent_time_ms = esphome::millis();

    // Ensure COOL button pin is setup as an output (harmless if balboa_custom also configures it)
    gpio_set_direction((gpio_num_t)PIN_WRITE_BTN2, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_WRITE_BTN2, 0);

    // Press COOL shortly after boot to initialize/set the displayed set-temp
    // Press on at 1.5s, release at 1.7s. Update the last_set_sent_time when pressed.
    this->set_timeout("boot_press_cool_on", 1500, [this]() {
      ESP_LOGI(TAG, "Boot: auto-pressing COOL to initialize set temp");
      gpio_set_level((gpio_num_t)PIN_WRITE_BTN2, 1);
      last_set_sent_time_ms = esphome::millis();
    });
    this->set_timeout("boot_press_cool_off", 1700, []() {
      gpio_set_level((gpio_num_t)PIN_WRITE_BTN2, 0);
    });
  }

  void loop() override {
    uint32_t now = esphome::millis();

    // Report any partial/incomplete frames detected by ISR since last check
    uint32_t partials = 0;
    portENTER_CRITICAL(&spinlock_);
    partials = partial_frame_count;
    partial_frame_count = 0;
    if (partials > 0) {
      // Invalidate stored frame here instead of inside ISR to keep ISR short and non-blocking
      last_frame_valid = false;
    }
    portEXIT_CRITICAL(&spinlock_);
    if (partials > 0) {
      ESP_LOGW(TAG, "Dropped %u spurious gaps (clock edges with no accumulated bits)", partials);
    }

    // If no new frame, allow heartbeat publishes of last known value (only if last frame was valid)
    if (!frame_ready) {
      bool heartbeat_due = (now - last_publish_time >= HEARTBEAT_MS);

      // If the set-temp hasn't been captured for a while, force a 'cool' press to make the tub show/publish it
      if ((now - last_set_sent_time_ms) >= SET_FORCE_INTERVAL_MS) {
        ESP_LOGI(TAG, "No set-temp captured for %ums — auto-pressing COOL to refresh set temp", static_cast<unsigned>(now - last_set_sent_time_ms));
        // Activate the physical COOL press (use balboa pin macro)
        gpio_set_level((gpio_num_t)PIN_WRITE_BTN2, 1);
        // Ensure we release it after a short duration (mirror existing press timing)
        this->set_timeout("auto_press_cool", 200, [](){ gpio_set_level((gpio_num_t)PIN_WRITE_BTN2, 0); });
        // Update timer to avoid repeated presses
        last_set_sent_time_ms = now;
        // Also reset heartbeat timing so we don't immediately publish stale data
        last_publish_time = now;
      }

      if (!heartbeat_due) return;

      // Read protected copy of last_frame_valid to avoid races with ISR
      portENTER_CRITICAL(&spinlock_);
      bool lfv = last_frame_valid;
      portEXIT_CRITICAL(&spinlock_);

      if (!lfv) {
        // No valid stored frame — still publish any known stored values (measured/set/binary) so HA sees activity
        if (measured_temp_sensor_ && last_measured_temp >= 0) measured_temp_sensor_->publish_state(static_cast<float>(last_measured_temp));
        if (set_temp_sensor_ && last_set_temp >= 0) set_temp_sensor_->publish_state(static_cast<float>(last_set_temp));
        if (heater_sensor_ && last_heater >= 0) { heater_sensor_->publish_state(static_cast<bool>(last_heater)); }
        if (pump_sensor_ && last_pump >= 0) { pump_sensor_->publish_state(static_cast<bool>(last_pump)); }
        if (light_sensor_ && last_light >= 0) { light_sensor_->publish_state(static_cast<bool>(last_light)); }

        ESP_LOGI(TAG, "Heartbeat publish (stored): measured=%d set=%d heater=%d pump=%d light=%d", last_measured_temp, last_set_temp, last_heater, last_pump, last_light);

        last_publish_time = now;
        return;
      }

      uint32_t out = last_published_value & 0xFFFFFF;

      uint8_t p1 = (out >> 17) & 0x7F;  // top 7 bits
      uint8_t p2 = (out >> 10) & 0x7F;  // next 7 bits
      uint8_t p3 = (out >> 3)  & 0x7F;  // next 7 bits
      uint8_t p4 = out & 0x7;          // last 3 bits

      // Decode/validate exactly like new frames
      // Require p1 to match pattern 0xx0x00 (bits 6,3,1,0 must be zero)
      // AND require the least-significant bit of p4 to be 0 (LSB of status nibble)
      const uint8_t checksum_mask = 0x4B;  // 0b1001011 (mask bits 6,3,1,0)
      const uint8_t checksum_val  = 0x00;  // expected zeros in masked bits
      const uint8_t p4_mask = 0x1;        // require p4 LSB == 0
      if ((p1 & checksum_mask) != checksum_val || (p4 & p4_mask) != 0) {
        ESP_LOGW(TAG, "Heartbeat: stored frame fails checksum (p1 masked=0x%02X expected=0x%02X, p4_lsb=0x%X expected=0x0), not publishing",
                static_cast<unsigned>(p1 & checksum_mask), static_cast<unsigned>(checksum_val), static_cast<unsigned>(p4 & p4_mask));
        return;
      }

      int8_t digit2 = decode_7seg(p2);
      int8_t digit3 = decode_7seg(p3);

      int16_t temp = decode_temp(p1, digit2, digit3);

      // Heartbeat: publish binary sensor states as well
      // Heater is on bit 2 of p1 (observed from hardware)
      int heater_val = static_cast<int>((p1 >> 2) & 0x1);
      int pump_val = static_cast<int>((p4 >> 2) & 0x1);
      int light_val = static_cast<int>((p4 >> 1) & 0x1);

      // Log at info level so this appears even when debug is off
      ESP_LOGI(TAG, "Heartbeat publish: temp=%d set=%d status=0x%X heater=%d pump=%d light=%d", temp, last_set_temp, static_cast<unsigned>(p4), heater_val, pump_val, light_val);

      if (measured_temp_sensor_ && temp >= 0) measured_temp_sensor_->publish_state(static_cast<float>(temp));
      if (set_temp_sensor_ && last_set_temp >= 0) set_temp_sensor_->publish_state(static_cast<float>(last_set_temp));
      if (heater_sensor_) { heater_sensor_->publish_state(static_cast<bool>(heater_val)); last_heater = heater_val; }
      if (pump_sensor_) { pump_sensor_->publish_state(static_cast<bool>(pump_val)); last_pump = pump_val; }
      if (light_sensor_) { light_sensor_->publish_state(static_cast<bool>(light_val)); last_light = light_val; }

      // If p2/p3 form a valid temperature, clear any previous error and skip error processing
      if (temp >= 0) {
        if (!last_error_code_.empty()) {
          if (error_text_sensor_) error_text_sensor_->publish_state("");
          last_error_code_.clear(); candidate_error.clear(); stable_error = 0;
        }
      } else if (error_text_sensor_) {
        char c2 = decode_7seg_char(p2);
        char c3 = decode_7seg_char(p3);
        std::string code = "";
        code.push_back(c2 != '\0' ? c2 : '?');
        code.push_back(c3 != '\0' ? c3 : '?');
        const char *trans = translate_error_code(code);

        // Treat any decoded (non-blank) character sequence as a candidate error, or a known translation
        if (trans != nullptr || (c2 != '\0' || c3 != '\0')) {
          if (candidate_error == code) { if (stable_error < 255) stable_error++; } else { candidate_error = code; stable_error = 1; }
          if (stable_error >= ERROR_STABLE_THRESHOLD && code != last_error_code_) {
            if (trans) error_text_sensor_->publish_state(code + std::string(" - ") + trans);
            else error_text_sensor_->publish_state(code);
            last_error_code_ = code;
          }
        } else {
          // Not an error -> reset candidate
          candidate_error.clear(); stable_error = 0;
        }
      }

      last_publish_time = now;
      first_publish = false;
      return;
    }

    // Copy shared data under spinlock to avoid races with ISR
    portENTER_CRITICAL(&spinlock_);
    uint64_t value = completed_frame;
    uint8_t  n_bits = completed_bit_count;
    frame_ready = false;
    portEXIT_CRITICAL(&spinlock_);

    discovery_record_frame(n_bits, value);

    // Publish every observed frame so discovery tooling can count repeats and cluster by length.
    if (raw_frame_sensor_) {
      char bits[128];
      size_t pos = 0;
      int written = snprintf(bits + pos, sizeof(bits) - pos, "%u|%u:",
        static_cast<unsigned>(discovery_frame_seq_), static_cast<unsigned>(n_bits));
      if (written > 0) pos += static_cast<size_t>(written);
      append_bits(bits + pos, sizeof(bits) - pos, value, n_bits);
      raw_frame_sensor_->publish_state(std::string(bits));
    }

    // For protocol discovery, only keep the legacy temperature/state decoder when the frame is 24 bits.
    if (n_bits != 24) {
      last_frame_valid = false;
      return;
    }

    // Decode the frame using the lower 24 bits for GS100-compatible decoding.
    uint32_t out = static_cast<uint32_t>(value) & 0xFFFFFF;

    // Split into parts
    uint8_t p1 = (out >> 17) & 0x7F;  // top 7 bits
    uint8_t p2 = (out >> 10) & 0x7F;  // next 7 bits
    uint8_t p3 = (out >> 3)  & 0x7F;  // next 7 bits
    uint8_t p4 = out & 0x7;          // last 3 bits

    // Verify checksum to filter invalid frames
    const uint8_t checksum_mask = 0x4B;  // 0b1001011 (mask bits 6,3,1,0)
    const uint8_t checksum_val  = 0x00;  // expected zeros in masked bits
    const uint8_t p4_mask = 0x1;        // require p4 LSB == 0
    if ((p1 & checksum_mask) != checksum_val || (p4 & p4_mask) != 0) {
      ESP_LOGW(TAG, "Frame fails checksum (p1 masked=0x%02X expected=0x%02X, p4_lsb=0x%X), ignoring",
                static_cast<unsigned>(p1 & checksum_mask), static_cast<unsigned>(checksum_val), static_cast<unsigned>(p4 & p4_mask));
      // Treat as non-existent frame
      last_frame_valid = false;
      return;
    }

    // Small debug: log raw frame and parts
    ESP_LOGD(TAG, "Frame received raw=0x%06X p1=0x%02X p2=0x%02X p3=0x%02X p4=0x%X", out, static_cast<unsigned>(p1), static_cast<unsigned>(p2), static_cast<unsigned>(p3), static_cast<unsigned>(p4));

    // Decode the 7-seg patterns to digits
    int8_t digit2 = decode_7seg(p2);
    int8_t digit3 = decode_7seg(p3);

    // Check if this is a zero display (both raw bytes are 0x00).
    // Previously required decoded digits to be 0 too, but blank frames decode to -1 and were missed.
    bool is_zero = (p2 == 0x00 && p3 == 0x00);

    // Stability update for zero detection
    if (candidate_is_zero == is_zero) {
      if (stable_zero < 255) stable_zero++;
    } else {
      candidate_is_zero = is_zero;
      stable_zero = 1;
    }

    if (is_zero) {
      ESP_LOGD(TAG, "Zero raw detected: p2=0x%02X p3=0x%02X decoded d2=%d d3=%d", static_cast<unsigned>(p2), static_cast<unsigned>(p3), digit2, digit3);
    }

    // Decode temperature if not zero
    int16_t temp = -1;
    if (!is_zero && digit2 >= 0 && digit3 >= 0) {
      temp = decode_temp(p1, digit2, digit3);
    }

    // Decode/publish any error-code text (p2/p3) but only after it is stable and looks like an error
    if (error_text_sensor_) {
      // If the frame decodes to a numeric temperature, don't treat as an error
      if (temp >= 0) {
        candidate_error.clear(); stable_error = 0;
      } else {
        char c2 = decode_7seg_char(p2);
        char c3 = decode_7seg_char(p3);
        std::string code = "";
        code.push_back(c2 != '\0' ? c2 : '?');
        code.push_back(c3 != '\0' ? c3 : '?');
        const char *trans = translate_error_code(code);

        // Treat any decoded (non-blank) character sequence as a candidate error, or a known translation
        if (trans != nullptr || (c2 != '\0' || c3 != '\0')) {
          if (candidate_error == code) { if (stable_error < 255) stable_error++; } else { candidate_error = code; stable_error = 1; }
          if (stable_error >= ERROR_STABLE_THRESHOLD && code != last_error_code_) {
            if (trans) error_text_sensor_->publish_state(code + std::string(" - ") + trans);
            else error_text_sensor_->publish_state(code);
            last_error_code_ = code;
          }
        } else {
          // Not an error -> reset candidate tracking
          candidate_error.clear(); stable_error = 0;
        }
      }
    }

    // Stability update for temperature
    if (candidate_temp == temp) {
      if (stable_temp < 255) stable_temp++;
    } else {
      candidate_temp = temp;
      stable_temp = 1;
    }

    // Record when we last saw a candidate temperature (even if transient)
    if (candidate_temp >= 0) {
      last_candidate_temp_time = now;
    }

    // Check if we should commit stable values
    bool zero_stable = (stable_zero >= STABLE_THRESHOLD);
    bool temp_stable = (stable_temp >= STABLE_THRESHOLD);

    // Set mode logic: detect 0x00 alternation pattern
    if (zero_stable && candidate_is_zero) {
      // We just saw stable zeros - update last_zero_seen_time
      last_zero_seen_time = now;

      // If we saw a recent candidate temp (even just-before the zero), accept it as potential
      if (set_temp_potential < 0 && candidate_temp >= 0 && (now - last_candidate_temp_time <= 3000)) {
        set_temp_potential = candidate_temp; // raw numeric from display
        ESP_LOGD(TAG, "Zero detected and recent candidate found: set_temp_potential=%d (age=%ums)", set_temp_potential, static_cast<unsigned>(now - last_candidate_temp_time));
      }
      in_set_mode = true;
      // Cancel any pending measured-temp publish because set mode is starting
      pending_measured_temp = -1;
      pending_measured_since = 0;
      ESP_LOGD(TAG, "Zero detected (0x00), entering/staying in set mode");
    } else if (candidate_temp >= 0 && in_set_mode) {
      // We have observed a non-zero temp while already in set mode. Set as potential immediately
      int16_t display_candidate = candidate_temp; // raw numeric from display
      if (set_temp_potential != display_candidate) {
        set_temp_potential = display_candidate;
        last_candidate_temp_time = now;
        ESP_LOGD(TAG, "Set temp potential updated (transient): %d", set_temp_potential);
      } else {
        // refresh timestamp even if same potential
        last_candidate_temp_time = now;
      }
    }

    // Check if we should exit set mode (no zeros for 5 seconds)
    if (in_set_mode && (now - last_zero_seen_time >= SET_MODE_TIMEOUT_MS)) {
      in_set_mode = false;
      set_temp_potential = -1;
      ESP_LOGD(TAG, "Exited set mode (timeout)");
    }

    // Publish set temp if we have a potential and see another zero
    if (zero_stable && candidate_is_zero && set_temp_potential >= 0 && set_temp_potential != last_set_temp) {
      // Optional safety: ensure the candidate temp was seen recently (within 3s) to avoid stale data
      if (now - last_candidate_temp_time <= 3000) {
        last_set_temp = set_temp_potential;
        if (set_temp_sensor_) {
          set_temp_sensor_->publish_state(static_cast<float>(last_set_temp));
          ESP_LOGD(TAG, "Publishing set temp: %d [confirmed by zero]", last_set_temp);
        }
        // Reset the auto-refresh timer since we successfully captured & published a set temp
        last_set_sent_time_ms = now;
        last_publish_time = now;
      } else {
        ESP_LOGW(TAG, "Set temp potential too old (%ums), ignoring", static_cast<unsigned>(now - last_candidate_temp_time));
      }
    }

    // Publish measured temp, but wait a short time to ensure we are not entering set mode
    if (temp_stable && candidate_temp >= 0 && candidate_temp != last_measured_temp) {
      // When a stable numeric temperature is visible, clear any previously-published error code
      if (last_error_code_ != "") {
        if (error_text_sensor_) error_text_sensor_->publish_state("");
        last_error_code_.clear(); candidate_error.clear(); stable_error = 0;
      }
      if (in_set_mode) {
        // If we're in set mode, drop any candidate
        pending_measured_temp = -1;
        pending_measured_since = 0;
      } else {
        // Not in set mode: start or evaluate pending timer
        int16_t display_candidate = candidate_temp; // raw numeric from display
        if (pending_measured_temp != display_candidate) {
          // New candidate: start pending timer
          pending_measured_temp = display_candidate;
          pending_measured_since = now;
          ESP_LOGD(TAG, "Measured temp candidate %d pending, waiting %ums to ensure not set-mode", display_candidate, static_cast<unsigned>(MEASURE_PUBLISH_DELAY_MS));
        } else if ((now - pending_measured_since) >= MEASURE_PUBLISH_DELAY_MS) {
          // Timer elapsed and still not in set mode -> publish
          last_measured_temp = pending_measured_temp;
          if (measured_temp_sensor_) {
            measured_temp_sensor_->publish_state(static_cast<float>(last_measured_temp));
            ESP_LOGD(TAG, "Publishing measured temp: %d", last_measured_temp);
          }
          last_publish_time = now;
          pending_measured_temp = -1;
          pending_measured_since = 0;
        }
      }
    } else {
      // No stable candidate or candidate changed -> clear any pending measured temp
      if (candidate_temp < 0 || pending_measured_temp != candidate_temp) {
        pending_measured_temp = -1;
        pending_measured_since = 0;
      }
    }

    // Always update binary sensors from p4 and p1 with per-bit stability
    uint8_t p1_bits = p1;
    int8_t cur_heater = static_cast<int8_t>((p1_bits >> 2) & 0x1);
    int8_t cur_pump = static_cast<int8_t>((p4 >> 2) & 0x1);
    int8_t cur_light = static_cast<int8_t>((p4 >> 1) & 0x1);

    // Update heater stability (existing)
    if (candidate_heater == cur_heater) { if (stable_heater < 255) stable_heater++; } else { candidate_heater = cur_heater; stable_heater = 1; }

    // Update pump stability
    if (candidate_pump == cur_pump) {
      if (stable_pump < 255) stable_pump++;
    } else {
      candidate_pump = cur_pump; stable_pump = 1;
    }

    // Update light stability
    if (candidate_light == cur_light) {
      if (stable_light < 255) stable_light++;
    } else {
      candidate_light = cur_light; stable_light = 1;
    }

    // Determine which values are stable enough to publish
    bool pump_ok = (stable_pump >= PUMP_STABLE_THRESHOLD);
    bool light_ok = (stable_light >= STABLE_THRESHOLD);

    // Heater hysteresis: turn ON immediately when bit set; only turn OFF after it has been clear for HEATER_OFF_TIMEOUT_MS
    int8_t pub_heater = last_heater;
    if (cur_heater == 1) {
      // immediate on
      pub_heater = 1;
      last_heater_off_time = 0;
    } else {
      // cur_heater == 0
      if (last_heater == 1) {
        if (last_heater_off_time == 0) last_heater_off_time = now;
        if ((now - last_heater_off_time) >= HEATER_OFF_TIMEOUT_MS) {
          pub_heater = 0;
          last_heater_off_time = 0;
        } else {
          pub_heater = 1; // stay on until timeout
        }
      } else {
        pub_heater = 0;
      }
    }

    int8_t pub_pump = pump_ok ? candidate_pump : last_pump;
    int8_t pub_light = light_ok ? candidate_light : last_light;

    bool binary_changed = (pub_heater != last_heater || pub_pump != last_pump || pub_light != last_light);
    if (binary_changed) {
      ESP_LOGD(TAG, "Binary sensors updated: heater=%d pump=%d light=%d (stable: h=%u p=%u l=%u)", pub_heater, pub_pump, pub_light, static_cast<unsigned>(stable_heater), static_cast<unsigned>(stable_pump), static_cast<unsigned>(stable_light));
      if (heater_sensor_) { heater_sensor_->publish_state(static_cast<bool>(pub_heater)); last_heater = pub_heater; }
      if (pump_sensor_) { pump_sensor_->publish_state(static_cast<bool>(pub_pump)); last_pump = pub_pump; }
      if (light_sensor_) { light_sensor_->publish_state(static_cast<bool>(pub_light)); last_light = pub_light; }

      last_published_value = value;
      last_publish_time = now;
      first_publish = false;
      portENTER_CRITICAL(&spinlock_);
      last_frame_valid = true;
      portEXIT_CRITICAL(&spinlock_);
    } else {
      // No change; do not publish
      ESP_LOGD(TAG, "No changes detected");
    }
  }



  // Public dispatcher safely callable from C ISR wrapper
  void IRAM_ATTR handle_isr() { this->on_clock_edge_isr(); }

 private:
  // Spinlock for protecting shared variables between ISR and loop
  portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;

  // ISR timing for frame gap detection (CPU cycle-count of last clock edge)
  // We avoid esp_timer_get_time() in ISR; use CPU cycle count and a fixed NOP delay to sample later.
  volatile uint32_t last_clock_ccount = 0;  // low-overhead 32-bit cycle counter (wraps naturally)

  // Completed frame buffer (written by ISR on gap, read by loop()).
  // uint64_t supports up to 64 bits; actual bit count is stored separately.
  volatile uint64_t completed_frame = 0;
  volatile uint8_t  completed_bit_count = 0;

  // Count partial/incomplete frames detected by ISR (incremented when a gap resets a 0-bit frame)
  volatile uint32_t partial_frame_count = 0;

  // CPU frequency assumptions and derived constants for timing
  static constexpr uint32_t CPU_MHZ = 240u;                // ESP32 clock (MHz)
  static constexpr uint32_t CYCLES_PER_US = CPU_MHZ;       // cycles per microsecond
  static constexpr uint32_t FRAME_GAP_CYCLES = FRAME_GAP_US * CYCLES_PER_US;

  // Fixed-cycle sampling delay implemented with cycle-count busy-wait to sample after clock rising edge
  // 240 MHz -> 240 cycles/us -> 8 us -> 1920 cycles
  static constexpr uint32_t SAMPLE_DELAY_US = 1u;          // target sample delay in microseconds
  static constexpr uint32_t SAMPLE_DELAY_CYCLES = SAMPLE_DELAY_US * CYCLES_PER_US;

  // Read cycle counter (IRAM safe)
  static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile ("rsr.ccount %0" : "=a" (ccount));
    return ccount;
  }

  // Removed C++ static wrapper to avoid relocation/linker issues. A plain C ISR wrapper is defined at global scope.

  void IRAM_ATTR on_clock_edge_isr() {
    // ISR: frame boundary is the inter-frame gap, not a fixed bit count.
    // On a gap: commit whatever was accumulated as a completed frame, then start fresh.
    // This correctly handles any frame length up to 64 bits.

    portENTER_CRITICAL_ISR(&spinlock_);

    uint32_t now_ccount = get_cycle_count();
    if (last_clock_ccount != 0 && (now_ccount - last_clock_ccount) > FRAME_GAP_CYCLES) {
      if (bit_count > 0) {
        // Commit the accumulated frame
        completed_frame = shift_reg;
        completed_bit_count = bit_count;
        frame_ready = true;
      } else {
        // Gap with no bits — spurious gap, ignore
        partial_frame_count++;
      }
      shift_reg = 0;
      bit_count = 0;
    }
    last_clock_ccount = now_ccount;

    // Exit critical before busy-wait to minimise ISR latency impact on other interrupts
    uint32_t start_ccount = now_ccount;
    portEXIT_CRITICAL_ISR(&spinlock_);

    // Busy-wait for data line to settle
    while ((get_cycle_count() - start_ccount) < SAMPLE_DELAY_CYCLES) {
      asm volatile ("nop");
    }

    portENTER_CRITICAL_ISR(&spinlock_);
    bool bit = gpio_get_level((gpio_num_t)DATA_PIN);
    // Cap at 64 bits to avoid overflow; extra bits are silently dropped
    if (bit_count < 64) {
      shift_reg = (shift_reg << 1) | static_cast<uint64_t>(bit);
      bit_count++;
    }
    portEXIT_CRITICAL_ISR(&spinlock_);
  }
};

}  // namespace esp32_spa

// Plain C ISR wrapper placed in IRAM to avoid dangerous relocations when linking C++ static member wrappers.
extern "C" void IRAM_ATTR esp32_spa_isr_wrapper(void* arg) {
  auto *self = static_cast<esp32_spa::HotTubDisplaySensor*>(arg);
  if (self) self->handle_isr();
}