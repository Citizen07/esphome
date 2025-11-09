#include "remote_receiver.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#if defined(USE_LIBRETINY) || defined(USE_ESP8266)

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver";

void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg) {
  const bool curr_level = arg->pin.digital_read();
  const uint32_t curr_micros = micros();
  const bool prev_level = arg->prev_level;
  const uint32_t prev_micros = arg->prev_micros;

  arg->prev_micros = curr_micros;
  arg->prev_level = curr_level;

  // filter out short pulses or if the level is the same
  if (curr_micros - prev_micros < arg->filter_us || prev_level == curr_level) {
    return;
  }

  // commit if prev level is different from last level in the buffer
  uint32_t buffer_write_at = arg->buffer_write_at + 1;
  if (buffer_write_at >= arg->buffer_size) {
    buffer_write_at = 0;
  }
  if (prev_level == buffer_write_at % 2) {
    arg->buffer[buffer_write_at] = prev_micros;
    arg->buffer_write_at = buffer_write_at;
  }
}

void RemoteReceiverComponent::setup() {
  this->pin_->setup();

  uint32_t curr_micros = micros();
  bool curr_level = this->pin_->digital_read();
  auto &s = this->store_;
  s.filter_us = this->filter_us_;
  s.pin = this->pin_->to_isr();
  s.prev_micros = curr_micros;
  s.prev_level = curr_level;
  s.buffer_write_at = curr_level;
  s.buffer_read_at = curr_level;
  s.buffer_size = (this->buffer_size_ + 1) & ~1;  // round up to the nearest even number
  s.buffer = new uint32_t[s.buffer_size];
  memset((void *) s.buffer, 0, s.buffer_size * sizeof(uint32_t));
  this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
  this->high_freq_.start();
}

void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG,
                "  Buffer Size: %u\n"
                "  Tolerance: %u%s\n"
                "  Filter out pulses shorter than: %u us\n"
                "  Signal is done after %u us of no changes",
                this->buffer_size_, this->tolerance_,
                (this->tolerance_mode_ == remote_base::TOLERANCE_MODE_TIME) ? " us" : "%", this->filter_us_,
                this->idle_us_);
}

void RemoteReceiverComponent::loop() {
  auto &s = this->store_;

  // copy write at to local variables, as it's volatile
  const uint32_t write_at = s.buffer_write_at;
  const uint32_t dist = (s.buffer_size + write_at - s.buffer_read_at) % s.buffer_size;
  // signals must at least one rising and one leading edge
  if (dist <= 1)
    return;
  const uint32_t now = micros();
  if (now - s.buffer[write_at] < this->idle_us_ * 2) {
    // The last change was fewer than the configured idle time ago.
    return;
  }

  ESP_LOGVV(TAG, "read_at=%u write_at=%u dist=%u now=%u end=%u", s.buffer_read_at, write_at, dist, now,
            s.buffer[write_at]);

  // Skip first value, it's from the previous idle level
  s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
  uint32_t prev = s.buffer_read_at;
  s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
  const uint32_t reserve_size = 1 + (s.buffer_size + write_at - s.buffer_read_at) % s.buffer_size;
  this->temp_.clear();
  this->temp_.reserve(reserve_size);
  int32_t multiplier = s.buffer_read_at % 2 == 0 ? 1 : -1;

  for (uint32_t i = 0; prev != write_at; i++) {
    int32_t delta = s.buffer[s.buffer_read_at] - s.buffer[prev];
    if (uint32_t(delta) >= this->idle_us_) {
      // already found a space longer than idle. There must have been two pulses
      break;
    }

    ESP_LOGVV(TAG, "  i=%u buffer[%u]=%u - buffer[%u]=%u -> %d", i, s.buffer_read_at, s.buffer[s.buffer_read_at], prev,
              s.buffer[prev], multiplier * delta);
    this->temp_.push_back(multiplier * delta);
    prev = s.buffer_read_at;
    s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
    multiplier *= -1;
  }
  s.buffer_read_at = (s.buffer_size + s.buffer_read_at - 1) % s.buffer_size;
  this->temp_.push_back(this->idle_us_ * multiplier);

  this->call_listeners_dumpers_();
}

}  // namespace remote_receiver
}  // namespace esphome

#endif
