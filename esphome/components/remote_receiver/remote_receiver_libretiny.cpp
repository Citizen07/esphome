#include "remote_receiver.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_LIBRETINY

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.libretiny";

void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg) {
  const uint32_t next = (arg->buffer_write_at + 1) % arg->buffer_size;
  // If next is buffer_read, we have hit an overflow
  if (next == arg->buffer_read_at) {
    arg->overflow = true;
    return;
  }
  arg->overflow = false;

  // If the lhs is 1 (rising edge) we should write to an uneven index and vice versa
  const bool level = arg->pin.digital_read();
  if (level != next % 2)
    return;

  const uint32_t last_change = arg->buffer[arg->buffer_write_at];
  const uint32_t now = micros();
  const uint32_t time_since_change = now - last_change;
  if (time_since_change <= arg->filter_us)
    return;
  if (time_since_change >= arg->idle_us) {
    // Check if we received a consecutive idle signal which was not processed yet.
    if (arg->buffer_idle_at == arg->buffer_write_at && arg->buffer_read_at != arg->buffer_idle_at) {
      const uint32_t prev_write_at = (arg->buffer_size + arg->buffer_write_at - 1) % arg->buffer_size;
      // Remove the previous idle signal if it was a consecutive and even it was not processed.
      if (last_change - arg->buffer[prev_write_at] >= arg->idle_us && prev_write_at != arg->buffer_read_at) {
        arg->buffer_write_at = prev_write_at;
        arg->buffer_idle_at = prev_write_at;
        arg->buffer[prev_write_at] = now;
        return;
      }
    }
    arg->buffer_idle_at = next;
  }

  arg->buffer[arg->buffer_write_at = next] = now;
}

void RemoteReceiverComponent::setup() {
  this->pin_->setup();
  auto &s = this->store_;
  s.filter_us = this->filter_us_;
  s.idle_us = this->idle_us_;
  s.pin = this->pin_->to_isr();
  s.buffer_size = this->buffer_size_;

  this->high_freq_.start();
  if (s.buffer_size % 2 != 0) {
    // Make sure divisible by two. This way, we know that every 0bxxx0 index is a space and every 0bxxx1 index is a mark
    s.buffer_size++;
  }

  s.buffer = new uint32_t[s.buffer_size];
  void *buf = (void *) s.buffer;
  memset(buf, 0, s.buffer_size * sizeof(uint32_t));

  // First index is a space.
  if (this->pin_->digital_read()) {
    s.buffer_write_at = s.buffer_read_at = s.buffer_idle_at = 1;
  } else {
    s.buffer_write_at = s.buffer_read_at = s.buffer_idle_at = 0;
  }
  this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
}
void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  LOG_PIN("  Pin: ", this->pin_);
  if (this->pin_->digital_read()) {
    ESP_LOGW(TAG, "Remote Receiver Signal starts with a HIGH value. Usually this means you have to "
                  "invert the signal using 'inverted: True' in the pin schema!");
  }
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
  const uint32_t idle_at = s.buffer_idle_at;
  const uint32_t dist = (s.buffer_size + idle_at - s.buffer_read_at) % s.buffer_size;
  // signals must at least one rising and one leading edge
  if (dist <= 1)
    return;

  const uint32_t now = micros();
  ESP_LOGVV(TAG, "read_at=%u write_at=%u dist=%u now=%u end=%u", s.buffer_read_at, write_at, dist, now,
            s.buffer[write_at]);

  // Skip first value, it's from the previous idle level
  s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
  uint32_t prev = s.buffer_read_at;
  uint32_t read_at = (s.buffer_read_at + 1) % s.buffer_size;
  const uint32_t reserve_size = 1 + (s.buffer_size + idle_at - read_at) % s.buffer_size;
  this->temp_.clear();
  this->temp_.reserve(reserve_size);
  int32_t multiplier = read_at % 2 == 0 ? 1 : -1;

  for (uint32_t i = 0; prev != idle_at; i++) {
    int32_t delta = s.buffer[read_at] - s.buffer[prev];
    if (uint32_t(delta) >= this->idle_us_) {
      if (this->temp_.empty()) {
        prev = s.buffer_read_at = read_at;
        read_at = (read_at + 1) % s.buffer_size;
        multiplier *= -1;
        continue;
      }

      // already found a space longer than idle. There must have been more than one pulse
      break;
    }

    ESP_LOGVV(TAG, "  i=%u buffer[%u]=%u - buffer[%u]=%u -> %d", i, read_at, s.buffer[read_at], prev, s.buffer[prev],
              multiplier * delta);
    this->temp_.push_back(multiplier * delta);
    prev = s.buffer_read_at = read_at;
    read_at = (read_at + 1) % s.buffer_size;
    multiplier *= -1;
  }
  this->temp_.push_back(this->idle_us_ * multiplier);

  this->call_listeners_dumpers_();
}

}  // namespace remote_receiver
}  // namespace esphome

#endif
