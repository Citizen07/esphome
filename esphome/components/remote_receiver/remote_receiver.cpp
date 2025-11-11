#include "remote_receiver.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#if defined(USE_LIBRETINY) || defined(USE_ESP8266)

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver";

void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg) {
  // invert level so it matches the level of the signal before the edge
  const bool curr_level = !arg->pin.digital_read();
  const uint32_t curr_micros = micros();
  const bool prev_level = arg->prev_level;
  const uint32_t prev_micros = arg->prev_micros;

  // store prev for next interrupt
  arg->prev_micros = curr_micros;
  arg->prev_level = curr_level;

  // filter out short pulses or if the level is the same
  if (curr_micros - prev_micros < arg->filter_us || prev_level == curr_level) {
    return;
  }

  // commit prev if level is different from the last commit level
  const bool commit_level = arg->commit_level;
  const uint32_t commit_micros = arg->commit_micros;
  if (prev_level != commit_level) {
    uint32_t buffer_write = arg->buffer_write;
    int32_t delta = std::min(arg->idle_us, prev_micros - commit_micros);
    int32_t multiplier = ((int32_t) prev_level << 1) - 1;
    arg->buffer[buffer_write++] = delta * multiplier;
    if (buffer_write >= arg->buffer_size) {
      buffer_write = 0;
    }
    // check for overflow are reset write pointer if necessary
    if (buffer_write == arg->buffer_read) {
      buffer_write = arg->buffer_start;
      arg->overflow = true;
    }
    // start a new sequence if the idle time has been exceeded
    if (delta >= arg->idle_us) {
      arg->buffer_start = buffer_write;
    }
    arg->buffer_write = buffer_write;
    arg->commit_micros = prev_micros;
    arg->commit_level = prev_level;
  }
}

void RemoteReceiverComponent::setup() {
  this->pin_->setup();
  this->store_.idle_us = this->idle_us_;
  this->store_.filter_us = this->filter_us_;
  this->store_.pin = this->pin_->to_isr();
  this->store_.buffer = new int32_t[this->buffer_size_];
  this->store_.buffer_size = this->buffer_size_;
  this->store_.prev_micros = this->store_.commit_micros = micros();
  this->store_.prev_level = this->store_.commit_level = this->pin_->digital_read();
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

  // check for data again
  const uint32_t last_index = s.buffer_start;
  if (last_index == s.buffer_read) {
    return;
  }

  // find the size of the buffer and reserve the memory
  uint32_t temp_read = s.buffer_read;
  uint32_t reserve_size = 0;
  while (temp_read != last_index && std::abs(s.buffer[temp_read++]) < this->idle_us_) {
    if (temp_read >= s.buffer_size) {
      temp_read = 0;
    }
    reserve_size++;
  }
  this->temp_.clear();
  this->temp_.reserve(reserve_size + 1);

  // read the buffer
  for (uint32_t i = 0; i < reserve_size + 1; i++) {
    this->temp_.push_back((int32_t) s.buffer[s.buffer_read++]);
    if (s.buffer_read >= s.buffer_size) {
      s.buffer_read = 0;
    }
  }

  // call the listeners and dumpers
  this->call_listeners_dumpers_();
}

}  // namespace remote_receiver
}  // namespace esphome

#endif
