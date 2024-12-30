#include "obd_component.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/canbus/canbus.h"

namespace esphome {
namespace obd {

static const char *const TAG = "obd";

// Skip setup from PollingComponent to stop polling from starting automatically
void OBDComponent::call_setup() {
  if (this->enabled_by_default_) {
    this->start_poller();
  }
}

void OBDComponent::dump_config() { ESP_LOGCONFIG(TAG, "OBD Component"); }

void OBDComponent::update() {
  if (this->current_request_ == nullptr) {
    // No current request, find the next request to do
    for (auto *request : this->pidrequests_) {
      if (request->start()) {
        this->current_request_ = request;
        return;
      }
    }

    // No pid to be polled, exit loop
    return;
  } else {
    if (this->current_request_->update()) {
      this->current_request_ = nullptr;
    }
  }
}

void OBDComponent::send(std::uint32_t can_id, bool use_extended_id, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  std::vector<uint8_t> data(8, 0xAA);
  data[0] = a;
  data[1] = b;
  data[2] = c;
  data[3] = d;

  this->canbus_->send_data(can_id, use_extended_id, data);
}

void OBDComponent::send(std::uint32_t can_id, bool use_extended_id, uint8_t a, uint8_t b, uint8_t c) {
  std::vector<uint8_t> data(8, 0xAA);
  data[0] = a;
  data[1] = b;
  data[2] = c;

  this->canbus_->send_data(can_id, use_extended_id, data);
}

void OBDComponent::add_pidrequest(PIDRequest *request) {
  ESP_LOGVV(TAG, "add request for canid=0x%03, pid=0x%06" PRIx32, request->can_id_, PRIx32, request->pid_);
  this->pidrequests_.push_back(request);
};

void PIDRequest::setup() {
  this->parent_->add_pidrequest(this);

  auto trigger = new OBDCanbusTrigger(this);
  trigger->setup();
}

bool PIDRequest::start() {
  if (this->state_ != WAITING)
    return false;

  if ((this->last_polled_ + this->interval_) >= millis())
    return false;

  this->response_buffer_.clear();
  this->response_buffer_.reserve(this->reply_length_);

  auto can_id = this->can_id_;
  auto pid = this->pid_;

  ESP_LOGD(TAG, "polling can_id: 0x%03x for pid 0x%04x", can_id, pid);

  if (pid > 0xFFFF) {
    // 24 bit pid
    this->parent_->send(can_id, this->use_extended_id_, 0x03, (pid >> 16) & 0xFF, (pid >> 8) & 0xFF, pid & 0xFF);
  } else {
    this->parent_->send(can_id, this->use_extended_id_, 0x02, (pid >> 8) & 0xFF, pid & 0xFF);
  }

  this->last_polled_ = millis();
  this->state_ = POLLING;

  return true;
}

bool PIDRequest::update() {
  if (this->state_ != POLLING)
    return true;  // Invalid state, update should not have been called here

  if ((this->last_polled_ + this->timeout_) > millis()) {
    return false;
  }

  if (this->response_buffer_.size() < this->reply_length_) {
    ESP_LOGD(TAG, "timeout for polling can_id: 0x%03x for pid 0x%04x", this->can_id_, this->pid_);
    this->state_ = WAITING;
    return true;
  }

  for (auto *trigger : this->triggers_) {
    trigger->trigger(this->response_buffer_);
  }

  for (auto *sensor : this->sensors_) {
    sensor->update(this->response_buffer_);
  }

  this->state_ = WAITING;
  return true;
}

void PIDRequest::handle_incoming(std::vector<uint8_t> &data) {
  if (this->state_ != POLLING)
    return;  // Not our cup of tea here, some other pid might be polling on the same can_id

  ESP_LOGD(TAG, "recieved content for pid 0x%04x: %s", this->pid_, format_hex_pretty(data).c_str());

  // Handle the data
  if ((data[0] & 0xF0) == 0x10) {
    // This is a flow control frame, we should ask for more
    this->parent_->send(this->can_id_, this->use_extended_id_, 0x30, 0x0, 0x10);
  }

  for (int i = 0; i < data.size(); i++) {
    this->response_buffer_.push_back(data[i]);
  }
}

void PIDRequest::dump_config() {
  ESP_LOGCONFIG(TAG, "PID Request 0x%03", this->pid_);

  if (this->use_extended_id_) {
    ESP_LOGCONFIG(TAG, "  Can extended id: 0x%08" PRIx32, this->can_id_);
    ESP_LOGCONFIG(TAG, "  Can response id: 0x%08" PRIx32, this->can_response_id_);
  } else {
    ESP_LOGCONFIG(TAG, "  Can id: 0x%03" PRIx32, this->can_id_);
    ESP_LOGCONFIG(TAG, "  Can response id: 0x%03" PRIx32, this->can_response_id_);
  }

  ESP_LOGCONFIG(TAG, "  Pid: 0x%03" PRIx32, this->pid_);
  ESP_LOGCONFIG(TAG, "  Interval: %ims", this->interval_);
  ESP_LOGCONFIG(TAG, "  Timeout: %ims", this->timeout_);
}

void OBDSensor::update(const std::vector<uint8_t> &data) {
  auto value = this->data_to_value_func_(data);
  this->publish_state(value);
}

void OBDSensor::dump_config() { LOG_SENSOR("", "OBD Sensor", this); }

void OBDBinarySensor::update(const std::vector<uint8_t> &data) {
  auto value = this->data_to_value_func_(data);
  this->publish_state(value);
}

void OBDBinarySensor::dump_config() { LOG_BINARY_SENSOR("", "OBD Binary Sensor", this); }

}  // namespace obd
}  // namespace esphome
