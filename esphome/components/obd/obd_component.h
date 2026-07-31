#include "esphome/core/component.h"
#include "esphome/core/version.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#pragma once

namespace esphome {
namespace obd {

class PIDRequest;
class OBDPidTrigger;
class OBDSensorBase;
class OBDSensor;

enum polling_speed_t {
  POLLING_FAST,
  POLLING_SLOW,
};

class OBDComponent : public PollingComponent {
  friend class OBDCanbusTrigger;

 public:
  explicit OBDComponent() : PollingComponent(100) {}
  void call_setup() override;
  void update() override;

  void set_canbus(canbus::Canbus *canbus) { this->canbus_ = canbus; }
  void set_enabled_by_default(bool enabled_by_default) { this->enabled_by_default_ = enabled_by_default; }
  void set_polling_speed(polling_speed_t speed) { this->polling_speed_ = speed; }
  polling_speed_t get_polling_speed() const { return this->polling_speed_; }

  void add_pidrequest(PIDRequest *request);
  void dump_config() override;

  void send(std::uint32_t can_id, bool use_extended_id, uint8_t a, uint8_t b, uint8_t c);
  void send(std::uint32_t can_id, bool use_extended_id, uint8_t a, uint8_t b, uint8_t c, uint8_t d);

 protected:
  canbus::Canbus *canbus_{nullptr};
  bool enabled_by_default_{false};
  polling_speed_t polling_speed_{POLLING_FAST};

  std::vector<PIDRequest *> pidrequests_{};

  PIDRequest *current_request_{nullptr};
};

using data_to_value_t = std::function<float(std::vector<uint8_t>)>;
using data_to_bool_t = std::function<float(std::vector<uint8_t>)>;

enum request_state_t {
  WAITING,
  WAKING,
  POLLING,
};

class PIDRequest : public Component {
  friend class OBDCanbusTrigger;

 public:
  explicit PIDRequest(OBDComponent *parent, const std::uint32_t can_id, const std::uint32_t pid,
                      const std::uint32_t can_response_id, const bool use_extended_id)
      : parent_(parent),
        can_id_(can_id),
        can_response_id_(can_response_id),
        pid_(pid),
        use_extended_id_(use_extended_id) {
    this->state_ = WAITING;
  };

  void setup() override;
  uint32_t last_polled_{0};

  bool start();
  bool update();

  void set_timeout(std::uint32_t timeout) { this->timeout_ = timeout; }
  void set_interval(std::uint32_t interval) { this->interval_ = interval; }
  void set_slow_interval(std::uint32_t slow_interval) { this->slow_interval_ = slow_interval; }
  void set_reply_length(std::uint32_t reply_length) { this->reply_length_ = reply_length; }
  void set_wake(bool wake) { this->wake_ = wake; }

  void add_sensor(OBDSensorBase *sensor) { this->sensors_.push_back(sensor); }
  void add_trigger(OBDPidTrigger *trigger) { this->triggers_.push_back(trigger); }
  void dump_config() override;

 protected:
  void handle_incoming(const std::vector<uint8_t> &data);
  void send_wake();
  void send_pid_request();

  OBDComponent *parent_;
  uint32_t can_id_;
  uint32_t can_response_id_;
  uint32_t pid_;
  bool use_extended_id_;
  uint32_t interval_{5000};
  uint32_t slow_interval_{0};
  uint32_t timeout_{500};
  uint32_t reply_length_{8};
  bool wake_{false};
  uint8_t wake_retries_{0};

  request_state_t state_{WAITING};
  std::vector<uint8_t> response_buffer_{};

  std::vector<OBDSensorBase *> sensors_{};
  std::vector<OBDPidTrigger *> triggers_{};
};

class OBDPidTrigger : public Trigger<std::vector<uint8_t>>, public Component {
 public:
  explicit OBDPidTrigger(PIDRequest *parent) : parent_(parent) {}

  void setup() override { this->parent_->add_trigger(this); }

 protected:
  PIDRequest *parent_;
};

class OBDSensorBase {
 public:
  virtual void update(const std::vector<uint8_t> &data) {}
};

class OBDSensor : public OBDSensorBase, public sensor::Sensor, public Component {
 public:
  explicit OBDSensor(PIDRequest *parent) : parent_(parent) {}

  void set_template(data_to_value_t &&lambda) { this->data_to_value_func_ = lambda; }

  void update(const std::vector<uint8_t> &data) override;
  void setup() override { this->parent_->add_sensor(this); }
  void dump_config() override;

 protected:
  PIDRequest *parent_;
  data_to_value_t data_to_value_func_{};
};

class OBDBinarySensor : public OBDSensorBase, public binary_sensor::BinarySensor, public Component {
 public:
  explicit OBDBinarySensor(PIDRequest *parent) : parent_(parent) {}

  void set_template(data_to_bool_t &&lambda) { this->data_to_value_func_ = lambda; }
  void update(const std::vector<uint8_t> &data) override;

  void setup() override { this->parent_->add_sensor(this); }

  void dump_config() override;

 protected:
  PIDRequest *parent_;
  data_to_bool_t data_to_value_func_;
};

class OBDCanbusTrigger : public Action<std::vector<uint8_t>, uint32_t, bool> {
 public:
  explicit OBDCanbusTrigger(PIDRequest *parent)
      : parent_(parent) {
    this->trigger_ =
        new canbus::CanbusTrigger(parent->parent_->canbus_, parent->can_response_id_, 0x1FFFFFFF, parent->use_extended_id_);
    auto automation = new Automation<std::vector<uint8_t>, uint32_t, bool>(this->trigger_);
    automation->add_action(this);
  };

  void setup() { this->trigger_->setup(); }

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const std::vector<uint8_t> &data, const uint32_t &can_id, const bool &rx) override
#else
  void play(std::vector<uint8_t> data, uint32_t can_id, bool rx) override
#endif
  {
    this->parent_->handle_incoming(data);
  }

 protected:
  PIDRequest *parent_;
  canbus::CanbusTrigger *trigger_;
};

template<typename... Ts> class SetPollingSpeedAction : public Action<Ts...> {
 public:
  explicit SetPollingSpeedAction(OBDComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(polling_speed_t, polling_speed)

  void play(Ts... x) override { this->parent_->set_polling_speed(this->polling_speed_.value(x...)); }

 protected:
  OBDComponent *parent_;
};

}  // namespace obd
}  // namespace esphome
