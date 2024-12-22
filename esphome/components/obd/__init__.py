import esphome.config_validation as cv
import esphome.codegen as cg
import esphome.components.sensor as s
import esphome.components.binary_sensor as bs
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_INTERVAL,
    CONF_TRIGGER_ID
)

obd_ns = cg.esphome_ns.namespace("obd")
OBDComponent = obd_ns.class_("OBDComponent", cg.PollingComponent)
OBDSensor = obd_ns.class_("OBDSensor", s.Sensor, cg.Component)
OBDBinarySensor = obd_ns.class_("OBDBinarySensor", bs.BinarySensor, cg.Component)
PIDRequest = obd_ns.class_("PIDRequest", cg.Component)
OBDPidTrigger = obd_ns.class_(
    "OBDPidTrigger", 
    automation.Trigger.template(cg.std_vector.template(cg.uint8)),
    cg.Component
)

CONF_CANBUS_ID = 'canbus_id'
CONF_ENABLED_BY_DEFAULT = 'enabled_by_default'
CONF_OBD_ID = 'obd_id'
CONF_PID_ID = "pid_id"
CONF_CAN_ID = "can_id"
CONF_RESPONSE_CAN_ID = "response_can_id"
CONF_USE_EXTENDED_ID = "use_extended_id"
CONF_PIDS = "pids"
CONF_PID = "pid"
CONF_TIMEOUT = "timeout"
CONF_REPLY_LENGTH = "reply_length"
CONF_ON_FRAME = "on_frame"
CONF_MASK = "mask"
CONF_SIGNED = "signed"

CONFIG_SCHEMA = cv.Schema({
  cv.GenerateID(): cv.declare_id(OBDComponent),
  cv.Required(CONF_CANBUS_ID): cv.use_id("CanbusComponent"),
  cv.Optional(CONF_ENABLED_BY_DEFAULT, default=False): cv.boolean,
  cv.Optional(CONF_PIDS): cv.ensure_list(
      {
          cv.GenerateID(CONF_ID): cv.declare_id(PIDRequest),
          cv.Required(CONF_CAN_ID): cv.int_range(min=0, max=0x1FFFFFFF),
          cv.Required(CONF_PID): cv.int_range(min=0, max=0x1FFFFFFF),
          cv.Optional(CONF_RESPONSE_CAN_ID): cv.int_range(min=0, max=0x1FFFFFFF),
          cv.Optional(CONF_USE_EXTENDED_ID, default=False): cv.boolean,
          cv.Optional(CONF_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
          cv.Optional(CONF_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
          cv.Optional(CONF_REPLY_LENGTH, default=8): cv.positive_int,
          cv.Optional(CONF_ON_FRAME): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OBDPidTrigger),
            }
          ),
      }
  )
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    can_bus = await cg.get_variable(config[CONF_CANBUS_ID])

    cg.add(var.set_canbus(can_bus))
    cg.add(var.set_enabled_by_default(config[CONF_ENABLED_BY_DEFAULT]))

    for pid_conf in config.get(CONF_PIDS, []):
        can_id = pid_conf[CONF_CAN_ID]
        pid = pid_conf[CONF_PID]
        use_extended_id = pid_conf[CONF_USE_EXTENDED_ID]
        response_can_id = pid_conf.get(CONF_RESPONSE_CAN_ID)
        
        if (response_can_id is None):
            response_can_id = can_id | 8

        pid_request = cg.new_Pvariable(
            pid_conf[CONF_ID], var, can_id, pid, response_can_id, use_extended_id
        )
        await cg.register_component(pid_request, pid_conf)

        cg.add(pid_request.set_interval(pid_conf[CONF_INTERVAL]))
        cg.add(pid_request.set_timeout(pid_conf[CONF_TIMEOUT]))
        cg.add(pid_request.set_reply_length(pid_conf[CONF_REPLY_LENGTH]))

        for trigger_conf in pid_conf.get(CONF_ON_FRAME, []):
            trigger = cg.new_Pvariable(trigger_conf[CONF_TRIGGER_ID], pid_request)
            await cg.register_component(trigger, trigger_conf)
            await automation.build_automation(trigger, [(cg.std_vector.template(cg.uint8), "data")], trigger_conf)

    return var
