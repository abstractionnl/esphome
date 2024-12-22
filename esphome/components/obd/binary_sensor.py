import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.cpp_generator import LambdaExpression
from esphome.const import (
    CONF_LAMBDA,
    CONF_INDEX,
)
from . import (
    CONF_PID_ID,
    CONF_MASK,
    OBDBinarySensor,
    PIDRequest
)

CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(
        OBDBinarySensor,
    )
    .extend(
        {
            cv.Required(CONF_PID_ID): cv.use_id(PIDRequest),
            cv.Optional(CONF_LAMBDA): cv.returning_lambda,
            cv.Optional(CONF_INDEX): cv.positive_int,
            cv.Optional(CONF_MASK): cv.hex_int,
        }
    ),
    cv.has_exactly_one_key(CONF_LAMBDA, CONF_INDEX),
    cv.has_none_or_all_keys(CONF_INDEX, CONF_MASK)
)

async def to_code(config):
    pid_request = await cg.get_variable(config[CONF_PID_ID])
    var = await binary_sensor.new_binary_sensor(config, pid_request)
    await cg.register_component(var, config)

    if lambda_ := config.get(CONF_LAMBDA):
        template = await cg.process_lambda(lambda_, [(cg.std_vector.template(cg.uint8), "data")], return_type=cg.bool_)
        cg.add(var.set_template(template))
    
    else:
        index = config[CONF_INDEX]
        mask = config[CONF_MASK]
        template = LambdaExpression(
                f"return (data[{index}] & {mask}) == {mask};",
                [(cg.std_vector.template(cg.uint8), "data")], 
                return_type=cg.bool_
            )
        cg.add(var.set_template(template))