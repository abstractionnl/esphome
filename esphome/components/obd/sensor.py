import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_INDEX, CONF_LAMBDA
from esphome.cpp_generator import LambdaExpression

from . import CONF_PID_ID, CONF_SIGNED, OBDSensor, PIDRequest

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        OBDSensor,
    ).extend(
        {
            cv.Required(CONF_PID_ID): cv.use_id(PIDRequest),
            cv.Optional(CONF_SIGNED, default=False): cv.boolean,
            cv.Exclusive(CONF_LAMBDA, CONF_LAMBDA): cv.returning_lambda,
            cv.Exclusive(CONF_INDEX, CONF_LAMBDA): cv.ensure_list(cv.positive_int),
        }
    ),
    cv.has_exactly_one_key(CONF_LAMBDA, CONF_INDEX),
)


async def to_code(config):
    pid_request = await cg.get_variable(config[CONF_PID_ID])
    var = await sensor.new_sensor(config, pid_request)
    await cg.register_component(var, config)

    template = False

    if lambda_ := config.get(CONF_LAMBDA):
        template = await cg.process_lambda(
            lambda_, [(cg.std_vector.template(cg.uint8), "data")], return_type=cg.float_
        )
        cg.add(var.set_template(template))
    else:
        indexes = config[CONF_INDEX]
        signed = "(int8_t)" if config[CONF_SIGNED] else ""

        if len(indexes) == 4:
            template = LambdaExpression(
                f"return ({signed}data[{indexes[0]}] << 24) | (data[{indexes[1]}] << 16) | (data[{indexes[2]}] << 8) | data[{indexes[3]}];",
                [(cg.std_vector.template(cg.uint8), "data")],
                return_type=cg.float_,
            )

        if len(indexes) == 3:
            template = LambdaExpression(
                f"return ({signed}data[{indexes[0]}] << 16) | (data[{indexes[1]}] << 8) | data[{indexes[2]}];",
                [(cg.std_vector.template(cg.uint8), "data")],
                return_type=cg.float_,
            )

        if len(indexes) == 2:
            template = LambdaExpression(
                f"return ({signed}data[{indexes[0]}] << 8) | data[{indexes[1]}];",
                [(cg.std_vector.template(cg.uint8), "data")],
                return_type=cg.float_,
            )

        if len(indexes) == 1:
            template = LambdaExpression(
                f"return {signed}data[{indexes[0]}];",
                [(cg.std_vector.template(cg.uint8), "data")],
                return_type=cg.float_,
            )

        cg.add(var.set_template(template))
