import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, switch, text_sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_HEATER,
    CONF_TEMPERATURE,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BATTERY,
    ICON_CHIP,
    ICON_HEART_PULSE,
    ICON_HEATING_COIL,
    ICON_RADIOACTIVE,
    ICON_THERMOMETER,
    UNIT_CELSIUS,
    UNIT_VOLT,
)

CODEOWNERS = ["@mvirkkunen"]
MULTI_CONF = True
AUTO_LOAD = ["sensor", "switch", "text_sensor"]

webasto_ns = cg.esphome_ns.namespace("webasto")
#WebastoSensor = webasto_ns.class_("WebastoSensor", sensor.Sensor)
WebastoSwitch = webasto_ns.class_("WebastoSwitch", switch.Switch)
WebastoComponent = webasto_ns.class_("WebastoComponent", cg.Component, uart.UARTDevice)

cv.polling_component_schema

CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_FAULTS = "faults"
CONF_DIAGNOSTICS = "diagnostics"

cv.polling_component_schema

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(WebastoComponent),
            cv.Optional(CONF_UPDATE_INTERVAL, default="10s"): cv.update_interval,
            cv.Optional(CONF_HEATER): switch.switch_schema(
                icon=ICON_HEATING_COIL,
            ).extend({
                cv.GenerateID(): cv.declare_id(WebastoSwitch)
            }),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_BATTERY,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_BATTERY,
            ),
            cv.Optional(CONF_FAULTS): text_sensor.text_sensor_schema(
                icon=ICON_RADIOACTIVE,
            ),
            cv.Optional(CONF_DIAGNOSTICS): text_sensor.text_sensor_schema(
                icon=ICON_HEART_PULSE,
            )
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    if CONF_HEATER in config:
        sw = await switch.new_switch(config[CONF_HEATER])
        cg.add(var.set_heater_switch(sw))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))

    if CONF_FAULTS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAULTS])
        cg.add(var.set_faults_sensor(sens))

    if CONF_DIAGNOSTICS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DIAGNOSTICS])
        cg.add(var.set_diagnostics_sensor(sens))
