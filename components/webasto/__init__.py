import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor, switch, text_sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_HEATER,
    CONF_TEMPERATURE,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BATTERY,
    ICON_HEATING_COIL,
    ICON_THERMOMETER,
    UNIT_CELSIUS,
    UNIT_VOLT,
)

CODEOWNERS = ["@mvirkkunen"]
MULTI_CONF = True
AUTO_LOAD = ["binary_sensor", "sensor", "switch", "text_sensor"]

webasto_ns = cg.esphome_ns.namespace("webasto")
WebastoSwitch = webasto_ns.class_("WebastoSwitch", switch.Switch)
WebastoComponent = webasto_ns.class_("WebastoComponent", cg.Component, uart.UARTDevice)

CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_CONNECTED = "connected"
CONF_FAULTS = "faults"
CONF_DIAGNOSTICS = "diagnostics"

ICON_ALERT = "mdi:alert"
ICON_CONNECTION = "mdi:connection"
ICON_DIAGNOSTICS = "mdi:clipboard-pulse"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(WebastoComponent),
            cv.Optional(CONF_UPDATE_INTERVAL, default="1500ms"): cv.update_interval,
            cv.Optional(CONF_HEATER): switch.switch_schema(
                icon=ICON_HEATING_COIL,
            ).extend({
                cv.GenerateID(): cv.declare_id(WebastoSwitch)
            }),
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                icon=ICON_CONNECTION,
                device_class=DEVICE_CLASS_CONNECTIVITY,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
            ),
            cv.Optional(CONF_SUPPLY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_BATTERY,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_BATTERY,
            ),
            cv.Optional(CONF_FAULTS): text_sensor.text_sensor_schema(
                icon=ICON_ALERT,
            ),
            cv.Optional(CONF_DIAGNOSTICS): text_sensor.text_sensor_schema(
                icon=ICON_DIAGNOSTICS,
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

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_SUPPLY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE])
        cg.add(var.set_supply_voltage_sensor(sens))

    if CONF_FAULTS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAULTS])
        cg.add(var.set_faults_sensor(sens))

    if CONF_DIAGNOSTICS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DIAGNOSTICS])
        cg.add(var.set_diagnostics_sensor(sens))
