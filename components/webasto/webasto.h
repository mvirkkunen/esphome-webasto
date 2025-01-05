#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace webasto {

using Bytes = std::vector<uint8_t>;

struct WebastoFault {
    unsigned major_code;
    unsigned minor_code;
    unsigned counter;
    unsigned operating_state;
    int temperature;
};

struct WebastoState {
    unsigned operating_state = 0;
    int temperature = 0;
    float flame_detector = 0.0f;
    float supply_voltage = 0.0f;
    float combustion_air_fan = 0.0f;
    int glow_plug = 0;
    float fuel_pump = 0.0f;
    bool combustion_air_fan_on = false;
    bool glow_plug_on = false;
    bool circulating_pump_on = false;
    bool fuel_pump_on = false;
    bool vehicle_fan_relay_on = false;
    bool main_switch_on = false;
    bool supplemental_heater_request = false;
    bool summer = false;
    bool flame_detected = false;
    std::vector<WebastoFault> faults;
};

class WebastoComponent;

class WebastoSwitch : public switch_::Switch {
public:
    void write_state(bool state) override;
    void set_parent(WebastoComponent* parent) { this->parent = parent; }

private:
    WebastoComponent* parent;
};

class WebastoComponent : public Component, public uart::UARTDevice {
public:
    void setup() override;
    void loop() override;
    void set_update_interval(uint32_t interval) { update_interval = interval; }
    void set_connected_sensor(binary_sensor::BinarySensor* sensor) { connected_sensor = sensor; }
    void set_temperature_sensor(sensor::Sensor* sensor) { temperature_sensor = sensor; }
    void set_supply_voltage_sensor(sensor::Sensor* sensor) { supply_voltage_sensor = sensor; }
    void set_faults_sensor(text_sensor::TextSensor* sensor) { faults_sensor = sensor; }
    void set_diagnostics_sensor(text_sensor::TextSensor* sensor) { diagnostics_sensor = sensor; }
    void set_heater_switch(WebastoSwitch* switch_) { heater_switch = switch_; heater_switch->set_parent(this); }
    void write_switch_state(bool state);

private:
    enum class CommState;

    void set_break(bool break_);
    uint32_t comm_state_enter();
    CommState comm_state_complete(const Bytes& reply);
    void update_sensors();
    void send_command(const Bytes& command);

    HighFrequencyLoopRequester high_freq_loop;
    CommState comm_state;
    WebastoState state;
    Bytes buffer;
    uint32_t state_entry = 0;
    uint32_t state_timeout = 0;
    unsigned init_attempts = 0;
    bool heater_on = false;
    bool heater_sync = false;
    uint32_t update_interval = 1500;
    binary_sensor::BinarySensor *connected_sensor = nullptr;
    sensor::Sensor *temperature_sensor = nullptr;
    sensor::Sensor *supply_voltage_sensor = nullptr;
    text_sensor::TextSensor *faults_sensor = nullptr;
    text_sensor::TextSensor *diagnostics_sensor = nullptr;
    WebastoSwitch *heater_switch = nullptr;
};

}
}
