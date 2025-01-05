#include "webasto.h"

#include <sstream>
#include <unordered_map>

#if defined(USE_ESP32_FRAMEWORK_ARDUINO)
#include "esphome/components/uart/uart_component_esp32_arduino.h"
#elif defined(USE_HOST)
// Simulated break
#else
#error This platform is not currently supported
#endif

namespace esphome {
namespace webasto {

extern const std::unordered_map<int, const char*> STATE_NAMES;

#define TAG "webasto"

const Bytes CMD_START_COMM =   { 0x81, 0x51, 0xf1, 0x81, 0x44 };
const Bytes CMD_READ_STATE_1 = { 0x83, 0x51, 0xf1, 0x2a, 0x01, 0x01, 0xf1 };
const Bytes CMD_READ_STATE_2 = { 0x83, 0x51, 0xf1, 0x2a, 0x01, 0x02, 0xf2 };
const Bytes CMD_READ_FAULTS =  { 0x81, 0x51, 0xf1, 0xa1, 0x64 };
const Bytes CMD_TURN_ON =      { 0x83, 0x51, 0xf1, 0x31, 0x22, 0xff, 0x17 };
const Bytes CMD_TURN_OFF =     { 0x83, 0x51, 0xf1, 0x31, 0x22, 0x00, 0x18 };
const Bytes CMD_KEEP_ON =      { 0x83, 0x51, 0xf1, 0x31, 0x22, 0x01, 0x19 };

uint32_t CMD_TIMEOUT = 500;

enum class WebastoComponent::CommState {
    START = 0,
    INIT_START_SYNC,  // 3000ms
    INIT_BREAK_1, // 300ms
    INIT_SLEEP_1, // 50ms
    INIT_BREAK_2, // 25ms
    INIT_SLEEP_2, // 100ms
    INIT_START_COMM,
    INIT_START_WAIT,
    READ_STATE_1,
    READ_STATE_2,
    READ_FAULTS,
    MORE_FAULTS,
    KEEP_ON,
    SET_HEATER,
    POLL_SLEEP,
};

void WebastoComponent::setup() {
    heater_sync = true;
    comm_state = CommState::START;
}

void WebastoComponent::loop() {
    // Check for state timeout
    size_t now = millis();
    if (now - state_entry >= state_timeout) {
        comm_state = comm_state_complete({});
        state_timeout = comm_state_enter();
        state_entry = now;
        return;
    }

    while (available()) {
        int byte = read();
        if (byte < 0) {
            break;
        }

        buffer.push_back(byte);

        // Is header byte available?
        if (buffer.empty()) {
            continue;
        }

        // Is entire reply available?
        size_t len = (buffer[0] & 0x0f) + 4;
        if (buffer.size() < len) {
            continue;
        }

        uint8_t checksum = 0;
        for (unsigned i = 0; i < len - 1; i++) {
            checksum += buffer[i];
        }

        if (checksum != buffer[buffer.size() - 1]) {
            ESP_LOGI(TAG, "invalid checksum");
            buffer.clear();
            break;
        }

        if (buffer[1] == 0x51) {
            // Echoed message due to half duplex, ignore
            buffer.clear();
            continue;
        }

        comm_state = comm_state_complete(buffer);
        state_timeout = comm_state_enter();
        state_entry = now;
        return;
    }
}

uint32_t WebastoComponent::comm_state_enter() {
    switch (comm_state) {
        case CommState::INIT_START_SYNC:
            ESP_LOGD(TAG, "attempting to connect...");
            init_attempts = 0;
            return 3000;

        case CommState::INIT_BREAK_1:
            high_freq_loop.start();
            set_break(true);
            return 300;

        case CommState::INIT_SLEEP_1:
            set_break(false);
            return 50;

        case CommState::INIT_BREAK_2:
            set_break(true);
            return 25;

        case CommState::INIT_SLEEP_2:
            set_break(false);
            high_freq_loop.stop();
            return 3000;

        case CommState::INIT_START_COMM:
            send_command(CMD_START_COMM);
            return CMD_TIMEOUT;

        case CommState::READ_STATE_1:
            send_command(CMD_READ_STATE_1);
            return CMD_TIMEOUT;

        case CommState::READ_STATE_2:
            send_command(CMD_READ_STATE_2);
            return CMD_TIMEOUT;

        case CommState::READ_FAULTS:
            send_command(CMD_READ_FAULTS);
            return CMD_TIMEOUT;

        case CommState::MORE_FAULTS: return CMD_TIMEOUT;

        case CommState::SET_HEATER:
            send_command(heater_on ? CMD_TURN_ON : CMD_TURN_OFF);
            return CMD_TIMEOUT;

        case CommState::KEEP_ON:
            send_command(CMD_KEEP_ON);
            return CMD_TIMEOUT;

        case CommState::POLL_SLEEP: return heater_on ? 1500 : update_interval;
    }

    ESP_LOGE(TAG, "unknown state entered: %d", comm_state);
    return 0;
}

WebastoComponent::CommState WebastoComponent::comm_state_complete(const Bytes& reply) {
    static std::vector<WebastoFault> faults;

    switch (comm_state) {
        case CommState::START: return CommState::INIT_START_SYNC;
        case CommState::INIT_START_SYNC: return CommState::INIT_BREAK_1;
        case CommState::INIT_BREAK_1: return CommState::INIT_SLEEP_1;
        case CommState::INIT_SLEEP_1: return CommState::INIT_BREAK_2;
        case CommState::INIT_BREAK_2: return CommState::INIT_SLEEP_2;
        case CommState::INIT_SLEEP_2: return CommState::INIT_START_COMM;
        case CommState::INIT_START_COMM:
            if (reply.size() < 7) {
                ESP_LOGE(TAG, "invalid reply length for INIT_START_COMM: %d", reply.size());
                init_attempts++;
                return (init_attempts > 3) ? CommState::INIT_START_SYNC : CommState::INIT_START_COMM;
            }

            return CommState::READ_STATE_1;

        case CommState::READ_STATE_1:
            if (reply.size() < 11) {
                ESP_LOGE(TAG, "invalid reply length for READ_STATE_1: %d", reply.size());
                return CommState::INIT_START_SYNC;
            }

            state.temperature = static_cast<int>(reply[5]) - 40;
            state.flame_detector = 204.8f / (204.8f - static_cast<float>(reply[6])) - 1.0f;
            state.supply_voltage = static_cast<float>(reply[7]) * 0.068275;
            state.operating_state = reply[9];
            return CommState::READ_STATE_2;

        case CommState::READ_STATE_2:
            if (reply.size() < 12) {
                ESP_LOGE(TAG, "invalid reply length for READ_STATE_2: %d", reply.size());
                return CommState::INIT_START_SYNC;
            }

            state.combustion_air_fan = static_cast<float>(reply[5]) / 255.0f;
            state.glow_plug = static_cast<float>(reply[6]);
            state.fuel_pump = 1.0f / static_cast<float>(reply[8]);

            state.combustion_air_fan_on = reply[9] & 0x01;
            state.glow_plug_on = reply[9] & 0x02;
            state.circulating_pump_on = reply[9] & 0x04;
            state.fuel_pump_on = reply[9] & 0x08;
            state.vehicle_fan_relay_on = reply[9] & 0x10;

            state.summer = reply[10] & 0x01;
            state.main_switch_on = reply[10] & 0x04;
            state.supplemental_heater_request = reply[10] & 0x08;
            state.flame_detected = reply[10] & 0x20;

            return CommState::READ_FAULTS;

        case CommState::READ_FAULTS:
            if (reply.size() < 8) {
                ESP_LOGE(TAG, "invalid reply length for READ_FAULTS: %d", reply.size());
                return CommState::INIT_START_SYNC;
            }

            if (reply[4] != 0xff || reply[5] != 0xff) {
                if (reply.size() < 12) {
                    ESP_LOGE(TAG, "invalid reply length for READ_FAULTS: %d", reply.size());
                    return CommState::MORE_FAULTS;
                }

                faults.push_back({
                    static_cast<unsigned>((static_cast<uint16_t>(reply[4]) << 8) | reply[5]),
                    reply[7],
                    reply[8],
                    reply[9],
                    static_cast<int>(reply[10]) - 40,
                });

                return CommState::MORE_FAULTS;
            }

            state.faults = faults;
            faults.clear();
            return CommState::POLL_SLEEP;

        case CommState::SET_HEATER:
            if (reply.size() < 7) {
                ESP_LOGE(TAG, "invalid reply length for SET_HEATER: %d", reply.size());
                return CommState::INIT_START_SYNC;
            }

            if (heater_switch) {
                heater_switch->publish_state(heater_on);
            }

            heater_sync = true;

            return CommState::READ_STATE_1;

        case CommState::KEEP_ON:
            if (reply.size() < 7) {
                ESP_LOGE(TAG, "invalid reply length for KEEP_ON: %d", reply.size());
                return CommState::INIT_START_SYNC;
            }

            return CommState::READ_STATE_1;

        case CommState::POLL_SLEEP:
            update_sensors();
            return !heater_sync
                ? CommState::SET_HEATER
                : heater_on
                ? CommState::KEEP_ON
                : CommState::READ_STATE_1;
    }

    ESP_LOGE(TAG, "unknown state completed: %d", comm_state);
    return CommState::INIT_START_SYNC;
}

void WebastoComponent::update_sensors() {
    if (temperature_sensor && temperature_sensor->get_state() != state.temperature) {
        temperature_sensor->publish_state(state.temperature);
    }

    if (battery_voltage_sensor && battery_voltage_sensor->get_state() != state.supply_voltage) {
        battery_voltage_sensor->publish_state(state.supply_voltage);
    }

    if (faults_sensor) {
        std::ostringstream ss;

        int index = 1;
        for (auto& fault : state.faults) {
            ss
                << index << ". "
                << fault.major_code << "/" << fault.minor_code
                << " (counter: " << fault.counter
                << ", state: " << fault.operating_state
                << ", temperature: " << fault.temperature << "°C)\r\n";
            index++;
        }

        std::string state = ss.str();
        if (state != faults_sensor->get_state()) {
            faults_sensor->publish_state(state);
        }
    }

    if (diagnostics_sensor) {
        std::ostringstream ss;
        ss << "Operating state: " << state.operating_state << " - ";

        auto it = STATE_NAMES.find(state.operating_state);
        if (it != STATE_NAMES.end()) {
            ss << it->second;
        } else {
            ss << "Unknown state";
        }

        ss << "\r\n";

        ss << "Heater on: " << (state.is_on ? "yes" : "no") << "\r\n";
        ss << "Temperature: " << state.temperature << "°C\r\n";

        std::string state = ss.str();
        if (state != diagnostics_sensor->get_state()) {
            diagnostics_sensor->publish_state(state);
        }
    }
}

void WebastoComponent::send_command(const Bytes& cmd) {
    while (available()) {
        read();
    }

    buffer.clear();
    write_array(cmd);
}

void WebastoComponent::write_switch_state(bool state) {
    heater_on = state;
    heater_sync = false;

    if (comm_state == CommState::POLL_SLEEP) {
        state_timeout = 0;
    }
}

void WebastoComponent::set_break(bool break_) {
#if defined(USE_ESP32_FRAMEWORK_ARDUINO)
    auto uart = static_cast<esphome::uart::ESP32ArduinoUARTComponent*>(parent_);
    uart_set_line_inverse(uart->get_hw_serial_number(), break_ ? UART_SIGNAL_TXD_INV : UART_SIGNAL_INV_DISABLE);
#elif defined(USE_HOST)
    ESP_LOGI(TAG, "Simulating serial break %s", break_ ? "ON" : "OFF");
#endif
}

void WebastoSwitch::write_state(bool state) {
    parent->write_switch_state(state);
}

const std::unordered_map<int, const char*> STATE_NAMES = {
    { 0, "Off state" },
    { 1, "Flame detector interrogation 1" },
    { 2, "Glowing 1" },
    { 3, "Glowing 11" },
    { 4, "Prestart fuel supply 1" },
    { 5, "Fuel supply 11" },
    { 6, "Fuel supply 12" },
    { 7, "Fuel supply 13" },
    { 8, "Fuel supply 15" },
    { 9, "Stabilization time" },
    { 10, "Fuel supply 14" },
    { 11, "Flame detector measuring phase 1" },
    { 12, "Glow plug ramp 1" },
    { 16, "Flame detector interrogation 2" },
    { 17, "Glow plug ramp 2" },
    { 18, "Glow plug ramp 3" },
    { 21, "Combustion process full load" },
    { 22, "Load change FL-PL 1" },
    { 23, "Load change FL-PL 2" },
    { 24, "Combustion process part load" },
    { 25, "Load change PL-FL 1" },
    { 26, "Load change PL-FL 2" },
    { 27, "Burn out 31" },
    { 28, "Burn out 32" },
    { 29, "Glowing 21" },
    { 30, "Fuel supply 25" },
    { 32, "Burn out 22" },
    { 33, "Burn out 23" },
    { 34, "Burn out 24" },
    { 35, "Control idle period 1" },
    { 36, "Control idle period 2" },
    { 37, "Glowing 4" },
    { 38, "Glowing 5" },
    { 39, "Glowing 6" },
    { 40, "Prestart fuel supply 2" },
    { 41, "Fuel supply 42" },
    { 42, "Fuel supply 33" },
    { 43, "Flame detector measuring phase 2" },
    { 44, "Fuel supply 34" },
    { 45, "Flame detector measuring phase 3" },
    { 48, "Ventilating" },
    { 49, "Fan idle" },
    { 50, "Cooling 2" },
    { 51, "Interlock" },
    { 52, "Heater interlock" },
    { 53, "Burn out 11" },
    { 54, "Burn out 12" },
    { 55, "Burn out 13" },
    { 56, "Burn out 14" },
    { 57, "Burn out 41" },
    { 58, "Burn out 42" },
    { 59, "Burn out 51" },
    { 60, "Burn out 61" },
    { 61, "Burn out 71" },
    { 63, "Cooling 1" },
    { 64, "Deactivation" },
    { 65, "Heater interlock deactivation" },
    { 66, "Interlock save to memory" },
    { 67, "Flame detector interrogation 3" },
    { 68, "Cooling 3" },
    { 69, "Cooling 4" },
    { 70, "Glowing 10" },
    { 71, "Idle time save to memory" },
    { 72, "Burn out 25" },
    { 73, "Burn out 15" },
    { 74, "Burn out 16" },
    { 75, "Flame detector interrogation 4" },
    { 76, "Prestart fuel supply 4" },
    { 77, "Prestart fuel supply 5" },
    { 78, "Prestart fuel supply 3" },
    { 80, "Off state" },
    { 81, "Flame detector interrogation 1" },
    { 82, "Glowing 1" },
    { 83, "Prestart fuel supply 1" },
    { 84, "Fuel supply 11" },
    { 85, "Flame detector measuring phase 1" },
    { 86, "Glowing 2" },
    { 87, "Glowing 3" },
    { 88, "Fuel supply 21" },
    { 89, "Glowing 10" },
    { 90, "Glowing 11" },
    { 91, "Flame detector interrogation 2" },
    { 92, "Glow plug ramp 1" },
    { 93, "Glow plug ramp 2" },
    { 96, "Flame detector measuring phase 2" },
    { 97, "Combustion process full load" },
    { 98, "Load change FL-PL 1" },
    { 99, "Load change FL-PL 2" },
    { 100, "Combustion process part load" },
    { 101, "Load change PL-FL 1" },
    { 102, "Load change PL-FL 2" },
    { 103, "Burn out 32" },
    { 104, "Burn out 22" },
    { 105, "Burn out 23" },
    { 112, "Burn out 24" },
    { 113, "Cooling 3" },
    { 114, "Control idle period 2" },
    { 115, "Cooling 4" },
    { 116, "Glowing 4" },
    { 117, "Fuel supply 33" },
    { 118, "Flame detector measuring phase 3" },
    { 119, "Ventilating" },
    { 120, "Cooling 2" },
    { 121, "Interlock" },
    { 122, "Glow plug ramp 3" },
    { 123, "Glow plug ramp 4" },
    { 128, "Heater interlock" },
    { 129, "Burn out 11" },
    { 130, "Burn out 12" },
    { 131, "Burn out 13" },
    { 132, "Cooling 1" },
    { 133, "Deactivation" },
    { 134, "Heater interlock deactivation" },
    { 135, "Interlock save to memory" },
    { 136, "Idle time save to memory" },
    { 137, "Burn out 25" },
    { 138, "Burn out 15" },
    { 139, "Flame detector interrogation 3" },
    { 140, "Flame detector interrogation 4" },
    { 141, "Glowing 5" },
    { 142, "Fuel supply 42" },
    { 143, "Flame detector measuring phase 4" },
};

}
}
