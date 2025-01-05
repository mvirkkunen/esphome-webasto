#include "webasto.h"
#include "webasto_tables.h"

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

#define TAG "webasto"

void appendFaults(std::ostringstream& ss, const std::vector<WebastoFault>& faults);

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

    if (faults_sensor) {
        faults_sensor->publish_state("");
    }
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

            if (connected_sensor) {
                connected_sensor->publish_state(false);
            }

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

            state.temperature = static_cast<int>(TEMPERATURE_CURVE[reply[5]]);
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
            state.glow_plug = reply[6];
            state.fuel_pump = reply[8] == 0 ? 0.0f : (1.0f / static_cast<float>(reply[8]));

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
        case CommState::MORE_FAULTS:
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
                    static_cast<int>(TEMPERATURE_CURVE[reply[10]]),
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
    if (connected_sensor) {
        connected_sensor->publish_state(true);
    }

    if (temperature_sensor && temperature_sensor->get_state() != state.temperature) {
        temperature_sensor->publish_state(state.temperature);
    }

    if (supply_voltage_sensor && supply_voltage_sensor->get_state() != state.supply_voltage) {
        supply_voltage_sensor->publish_state(state.supply_voltage);
    }

    if (faults_sensor) {
        std::ostringstream ss;
        appendFaults(ss, state.faults);

        std::string state = ss.str();
        if (state != faults_sensor->get_state()) {
            faults_sensor->publish_state(state);
        }
    }

    if (diagnostics_sensor) {
        std::ostringstream ss;

        ss << "Operating state: " << state.operating_state << " - ";

        auto it = OPERATING_STATE_NAME.find(state.operating_state);
        if (it != OPERATING_STATE_NAME.end()) {
            ss << it->second;
        } else {
            ss << "Unknown state";
        }

        ss << "\n";

        ss << "Supply voltage: " << state.supply_voltage << " V\n";
        ss << "Temperature: " << state.temperature << " °C\n";
        ss << "Combustion air fan: " << static_cast<int>(state.combustion_air_fan * 100.0f) << " %\n";
        ss << "Fuel pump: " << state.fuel_pump << " Hz\n";
        ss << "Glow plug: " << state.glow_plug << " %\n";
        ss << "Flame detector: " << state.flame_detector << " Ω\n";

        if (state.combustion_air_fan_on) ss << "[CAF] ";
        if (state.fuel_pump_on) ss << "[FP] ";
        if (state.glow_plug_on) ss << "[GP] ";
        if (state.circulating_pump_on) ss << "[CP] ";
        if (state.vehicle_fan_relay_on) ss << "[VFR] ";
        if (state.main_switch_on) ss << "[MS] ";
        if (state.supplemental_heater_request) ss << "[SHR] ";
        if (state.summer) ss << "[S] ";
        if (state.flame_detected) ss << "[FD] ";

        ss << "\n";

        if (state.faults.empty()) {
            ss << "No faults.\n";
        } else {
            ss << state.faults.size() << "fault(s):\n";
            appendFaults(ss, state.faults);
        }

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

    if (heater_switch) {
        heater_switch->publish_state(heater_on);
    }

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

void appendFaults(std::ostringstream& ss, const std::vector<WebastoFault>& faults) {
    int index = 1;
    for (auto& fault : faults) {
        ss
            << index << ". "
            << fault.major_code << "/" << fault.minor_code
            << " (counter: " << fault.counter
            << ", state: " << fault.operating_state
            << ", temperature: " << fault.temperature << "°C)\n";
        index++;
    }
}

}
}
