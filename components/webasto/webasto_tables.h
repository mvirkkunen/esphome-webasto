#include <array>
#include <cstdint>
#include <unordered_map>

namespace esphome {
namespace webasto {

extern const std::array<int16_t, 256> TEMPERATURE_CURVE;
extern const std::unordered_map<int, const char*> OPERATING_STATE_NAME;

}
}
