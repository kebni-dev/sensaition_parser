#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "kebni_definitions.hpp"

namespace kebni {

class KebniDriver {
  public:
    virtual ~KebniDriver() = default;

    virtual void onMeasurements(const Measurements &measurements) = 0;
    Configuration configureKebniDriver(const std::string &config);
    StreamError processByte(uint8_t byte);

  private:
    State state = State::WAIT_SYNC;
    Configuration configuration;
    Packet packet;

    int32_t readValue(const std::vector<uint8_t> &payload, const Field &field) const;
    std::vector<Field> buildFieldMap(Configuration &configuration) const;
    Configuration parseConfiguration(const std::string &input) const;
    uint16_t crc16(const uint8_t *data, size_t length) const;
    void handlePacket(const Packet &packet);
};

} // namespace kebni