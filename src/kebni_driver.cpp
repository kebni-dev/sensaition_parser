#include "kebni_driver/kebni_driver.hpp"

#include <boost/asio.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace kebni {

// Feeds one byte into the packet state machine (sync -> payload -> checksum).
StreamError KebniDriver::processByte(uint8_t byte) {

    if (!configuration.configured) {
        return StreamError::DriverNotConfigured;
    }

    switch (state) {

    case State::WAIT_SYNC:
        // 0xFA marks the start of a new packet
        if (byte == MESSAGE_START_BYTE) {
            packet = {};
            packet.start = byte;
            packet.payload.clear();
            state = State::READ_PAYLOAD;
        }
        break;

    case State::READ_PAYLOAD:
        packet.payload.push_back(byte);
        if (packet.payload.size() == configuration.payloadLength) {
            state = State::READ_CHECKSUM;
        }
        break;

    case State::READ_CHECKSUM:
        if (configuration.checksumType == ChecksumType::XOR8) {
            // XOR8: single byte checksum, packet complete
            packet.checksum = byte;
            state = State::WAIT_SYNC;
            handlePacket(packet);
            break;
        } else {
            // CRC16: this is the MSB, still need LSB
            packet.checksum = static_cast<uint8_t>(byte) << 8;
            state = State::READ_CHECKSUM_LSB;
            break;
        }
    case State::READ_CHECKSUM_LSB:
        // CRC16 LSB, packet complete
        packet.checksum |= static_cast<uint8_t>(byte);
        state = State::WAIT_SYNC;
        handlePacket(packet);
        break;

    default:
        state = State::WAIT_SYNC;
        return StreamError::UnexpectedState;
    }

    return StreamError::None;
}

// Parses the configuration string and sets up the driver.
// Returns the resulting Configuration (check configError for failures).
Configuration KebniDriver::configureKebniDriver(const std::string &config) {

    // Parse the string into frequency, payload length, sensor tokens, checksum type
    Configuration parsedConfiguration = parseConfiguration(config);
    if (parsedConfiguration.configError != ConfigError::None) {
        return parsedConfiguration;
    }

    // Convert sensor tokens into field descriptors (offset + byte order)
    std::vector<Field> fieldsOpt = buildFieldMap(parsedConfiguration);

    if (parsedConfiguration.configError != ConfigError::None) {
        return parsedConfiguration;
    }

    // Verify that the total field bytes match the declared payload length
    size_t totalFieldBytes = 0;
    for (const auto &f : fieldsOpt) {
        totalFieldBytes += f.byteOrder.size();
    }

    if (totalFieldBytes != parsedConfiguration.payloadLength) {
        parsedConfiguration.configError = ConfigError::InvalidPayloadLength;
        return parsedConfiguration;
    }

    parsedConfiguration.messageByteMapping = std::move(fieldsOpt);
    parsedConfiguration.configured = true;

    configuration = std::move(parsedConfiguration);
    return configuration;
}

// Validates checksum, decodes payload, and calls onMeasurements().
void KebniDriver::handlePacket(const Packet &packet) {

    Measurements measurements;

    // Verify packet integrity before decoding
    switch (configuration.checksumType) {
    case ChecksumType::XOR8: {
        uint8_t calc = 0;
        for (uint8_t b : packet.payload) {
            calc ^= b;
        }
        if (calc != static_cast<uint8_t>(packet.checksum)) {
            measurements.invalidChecksum = true;
            onMeasurements(measurements);
            return;
        }
        break;
    }
    case ChecksumType::CRC16: {
        uint16_t calc = crc16(packet.payload.data(), packet.payload.size());
        if (calc != packet.checksum) {
            measurements.invalidChecksum = true;
            onMeasurements(measurements);
            return;
        }
        break;
    }
    default:
        measurements.invalidChecksum = true;
        onMeasurements(measurements);
        return;
    }

    // Decode each field from the payload using its byte mapping and apply SI conversion
    for (const Field &f : configuration.messageByteMapping) {
        int32_t raw = readValue(packet.payload, f);
        measurements.add(f.sensorId, raw);
    }

    onMeasurements(measurements);
}

Configuration KebniDriver::parseConfiguration(const std::string &input) const {
    Configuration configuration;
    configuration.configError = ConfigError::None;
    size_t i = 0;

    if (input.size() < 9) {
        configuration.configError = ConfigError::InputTooShort;
        return configuration;
    }

    // Frequency divisor: 'oXXXX' (4 hex digits, output_hz = 1000 / divisor)
    if (input[i] != 'o') {
        configuration.configError = ConfigError::InvalidStartCharacter;
        return configuration;
    }

    try {
        configuration.freq_divisor = std::stoi(input.substr(i + 1, 4), nullptr, 16);
    } catch (...) {
        configuration.configError = ConfigError::InvalidFrequencyDivisor;
        return configuration;
    }

    if (configuration.freq_divisor == 0) {
        configuration.configError = ConfigError::ZeroFrequencyDivisor;
        return configuration;
    }
    configuration.freq = 1000.0 / configuration.freq_divisor;
    i += 5;

    // Payload length: 'sYY' (2 hex digits, number of bytes per packet)
    if (input[i] != 's') {
        configuration.configError = ConfigError::InvalidPayloadLength;
        return configuration;
    }
    try {
        configuration.payloadLength = static_cast<size_t>(std::stoi(input.substr(i + 1, 2), nullptr, 16));
    } catch (...) {
        configuration.configError = ConfigError::InvalidPayloadLength;
        return configuration;
    }
    i += 3;

    if (configuration.payloadLength == 0) {
        configuration.configError = ConfigError::InvalidPayloadLength;
        return configuration;
    }

    if (configuration.payloadLength > MAX_PAYLOAD_LENGTH) {
        configuration.configError = ConfigError::PayloadLengthExceedsMaximum;
        return configuration;
    }

    // Sensor field chunks: 3 hex chars each (2 for sensor ID, 1 for byte index)
    while (i + 2 < input.size() && std::isxdigit(input[i])) {
        configuration.values.push_back(input.substr(i, 3));
        i += 3;
    }

    // Checksum signature: 'x' for XOR8, 'X' for CRC16
    if (i >= input.size()) {
        configuration.configError = ConfigError::MissingChecksumSignature;
        return configuration;
    }

    configuration.signature = input[i];

    switch (configuration.signature) {
    case XOR8_SIGNATURE:
        configuration.checksumType = ChecksumType::XOR8;
        break;
    case CRC16_SIGNATURE:
        configuration.checksumType = ChecksumType::CRC16;
        break;
    default:
        configuration.configError = ConfigError::InvalidChecksumSignature;
        return configuration;
    }

    return configuration;
}

// Groups config chunks by sensor ID and builds the byte-to-field mapping.
// Each chunk is 3 hex chars: first 2 = sensor ID, last 1 = byte index (0-3).
// Consecutive chunks with the same sensor ID are grouped into a single Field
// that describes the full byte layout for that sensor in the payload.
std::vector<Field> KebniDriver::buildFieldMap(Configuration &configuration) const {

    const auto &sensorMap = sensorHexToNameMap();
    std::vector<Field> fields;

    Field currentField{};
    bool hasActiveField = false;

    std::vector<uint8_t> absoluteBytes;
    uint8_t currentStartFieldOffset = 0;

    for (const std::string &chunk : configuration.values) {

        // Validate chunk format: must be exactly 3 hex characters
        if (chunk.size() != 3 || !std::isxdigit(chunk[0]) || !std::isxdigit(chunk[1]) || !std::isxdigit(chunk[2])) {
            std::cerr << "Invalid payload chunk: " << chunk << "\n";
            configuration.configError = ConfigError::InvalidPayloadChunkFormat;
            return fields;
        }

        // First 2 chars = sensor hex ID (e.g. "00" -> accX, "03" -> gyroX)
        std::string hexKey = "0x" + chunk.substr(0, 2);
        int hexValue;

        try {
            hexValue = std::stoi(hexKey, nullptr, 16);
        } catch (...) {
            std::cerr << "Invalid sensor hex: " << hexKey << "\n";
            configuration.configError = ConfigError::InvalidPayloadHexValue;
            return fields;
        }

        auto it = sensorMap.find(hexKey);
        if (it == sensorMap.end()) {
            std::cerr << "Unknown sensor hex key: " << hexKey << "\n";
            configuration.configError = ConfigError::UnknownSensorId;
            return fields;
        }

        SensorId sensorId = static_cast<SensorId>(hexValue);

        uint8_t byteIndex = static_cast<uint8_t>(std::stoi(chunk.substr(2, 1), nullptr, 16));

        if (byteIndex > 3) {
            configuration.configError = ConfigError::InvalidByteIndex;
            return fields;
        }

        // New sensor ID means we finalize the previous field and start a new one
        if (!hasActiveField || currentField.sensorId != sensorId) {

            if (hasActiveField) {
                currentField.offset = currentStartFieldOffset;
                currentField.byteOrder = absoluteBytes;
                fields.push_back(std::move(currentField));
                currentStartFieldOffset += absoluteBytes.size();
            }

            currentField = Field{};
            currentField.sensorId = sensorId;
            absoluteBytes.clear();
            hasActiveField = true;
        }

        absoluteBytes.push_back(byteIndex);
    }

    // Adding last field
    if (hasActiveField) {
        currentField.offset = currentStartFieldOffset;
        currentField.byteOrder = absoluteBytes;
        fields.push_back(std::move(currentField));
    }

    return fields;
}

// CRC16 Checksum
uint16_t KebniDriver::crc16(const uint8_t *data, size_t length) const {
    const uint16_t poly = 0x1021;
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= *(data + i) << 8;
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Reconstructs a raw int32 from payload bytes using the field's byte order.
int32_t KebniDriver::readValue(const std::vector<uint8_t> &payload, const Field &field) const {
    uint32_t value = 0;
    const size_t n = field.byteOrder.size();

    if (field.offset + n > payload.size()) {
        return 0;
    }

    for (size_t i = 0; i < n; ++i) {
        uint8_t shiftBytes = field.byteOrder[i];  // target byte position in the int32
        uint8_t byte = payload[field.offset + i]; // actual byte from the payload

        value |= static_cast<uint32_t>(byte) << (shiftBytes * 8);
    }

    return static_cast<int32_t>(value);
}

// Debug print of the full driver configuration.
void Configuration::print() const {
    std::cout << "\nKebni Driver Configuration:" << std::endl;

    std::cout << "Configured: " << (configured ? "yes" : "no") << std::endl;
    std::cout << "Config error: " << toString(configError) << std::endl;

    std::cout << "Frequency divisor: " << freq_divisor << std::endl;
    std::cout << "Frequency: " << freq << " Hz\n";
    std::cout << "Payload length: " << payloadLength << " bytes\n";
    std::cout << "Checksum type: " << (checksumType == ChecksumType::XOR8 ? "XOR8" : "CRC16") << " ('" << signature
              << "')\n";

    std::cout << "\nMessage Byte Mapping:" << std::endl;
    for (size_t i = 0; i < messageByteMapping.size(); ++i) {
        const Field &f = messageByteMapping[i];

        std::cout << "Field " << i << ":\n";
        std::cout << "  Sensor: " << toString(f.sensorId) << std::endl;
        std::cout << "  Offset: " << static_cast<int>(f.offset) << std::endl;
        std::cout << "  Byte order: ";

        for (uint8_t b : f.byteOrder) {
            std::cout << static_cast<int>(b) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;
}

} // namespace kebni