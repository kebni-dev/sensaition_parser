// Core types, constants, and helpers for the Kebni serial protocol.
// Sensor IDs and conversions are defined in kebni_sensor_ids.hpp (STANDARD_SENSORS macro).
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "kebni_driver/kebni_sensor_ids.hpp"

namespace kebni {

inline const uint8_t MESSAGE_START_BYTE = 0xFA;
inline const int MAX_HEX_VALUE = 0xFF;
inline const double G_TO_MS2 = 9.80665; // 1 g in m/s²
inline const double DEG_TO_RAD = std::numbers::pi / 180.0;
inline const char XOR8_SIGNATURE = 'x';
inline const char CRC16_SIGNATURE = 'X';
inline const size_t MAX_PAYLOAD_LENGTH = 82; // Firmware limit (§8.2)

// Generated from STANDARD_SENSORS, plus special-case sensors.
enum class SensorId : uint8_t {
#define SENSOR(name, val, _) name = val,
    STANDARD_SENSORS
#undef SENSOR
        // Special-case sensors (non-linear, packed, enum, or bitmask data)
        imu_temperature = 0x09,
    calibrated_imu_temperature = 0x0F,
    gnss_num_sat = 0x2B,
    error_flags = 0x2F,
    sensor_valid = 0x30,
    alignment_status = 0x40,
    attitude_status = 0x41,
    utc_year_month = 0x45,
    utc_day_time = 0x46,
    gnss_fix_type = 0x47,
    utc_status = 0x4B,
};

// Packet parser state machine.
enum class State { WAIT_SYNC, READ_PAYLOAD, READ_CHECKSUM, READ_CHECKSUM_LSB };

// 'x' = XOR8 (1 byte), 'X' = CRC16 (2 bytes, MSB first).
enum class ChecksumType { XOR8, CRC16 };

// Sensor 0x40: navigation alignment status.
enum class AlignmentStatus : uint8_t {
    WaitingForGnssFix = 0,
    NavigationRunning = 1,
};

// Sensor 0x47: GNSS fix type (packed x2, one per receiver).
enum class GnssFixType : uint8_t {
    NoFix = 0,
    Fix2D = 2,
    Fix3D = 3,
};

inline bool isGnssFixed(GnssFixType ft) { return ft == GnssFixType::Fix2D || ft == GnssFixType::Fix3D; }

// Sensor 0x4B: UTC time validity.
enum class UtcStatus : uint8_t {
    NoFix = 0,
    Available = 1,
    Valid = 2,
};

// Sensor 0x2F: 26-bit error register. Bit definitions from Table 39 (§5.6).
struct ErrorFlags {
    bool flash_checksum_error = false;    // Bit 0: Faulty flash, params restored to factory default.
    bool imu0_faulty = false;             // Bit 1: IMU0 hardware fault.
    bool imu1_faulty = false;             // Bit 2: IMU1 hardware fault.
    bool imu2_faulty = false;             // Bit 3: IMU2 hardware fault.
    bool imu3_faulty = false;             // Bit 4: IMU3 hardware fault.
    bool imu4_faulty = false;             // Bit 5: IMU4 hardware fault.
    bool imu5_faulty = false;             // Bit 6: IMU5 hardware fault.
    bool imu6_faulty = false;             // Bit 7: IMU6 hardware fault.
    bool imu7_faulty = false;             // Bit 8: IMU7 hardware fault.
    bool any_imu_faulty = false;          // Bit 9: Any IMU hardware fault.
    bool barometer_faulty = false;        // Bit 10: Barometer circuit faulty.
    bool magnetometer_error = false;      // Bit 11: Magnetometer circuit faulty.
    bool voltage_3v3_low = false;         // Bit 12: 3.3V low voltage.
    bool voltage_5v_low = false;          // Bit 13: 5V low voltage.
    bool max_imu_temperature = false;     // Bit 14: IMU max temperature reached.
    bool min_imu_temperature = false;     // Bit 15: IMU min temperature reached.
    bool sensor_fusion_runtime = false;   // Bit 16: Sensor fusion diverged and restarted.
    bool wmm_init = false;                // Bit 17: Couldn't start WMM algorithm.
    bool wmm_time = false;                // Bit 18: WMM model is outdated.
    bool sensor_fusion_data_mgmt = false; // Bit 19: Software fault.
    bool gnss_data_mgmt = false;          // Bit 20: Software fault.
    bool sensor_fusion_deadline = false;  // Bit 21: Processor overloaded.
    bool data_uart_oversaturated = false; // Bit 22: Data UART oversaturated.
    bool user_uart_oversaturated = false; // Bit 23: User UART oversaturated.
    bool can_bus_oversaturated = false;   // Bit 24: CAN bus oversaturated.
    bool spi_bus_oversaturated = false;   // Bit 25: SPI bus oversaturated.
};

// Sensor 0x30: sensor availability bitmask.
struct SensorValidFlags {
    bool imu = false;           // Bit 0: IMU available.
    bool magnetometer = false;  // Bit 1: Magnetometer available.
    bool barometer = false;     // Bit 2: Barometer available.
    bool odometer = false;      // Bit 3: Odometer available.
    bool gnss1 = false;         // Bit 4: GNSS1 available.
    bool gnss2 = false;         // Bit 5: GNSS2 available.
    bool moving_relpos = false; // Bit 6: Moving relative position available.
    bool fixed_relpos = false;  // Bit 7: Fixed relative position available.
};

// Configuration parsing errors.
enum class ConfigError {
    None,                        // No error.
    InputTooShort,               // Config string shorter than minimum length.
    InvalidStartCharacter,       // Missing 'o' (frequency divisor marker).
    MissingChecksumSignature,    // Config ended before checksum signature.
    InvalidFrequencyDivisor,     // oXXXX field is not valid hex.
    ZeroFrequencyDivisor,        // Frequency divisor is zero.
    InvalidPayloadLength,        // sYY field missing or invalid.
    InvalidPayloadChunkFormat,   // Payload chunk is not 3 hex characters.
    InvalidPayloadHexValue,      // Payload chunk has invalid hex.
    UnknownSensorId,             // Sensor ID not recognized.
    InvalidChecksumSignature,    // Not 'x' (XOR8) or 'X' (CRC16).
    PayloadLengthExceedsMaximum, // Exceeds 82-byte firmware limit (§8.2).
    InvalidByteIndex,            // Byte index out of range (valid: 0-3).
};

// Stream processing errors.
enum class StreamError { None, DriverNotConfigured, UnexpectedState };

// Hex ID string -> sensor name, used for config string validation.
inline const std::unordered_map<std::string, std::string> &sensorHexToNameMap() {
    static const std::unordered_map<std::string, std::string> map = {
#define SENSOR(name, val, _) {#val, #name},
        STANDARD_SENSORS
#undef SENSOR
        {"0x09", "imu_temperature"},
        {"0x0F", "calibrated_imu_temperature"},
        {"0x2B", "gnss_num_sat"},
        {"0x2F", "error_flags"},
        {"0x30", "sensor_valid"},
        {"0x40", "alignment_status"},
        {"0x41", "attitude_status"},
        {"0x45", "utc_year_month"},
        {"0x46", "utc_day_time"},
        {"0x47", "gnss_fix_type"},
        {"0x4B", "utc_status"},
    };

    return map;
}

// Describes where a sensor's bytes sit in the payload.
struct Field {
    SensorId sensorId;
    uint8_t offset;                 // Byte offset in the payload.
    std::vector<uint8_t> byteOrder; // Byte indices for value reconstruction.
};

// Parsed driver configuration: frequency, payload layout, checksum, and field mapping.
struct Configuration {
    ConfigError configError;
    bool configured = false;
    float freq;                      // Output frequency in Hz (1000 / freq_divisor).
    int freq_divisor;                // From the 'oXXXX' field.
    size_t payloadLength;            // Expected payload bytes per packet.
    std::vector<std::string> values; // Raw hex sensor tokens from config string.
    ChecksumType checksumType;
    char signature;                        // 'x' or 'X'.
    std::vector<Field> messageByteMapping; // How to extract each sensor from the payload.

    bool hasSensor(SensorId id) const {
        return std::any_of(
            messageByteMapping.begin(), messageByteMapping.end(), [id](const Field &f) { return f.sensorId == id; });
    }

    void print() const;
};

// Raw protocol packet: sync byte + payload + checksum.
struct Packet {
    uint8_t start;
    std::vector<uint8_t> payload;
    uint16_t checksum;
};

// Decoded sensor measurements from a single packet. Values are in SI units.
struct Measurements {
    bool invalidChecksum = false;
#define SENSOR(name, val, _) std::optional<double> name;
    STANDARD_SENSORS
#undef SENSOR

    // Special-case fields (non-linear conversion)
    std::optional<double> imu_temperature;
    std::optional<double> calibrated_imu_temperature;
    std::optional<double> gnss1_num_sat;
    std::optional<double> gnss2_num_sat;
    std::optional<double> utc_year;
    std::optional<double> utc_month;
    std::optional<double> utc_day;
    std::optional<double> utc_hour;
    std::optional<double> utc_min;
    std::optional<double> utc_sec;

    // Typed enum/bitmask fields (with raw values for publishing)
    std::optional<ErrorFlags> error_flags;
    std::optional<uint32_t> error_flags_raw;
    std::optional<SensorValidFlags> sensor_valid;
    std::optional<uint8_t> sensor_valid_raw;
    std::optional<AlignmentStatus> alignment_status;
    std::optional<uint32_t> attitude_status;
    std::optional<GnssFixType> gnss1_fix_type;
    std::optional<GnssFixType> gnss2_fix_type;
    std::optional<UtcStatus> utc_status;

    void add(SensorId id, int32_t raw) {
        // Special-case: non-linear and packed conversions
        switch (id) {
        case SensorId::imu_temperature:
            imu_temperature = (static_cast<double>(raw) / 10000.0) * 80.0 + 20.0;
            return;
        case SensorId::calibrated_imu_temperature:
            calibrated_imu_temperature = (static_cast<double>(raw) / 10000.0) * 80.0 + 20.0;
            return;
        case SensorId::gnss_num_sat:
            gnss1_num_sat = static_cast<double>(raw & 0xFFFF);
            gnss2_num_sat = static_cast<double>((raw >> 16) & 0xFFFF);
            return;
        case SensorId::utc_year_month:
            utc_month = static_cast<double>(raw & 0xFF);
            utc_year = static_cast<double>((raw >> 16) & 0xFFFF);
            return;
        case SensorId::utc_day_time:
            utc_sec = static_cast<double>(raw & 0xFF);
            utc_min = static_cast<double>((raw >> 8) & 0xFF);
            utc_hour = static_cast<double>((raw >> 16) & 0xFF);
            utc_day = static_cast<double>((raw >> 24) & 0xFF);
            return;
        case SensorId::gnss_fix_type:
            gnss1_fix_type = static_cast<GnssFixType>(raw & 0xFFFF);
            gnss2_fix_type = static_cast<GnssFixType>((raw >> 16) & 0xFFFF);
            return;
        case SensorId::error_flags:
            error_flags_raw = static_cast<uint32_t>(raw);
            error_flags = ErrorFlags{
                .flash_checksum_error = static_cast<bool>(raw & (1 << 0)),
                .imu0_faulty = static_cast<bool>(raw & (1 << 1)),
                .imu1_faulty = static_cast<bool>(raw & (1 << 2)),
                .imu2_faulty = static_cast<bool>(raw & (1 << 3)),
                .imu3_faulty = static_cast<bool>(raw & (1 << 4)),
                .imu4_faulty = static_cast<bool>(raw & (1 << 5)),
                .imu5_faulty = static_cast<bool>(raw & (1 << 6)),
                .imu6_faulty = static_cast<bool>(raw & (1 << 7)),
                .imu7_faulty = static_cast<bool>(raw & (1 << 8)),
                .any_imu_faulty = static_cast<bool>(raw & (1 << 9)),
                .barometer_faulty = static_cast<bool>(raw & (1 << 10)),
                .magnetometer_error = static_cast<bool>(raw & (1 << 11)),
                .voltage_3v3_low = static_cast<bool>(raw & (1 << 12)),
                .voltage_5v_low = static_cast<bool>(raw & (1 << 13)),
                .max_imu_temperature = static_cast<bool>(raw & (1 << 14)),
                .min_imu_temperature = static_cast<bool>(raw & (1 << 15)),
                .sensor_fusion_runtime = static_cast<bool>(raw & (1 << 16)),
                .wmm_init = static_cast<bool>(raw & (1 << 17)),
                .wmm_time = static_cast<bool>(raw & (1 << 18)),
                .sensor_fusion_data_mgmt = static_cast<bool>(raw & (1 << 19)),
                .gnss_data_mgmt = static_cast<bool>(raw & (1 << 20)),
                .sensor_fusion_deadline = static_cast<bool>(raw & (1 << 21)),
                .data_uart_oversaturated = static_cast<bool>(raw & (1 << 22)),
                .user_uart_oversaturated = static_cast<bool>(raw & (1 << 23)),
                .can_bus_oversaturated = static_cast<bool>(raw & (1 << 24)),
                .spi_bus_oversaturated = static_cast<bool>(raw & (1 << 25)),
            };
            return;
        case SensorId::sensor_valid:
            sensor_valid_raw = static_cast<uint8_t>(raw & 0xFF);
            sensor_valid = SensorValidFlags{
                .imu = static_cast<bool>(raw & (1 << 0)),
                .magnetometer = static_cast<bool>(raw & (1 << 1)),
                .barometer = static_cast<bool>(raw & (1 << 2)),
                .odometer = static_cast<bool>(raw & (1 << 3)),
                .gnss1 = static_cast<bool>(raw & (1 << 4)),
                .gnss2 = static_cast<bool>(raw & (1 << 5)),
                .moving_relpos = static_cast<bool>(raw & (1 << 6)),
                .fixed_relpos = static_cast<bool>(raw & (1 << 7)),
            };
            return;
        case SensorId::alignment_status:
            alignment_status = static_cast<AlignmentStatus>(raw & 0xFF);
            return;
        case SensorId::attitude_status:
            attitude_status = static_cast<uint32_t>(raw);
            return;
        case SensorId::utc_status:
            utc_status = static_cast<UtcStatus>(raw & 0xFF);
            return;
        default:
            break;
        }

        // Standard sensors: simple multiplication
        switch (id) {
#define SENSOR(name, _, conversion)                                                                                    \
    case SensorId::name:                                                                                               \
        name = static_cast<double>(raw) * conversion;                                                                  \
        break;
            STANDARD_SENSORS
#undef SENSOR
        default:
            break;
        }
    }

    void print() const {
        std::cout << "\nMeasurements:" << std::endl;

#define SENSOR(name, val, _)                                                                                           \
    if (name.has_value()) {                                                                                            \
        std::cout << #name << " = " << name.value() << std::endl;                                                      \
    }
        STANDARD_SENSORS
#undef SENSOR
        if (imu_temperature.has_value()) {
            std::cout << "imu_temperature = " << imu_temperature.value() << std::endl;
        }
        if (calibrated_imu_temperature.has_value())
            std::cout << "calibrated_imu_temperature = " << calibrated_imu_temperature.value() << std::endl;
        if (gnss1_num_sat.has_value())
            std::cout << "gnss1_num_sat = " << gnss1_num_sat.value() << std::endl;
        if (gnss2_num_sat.has_value())
            std::cout << "gnss2_num_sat = " << gnss2_num_sat.value() << std::endl;
        if (utc_year.has_value())
            std::cout << "utc_year = " << utc_year.value() << std::endl;
        if (utc_month.has_value())
            std::cout << "utc_month = " << utc_month.value() << std::endl;
        if (utc_day.has_value())
            std::cout << "utc_day = " << utc_day.value() << std::endl;
        if (utc_hour.has_value())
            std::cout << "utc_hour = " << utc_hour.value() << std::endl;
        if (utc_min.has_value())
            std::cout << "utc_min = " << utc_min.value() << std::endl;
        if (utc_sec.has_value())
            std::cout << "utc_sec = " << utc_sec.value() << std::endl;
        if (error_flags.has_value())
            std::cout << "error_flags = (present)" << std::endl;
        if (sensor_valid.has_value())
            std::cout << "sensor_valid = (present)" << std::endl;
        if (alignment_status.has_value())
            std::cout << "alignment_status = " << static_cast<int>(alignment_status.value()) << std::endl;
        if (attitude_status.has_value())
            std::cout << "attitude_status = " << attitude_status.value() << std::endl;
        if (gnss1_fix_type.has_value())
            std::cout << "gnss1_fix_type = " << static_cast<int>(gnss1_fix_type.value()) << std::endl;
        if (gnss2_fix_type.has_value())
            std::cout << "gnss2_fix_type = " << static_cast<int>(gnss2_fix_type.value()) << std::endl;
        if (utc_status.has_value())
            std::cout << "utc_status = " << static_cast<int>(utc_status.value()) << std::endl;
    }
};

// toString() overloads for enums.
inline const char *toString(SensorId id) {
    switch (id) {
#define SENSOR(name, val, _)                                                                                           \
    case SensorId::name:                                                                                               \
        return #name;
        STANDARD_SENSORS
#undef SENSOR
    case SensorId::imu_temperature:
        return "imu_temperature";
    case SensorId::calibrated_imu_temperature:
        return "calibrated_imu_temperature";
    case SensorId::gnss_num_sat:
        return "gnss_num_sat";
    case SensorId::error_flags:
        return "error_flags";
    case SensorId::sensor_valid:
        return "sensor_valid";
    case SensorId::alignment_status:
        return "alignment_status";
    case SensorId::attitude_status:
        return "attitude_status";
    case SensorId::utc_year_month:
        return "utc_year_month";
    case SensorId::utc_day_time:
        return "utc_day_time";
    case SensorId::gnss_fix_type:
        return "gnss_fix_type";
    case SensorId::utc_status:
        return "utc_status";
    default:
        return "UNKNOWN";
    }
}

inline const char *toString(ConfigError error) {
    switch (error) {
    case ConfigError::None:
        return "No error";

    case ConfigError::InputTooShort:
        return "Configuration string is too short";

    case ConfigError::InvalidStartCharacter:
        return "Invalid start character (expected 'o')";

    case ConfigError::MissingChecksumSignature:
        return "Missing checksum signature";

    case ConfigError::InvalidFrequencyDivisor:
        return "Invalid frequency divisor";

    case ConfigError::ZeroFrequencyDivisor:
        return "Frequency divisor cannot be zero";

    case ConfigError::InvalidPayloadLength:
        return "Invalid payload length";

    case ConfigError::InvalidPayloadChunkFormat:
        return "Invalid payload chunk format";

    case ConfigError::InvalidPayloadHexValue:
        return "Invalid payload hex value";

    case ConfigError::UnknownSensorId:
        return "Unknown sensor ID";

    case ConfigError::InvalidChecksumSignature:
        return "Invalid checksum signature (expected 'x' or 'X')";

    case ConfigError::PayloadLengthExceedsMaximum:
        return "Payload length exceeds maximum of 82 bytes";

    case ConfigError::InvalidByteIndex:
        return "Byte index in field chunk is out of range (valid: 0-3)";
    default:
        return "Unknown configuration error";
    }
}

inline const char *toString(StreamError error) {
    switch (error) {
    case StreamError::None:
        return "No stream error";

    case StreamError::DriverNotConfigured:
        return "Driver is not configured";

    case StreamError::UnexpectedState:
        return "Unexpected parser state";

    default:
        return "Unknown stream error";
    }
}

inline const char *toString(State state) {
    switch (state) {
    case State::WAIT_SYNC:
        return "WAIT_SYNC";
    case State::READ_PAYLOAD:
        return "READ_PAYLOAD";
    case State::READ_CHECKSUM:
        return "READ_CHECKSUM";
    case State::READ_CHECKSUM_LSB:
        return "READ_CHECKSUM_LSB";
    default:
        return "UNKNOWN_STATE";
    }
}

} // namespace kebni