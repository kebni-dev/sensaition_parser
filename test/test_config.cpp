#include <gtest/gtest.h>
#include "kebni_driver/kebni_definitions.hpp"
#include "kebni_driver/kebni_driver.hpp"

class KebniDriverStub : public kebni::KebniDriver {
  public:
    void onMeasurements(const kebni::Measurements &) override {}
};

// Minimum-length guard

TEST(ConfigTest, InputTooShort_EmptyString) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InputTooShort);
}

TEST(ConfigTest, InputTooShort_EightChars) {
    KebniDriverStub driver;
    // 8 chars — one short of the 9-char minimum
    auto cfg = driver.configureKebniDriver("o0001s0x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InputTooShort);
}

// Valid single-field string (12 chars) must NOT be rejected as too short
TEST(ConfigTest, SingleFieldString_NotTooShort) {
    KebniDriverStub driver;
    // o0001 s01 000 x  → 1 byte payload, accX byte 0, XOR8
    auto cfg = driver.configureKebniDriver("o0001s01000x");
    EXPECT_NE(cfg.configError, kebni::ConfigError::InputTooShort);
}

// Payload length validation

TEST(ConfigTest, ZeroPayloadLength_Rejected) {
    KebniDriverStub driver;
    // o0001 s00 x  → payload length 0
    auto cfg = driver.configureKebniDriver("o0001s00x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidPayloadLength);
}

TEST(ConfigTest, PayloadLengthExceedsMaximum) {
    KebniDriverStub driver;
    // 0x53 = 83 > MAX_PAYLOAD_LENGTH (82)
    auto cfg = driver.configureKebniDriver("o0001s53000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::PayloadLengthExceedsMaximum);
}

TEST(ConfigTest, PayloadLengthAtMaximum_Accepted) {
    KebniDriverStub driver;
    // 0x52 = 82 == MAX_PAYLOAD_LENGTH — must NOT trigger the max-length error
    auto cfg = driver.configureKebniDriver("o0001s52000x");
    EXPECT_NE(cfg.configError, kebni::ConfigError::PayloadLengthExceedsMaximum);
}

// Byte index validation

TEST(ConfigTest, InvalidByteIndex_Four) {
    KebniDriverStub driver;
    // o0001 s01 004 x  → byte index 4 is out of range
    auto cfg = driver.configureKebniDriver("o0001s01004x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidByteIndex);
}

TEST(ConfigTest, InvalidByteIndex_F) {
    KebniDriverStub driver;
    // byte index F (15) — far out of range
    auto cfg = driver.configureKebniDriver("o0001s0100Fx");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidByteIndex);
}

TEST(ConfigTest, ValidByteIndex_Three) {
    KebniDriverStub driver;
    // byte index 3 — valid upper bound, full Int32 with indices 3,2,1,0
    auto cfg = driver.configureKebniDriver("o0001s04003002001000x");
    EXPECT_NE(cfg.configError, kebni::ConfigError::InvalidByteIndex);
}

// Remaining error paths

TEST(ConfigTest, InvalidStartCharacter) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("x0001s04003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidStartCharacter);
}

TEST(ConfigTest, InvalidFrequencyDivisor_NonHex) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("oZZZZs04003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidFrequencyDivisor);
}

TEST(ConfigTest, ZeroFrequencyDivisor) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0000s04003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::ZeroFrequencyDivisor);
}

TEST(ConfigTest, InvalidPayloadLength_MissingS) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0001p04003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidPayloadLength);
}

TEST(ConfigTest, InvalidPayloadLength_NonHex) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0001sZZ003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidPayloadLength);
}

TEST(ConfigTest, MissingChecksumSignature) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0001s04003002001000");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::MissingChecksumSignature);
}

TEST(ConfigTest, InvalidChecksumSignature) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0001s04003002001000z");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidChecksumSignature);
}

TEST(ConfigTest, InvalidPayloadChunkFormat) {
    KebniDriverStub driver;
    // chunk "0A!" — first char is xdigit so the loop admits it, but third char '!' is not hex
    auto cfg = driver.configureKebniDriver("o0001s030A!x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidPayloadChunkFormat);
}

TEST(ConfigTest, UnknownSensorId) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o0001s01FF0x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::UnknownSensorId);
}

TEST(ConfigTest, PayloadLengthMismatch) {
    KebniDriverStub driver;
    // s08 claims 8 bytes, but chunks only encode 4 bytes (one Int32)
    auto cfg = driver.configureKebniDriver("o0001s08003002001000x");
    EXPECT_EQ(cfg.configError, kebni::ConfigError::InvalidPayloadLength);
}

// Valid configurations

TEST(ConfigTest, SingleSensor_XOR8_CorrectFields) {
    KebniDriverStub driver;
    // 100 Hz, 4-byte accX (bytes 3,2,1,0), XOR8
    auto cfg = driver.configureKebniDriver("o000As04003002001000x");
    ASSERT_EQ(cfg.configError, kebni::ConfigError::None);
    EXPECT_TRUE(cfg.configured);
    EXPECT_NEAR(cfg.freq, 100.0f, 0.01f);
    EXPECT_EQ(cfg.payloadLength, 4u);
    ASSERT_EQ(cfg.messageByteMapping.size(), 1u);
    EXPECT_EQ(cfg.messageByteMapping[0].sensorId, kebni::SensorId::accX);
    EXPECT_EQ(cfg.messageByteMapping[0].offset, 0u);
    ASSERT_EQ(cfg.messageByteMapping[0].byteOrder.size(), 4u);
    EXPECT_EQ(cfg.messageByteMapping[0].byteOrder[0], 3u);
    EXPECT_EQ(cfg.messageByteMapping[0].byteOrder[1], 2u);
    EXPECT_EQ(cfg.messageByteMapping[0].byteOrder[2], 1u);
    EXPECT_EQ(cfg.messageByteMapping[0].byteOrder[3], 0u);
    EXPECT_EQ(cfg.checksumType, kebni::ChecksumType::XOR8);
}

TEST(ConfigTest, SingleSensor_CRC16_CorrectChecksumType) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o000As04003002001000X");
    ASSERT_EQ(cfg.configError, kebni::ConfigError::None);
    EXPECT_TRUE(cfg.configured);
    EXPECT_EQ(cfg.checksumType, kebni::ChecksumType::CRC16);
}

TEST(ConfigTest, MultiSensor_CorrectOffsets) {
    KebniDriverStub driver;
    // 500 Hz, accX (bytes 3,2,1,0) then gyroX (bytes 3,2,1,0), 8 bytes, XOR8
    auto cfg = driver.configureKebniDriver("o0002s08003002001000033032031030x");
    ASSERT_EQ(cfg.configError, kebni::ConfigError::None);
    EXPECT_TRUE(cfg.configured);
    EXPECT_EQ(cfg.payloadLength, 8u);
    ASSERT_EQ(cfg.messageByteMapping.size(), 2u);
    EXPECT_EQ(cfg.messageByteMapping[0].sensorId, kebni::SensorId::accX);
    EXPECT_EQ(cfg.messageByteMapping[0].offset, 0u);
    EXPECT_EQ(cfg.messageByteMapping[1].sensorId, kebni::SensorId::gyroX);
    EXPECT_EQ(cfg.messageByteMapping[1].offset, 4u);
}

TEST(ConfigTest, FrequencyDivisor_000A_ParsedAs10) {
    KebniDriverStub driver;
    auto cfg = driver.configureKebniDriver("o000As04003002001000x");
    ASSERT_EQ(cfg.configError, kebni::ConfigError::None);
    EXPECT_EQ(cfg.freq_divisor, 10);
}

TEST(ConfigTest, DefaultConfigString_Parses) {
    KebniDriverStub driver;
    // 100 Hz, 36 bytes, AccX-Z + GyroX-Z + InclX-Z (9 sensors × 4 bytes)
    const std::string def = "o000As24"
                            "003002001000"
                            "013012011010"
                            "023022021020"
                            "033032031030"
                            "043042041040"
                            "053052051050"
                            "063062061060"
                            "073072071070"
                            "083082081080"
                            "x";
    auto cfg = driver.configureKebniDriver(def);
    ASSERT_EQ(cfg.configError, kebni::ConfigError::None);
    EXPECT_TRUE(cfg.configured);
    EXPECT_EQ(cfg.payloadLength, 36u);
    ASSERT_EQ(cfg.messageByteMapping.size(), 9u);
    EXPECT_EQ(cfg.messageByteMapping[0].sensorId, kebni::SensorId::accX);
    EXPECT_EQ(cfg.messageByteMapping[1].sensorId, kebni::SensorId::accY);
    EXPECT_EQ(cfg.messageByteMapping[2].sensorId, kebni::SensorId::accZ);
    EXPECT_EQ(cfg.messageByteMapping[3].sensorId, kebni::SensorId::gyroX);
    EXPECT_EQ(cfg.messageByteMapping[4].sensorId, kebni::SensorId::gyroY);
    EXPECT_EQ(cfg.messageByteMapping[5].sensorId, kebni::SensorId::gyroZ);
    EXPECT_EQ(cfg.messageByteMapping[6].sensorId, kebni::SensorId::inclX);
    EXPECT_EQ(cfg.messageByteMapping[7].sensorId, kebni::SensorId::inclY);
    EXPECT_EQ(cfg.messageByteMapping[8].sensorId, kebni::SensorId::inclZ);
}
