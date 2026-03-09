#include <gtest/gtest.h>
#include "kebni_driver/kebni_definitions.hpp"
#include "kebni_driver/kebni_driver.hpp"

using namespace kebni;

class TestDriver : public kebni::KebniDriver {
  public:
    kebni::Measurements lastMeasurements;
    bool received = false;

    void onMeasurements(const kebni::Measurements &m) override {
        lastMeasurements = m;
        received = true;
    }
};

TEST(MeasurementsTest, AccelerometerXConversion) {
    Measurements m;
    // 1,000,000 µg = 1 g = 9.80665 m/s²
    m.add(SensorId::accX, 1000000);
    ASSERT_TRUE(m.accX.has_value());
    EXPECT_NEAR(m.accX.value(), 9.80665, 1e-4);
}

TEST(MeasurementsTest, GyroXConversion) {
    Measurements m;
    // 1,000,000 µdeg/s = 1 deg/s = pi/180 rad/s
    m.add(SensorId::gyroX, 1000000);
    ASSERT_TRUE(m.gyroX.has_value());
    EXPECT_NEAR(m.gyroX.value(), std::numbers::pi / 180.0, 1e-6);
}

TEST(MeasurementsTest, InclinometerConversion) {
    Measurements m;
    m.add(SensorId::inclX, 1000000);
    ASSERT_TRUE(m.inclX.has_value());
    EXPECT_NEAR(m.inclX.value(), 9.80665, 1e-4);
}

TEST(MeasurementsTest, MagnetometerConversion) {
    Measurements m;
    m.add(SensorId::magX, 1000); // 1000 mGauss = 1 Gauss = 1e-4 Tesla
    ASSERT_TRUE(m.magX.has_value());
    EXPECT_NEAR(m.magX.value(), 1e-4, 1e-10);
}

TEST(MeasurementsTest, RollConversionToRadians) {
    Measurements m;
    m.add(SensorId::roll, 90000000); // 90 deg = pi/2 rad
    ASSERT_TRUE(m.roll.has_value());
    EXPECT_NEAR(m.roll.value(), std::numbers::pi / 2.0, 1e-4);
}

TEST(MeasurementsTest, PitchConversionToRadians) {
    Measurements m;
    m.add(SensorId::pitch, 180000000); // 180 deg = pi rad
    ASSERT_TRUE(m.pitch.has_value());
    EXPECT_NEAR(m.pitch.value(), std::numbers::pi, 1e-4);
}

TEST(MeasurementsTest, HeadingConversionToRadians) {
    Measurements m;
    m.add(SensorId::heading, 360000000); // 360 deg = 2*pi rad
    ASSERT_TRUE(m.heading.has_value());
    EXPECT_NEAR(m.heading.value(), 2.0 * std::numbers::pi, 1e-4);
}

TEST(MeasurementsTest, ImuTemperatureConversion) {
    Measurements m;
    // Formula: (raw / 10000.0) * 80.0 + 20.0
    // raw=5000 -> 60°C
    m.add(SensorId::imu_temperature, 5000);
    ASSERT_TRUE(m.imu_temperature.has_value());
    EXPECT_NEAR(m.imu_temperature.value(), 60.0, 1e-6);
}

TEST(MeasurementsTest, BarometerConversion) {
    Measurements m;
    m.add(SensorId::barometer, 1013250);
    ASSERT_TRUE(m.barometer.has_value());
    EXPECT_NEAR(m.barometer.value(), 101325.0, 1e-1);
}

TEST(MeasurementsTest, OdometerSpeedConversion) {
    Measurements m;
    m.add(SensorId::odometer_speed, 1500);
    ASSERT_TRUE(m.odometer_speed.has_value());
    EXPECT_NEAR(m.odometer_speed.value(), 1.5, 1e-6);
}

TEST(MeasurementsTest, VerticalPositionConversion) {
    Measurements m;
    m.add(SensorId::vertical_position, 100000);
    ASSERT_TRUE(m.vertical_position.has_value());
    EXPECT_NEAR(m.vertical_position.value(), 100.0, 1e-6);
}

TEST(MeasurementsTest, CorrectedAccConversion) {
    Measurements m;
    m.add(SensorId::corrected_acc_x, 1000000);
    ASSERT_TRUE(m.corrected_acc_x.has_value());
    EXPECT_NEAR(m.corrected_acc_x.value(), 9.80665, 1e-4);
}

TEST(MeasurementsTest, CorrectedGyroConversion) {
    Measurements m;
    m.add(SensorId::corrected_gyro_x, 1000000);
    ASSERT_TRUE(m.corrected_gyro_x.has_value());
    EXPECT_NEAR(m.corrected_gyro_x.value(), std::numbers::pi / 180.0, 1e-6);
}

TEST(MeasurementsTest, RotationMatrixConversion) {
    Measurements m;
    m.add(SensorId::rotation_matrix_11, 1000000);
    ASSERT_TRUE(m.rotation_matrix_11.has_value());
    EXPECT_NEAR(m.rotation_matrix_11.value(), 1.0, 1e-6);
}

TEST(MeasurementsTest, SystemTimeMsConversion) {
    Measurements m;
    m.add(SensorId::system_time_ms, 5000);
    ASSERT_TRUE(m.system_time_ms.has_value());
    EXPECT_NEAR(m.system_time_ms.value(), 5000.0, 1e-6);
}

TEST(MeasurementsTest, SystemTimeUsConversion) {
    Measurements m;
    m.add(SensorId::system_time_us, 5000000);
    ASSERT_TRUE(m.system_time_us.has_value());
    EXPECT_NEAR(m.system_time_us.value(), 5000000.0, 1e-6);
}

TEST(MeasurementsTest, QualityRollConversion) {
    Measurements m;
    m.add(SensorId::quality_roll, 1000000);
    ASSERT_TRUE(m.quality_roll.has_value());
    EXPECT_NEAR(m.quality_roll.value(), std::numbers::pi / 180.0, 1e-6);
}

TEST(MeasurementsTest, GnssFixedRelposConversion) {
    Measurements m;
    m.add(SensorId::gnss_fixed_relpos_north, 2500);
    ASSERT_TRUE(m.gnss_fixed_relpos_north.has_value());
    EXPECT_NEAR(m.gnss_fixed_relpos_north.value(), 2.5, 1e-6);
}

TEST(MeasurementsTest, ErrorFlagsParsing) {
    Measurements m;
    // Set bits: flash_checksum(0), barometer(10), magnetometer(11)
    int32_t raw = (1 << 0) | (1 << 10) | (1 << 11);
    m.add(SensorId::error_flags, raw);
    ASSERT_TRUE(m.error_flags.has_value());
    EXPECT_TRUE(m.error_flags->flash_checksum_error);
    EXPECT_FALSE(m.error_flags->imu0_faulty);
    EXPECT_FALSE(m.error_flags->imu1_faulty);
    EXPECT_FALSE(m.error_flags->any_imu_faulty);
    EXPECT_TRUE(m.error_flags->barometer_faulty);
    EXPECT_TRUE(m.error_flags->magnetometer_error);
    EXPECT_FALSE(m.error_flags->voltage_3v3_low);
    EXPECT_FALSE(m.error_flags->spi_bus_oversaturated);
}

TEST(MeasurementsTest, ErrorFlagsAllClear) {
    Measurements m;
    m.add(SensorId::error_flags, 0);
    ASSERT_TRUE(m.error_flags.has_value());
    EXPECT_FALSE(m.error_flags->flash_checksum_error);
    EXPECT_FALSE(m.error_flags->any_imu_faulty);
    EXPECT_FALSE(m.error_flags->barometer_faulty);
    EXPECT_FALSE(m.error_flags->spi_bus_oversaturated);
}

TEST(MeasurementsTest, SensorValidFlagsParsing) {
    Measurements m;
    // IMU(0) + barometer(2) + GNSS1(4)
    int32_t raw = (1 << 0) | (1 << 2) | (1 << 4);
    m.add(SensorId::sensor_valid, raw);
    ASSERT_TRUE(m.sensor_valid.has_value());
    EXPECT_TRUE(m.sensor_valid->imu);
    EXPECT_FALSE(m.sensor_valid->magnetometer);
    EXPECT_TRUE(m.sensor_valid->barometer);
    EXPECT_FALSE(m.sensor_valid->odometer);
    EXPECT_TRUE(m.sensor_valid->gnss1);
    EXPECT_FALSE(m.sensor_valid->gnss2);
    EXPECT_FALSE(m.sensor_valid->moving_relpos);
    EXPECT_FALSE(m.sensor_valid->fixed_relpos);
}

TEST(MeasurementsTest, AlignmentStatusEnum) {
    Measurements m;
    m.add(SensorId::alignment_status, 0);
    ASSERT_TRUE(m.alignment_status.has_value());
    EXPECT_EQ(m.alignment_status.value(), AlignmentStatus::WaitingForGnssFix);

    m.add(SensorId::alignment_status, 1);
    EXPECT_EQ(m.alignment_status.value(), AlignmentStatus::NavigationRunning);
}

TEST(MeasurementsTest, AttitudeStatusRawPassthrough) {
    Measurements m;
    m.add(SensorId::attitude_status, 0xABCD);
    ASSERT_TRUE(m.attitude_status.has_value());
    EXPECT_EQ(m.attitude_status.value(), 0xABCDu);
}

TEST(MeasurementsTest, UtcStatusEnum) {
    Measurements m;
    m.add(SensorId::utc_status, 0);
    ASSERT_TRUE(m.utc_status.has_value());
    EXPECT_EQ(m.utc_status.value(), UtcStatus::NoFix);

    m.add(SensorId::utc_status, 1);
    EXPECT_EQ(m.utc_status.value(), UtcStatus::Available);

    m.add(SensorId::utc_status, 2);
    EXPECT_EQ(m.utc_status.value(), UtcStatus::Valid);
}

TEST(MeasurementsTest, SyncInCountPassthrough) {
    Measurements m;
    m.add(SensorId::sync_in_count, 42);
    ASSERT_TRUE(m.sync_in_count.has_value());
    EXPECT_NEAR(m.sync_in_count.value(), 42.0, 1e-6);
}

TEST(MeasurementsTest, CalibratedImuTemperatureConversion) {
    Measurements m;
    m.add(SensorId::calibrated_imu_temperature, 0);
    ASSERT_TRUE(m.calibrated_imu_temperature.has_value());
    EXPECT_NEAR(m.calibrated_imu_temperature.value(), 20.0, 1e-6);
}

TEST(MeasurementsTest, GnssNumSatParsing) {
    Measurements m;
    int32_t raw = (8 << 16) | 12;
    m.add(SensorId::gnss_num_sat, raw);
    ASSERT_TRUE(m.gnss1_num_sat.has_value());
    ASSERT_TRUE(m.gnss2_num_sat.has_value());
    EXPECT_NEAR(m.gnss1_num_sat.value(), 12.0, 1e-6);
    EXPECT_NEAR(m.gnss2_num_sat.value(), 8.0, 1e-6);
}

TEST(MeasurementsTest, UtcYearMonthParsing) {
    Measurements m;
    int32_t raw = (2026 << 16) | 2;
    m.add(SensorId::utc_year_month, raw);
    ASSERT_TRUE(m.utc_year.has_value());
    ASSERT_TRUE(m.utc_month.has_value());
    EXPECT_NEAR(m.utc_year.value(), 2026.0, 1e-6);
    EXPECT_NEAR(m.utc_month.value(), 2.0, 1e-6);
}

TEST(MeasurementsTest, UtcDayTimeParsing) {
    Measurements m;
    int32_t raw = (20 << 24) | (14 << 16) | (30 << 8) | 45;
    m.add(SensorId::utc_day_time, raw);
    ASSERT_TRUE(m.utc_day.has_value());
    ASSERT_TRUE(m.utc_hour.has_value());
    ASSERT_TRUE(m.utc_min.has_value());
    ASSERT_TRUE(m.utc_sec.has_value());
    EXPECT_NEAR(m.utc_day.value(), 20.0, 1e-6);
    EXPECT_NEAR(m.utc_hour.value(), 14.0, 1e-6);
    EXPECT_NEAR(m.utc_min.value(), 30.0, 1e-6);
    EXPECT_NEAR(m.utc_sec.value(), 45.0, 1e-6);
}

TEST(MeasurementsTest, GnssFixTypeParsing) {
    Measurements m;
    int32_t raw = (2 << 16) | 3; // GNSS1=3D, GNSS2=2D
    m.add(SensorId::gnss_fix_type, raw);
    ASSERT_TRUE(m.gnss1_fix_type.has_value());
    ASSERT_TRUE(m.gnss2_fix_type.has_value());
    EXPECT_EQ(m.gnss1_fix_type.value(), GnssFixType::Fix3D);
    EXPECT_EQ(m.gnss2_fix_type.value(), GnssFixType::Fix2D);
}

TEST(MeasurementsTest, RollPitchHeadingAreRadians) {
    Measurements m;
    m.add(SensorId::roll, 45000000);     // 45 deg
    m.add(SensorId::pitch, -30000000);   // -30 deg
    m.add(SensorId::heading, 270000000); // 270 deg

    ASSERT_TRUE(m.roll.has_value());
    ASSERT_TRUE(m.pitch.has_value());
    ASSERT_TRUE(m.heading.has_value());

    EXPECT_NEAR(m.roll.value(), std::numbers::pi / 4.0, 1e-4);
    EXPECT_NEAR(m.pitch.value(), -std::numbers::pi / 6.0, 1e-4);
    EXPECT_NEAR(m.heading.value(), 3.0 * std::numbers::pi / 2.0, 1e-4);
}

// Helper: compute CRC16 (poly 0x1021, init 0) matching KebniDriver::crc16
static uint16_t computeCrc16(const std::vector<uint8_t> &data) {
    const uint16_t poly = 0x1021;
    uint16_t crc = 0;
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Helper: feed a complete packet (start byte + payload + checksum) into a driver
static void feedPacketXor8(TestDriver &driver, const std::vector<uint8_t> &payload) {
    uint8_t xor_checksum = 0;
    for (uint8_t b : payload)
        xor_checksum ^= b;

    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(xor_checksum);
}

static void feedPacketCrc16(TestDriver &driver, const std::vector<uint8_t> &payload) {
    uint16_t crc = computeCrc16(payload);

    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(static_cast<uint8_t>(crc >> 8));   // MSB first
    driver.processByte(static_cast<uint8_t>(crc & 0xFF)); // LSB second
}

// XOR8 checksum tests

TEST(DriverIntegrationTest, Xor8FullPacketDecoding) {
    TestDriver driver;

    // Config: 500Hz, 8-byte payload, accX + gyroX, XOR8 checksum ('x')
    std::string config = "o0002s08003002001000033032031030x";
    auto result = driver.configureKebniDriver(config);
    ASSERT_EQ(result.configError, kebni::ConfigError::None);
    ASSERT_TRUE(result.configured);

    // accX = 1000000 (0x000F4240), gyroX = 1000000 (0x000F4240)
    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};
    feedPacketXor8(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    ASSERT_TRUE(driver.lastMeasurements.gyroX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
    EXPECT_NEAR(driver.lastMeasurements.gyroX.value(), std::numbers::pi / 180.0, 1e-6);
}

TEST(DriverIntegrationTest, Xor8InvalidChecksum) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030x";
    driver.configureKebniDriver(config);

    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};

    // Feed with wrong checksum (flip a bit)
    uint8_t xor_checksum = 0;
    for (uint8_t b : payload)
        xor_checksum ^= b;
    xor_checksum ^= 0x01; // corrupt it

    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(xor_checksum);

    ASSERT_TRUE(driver.received);
    EXPECT_TRUE(driver.lastMeasurements.invalidChecksum);
    EXPECT_FALSE(driver.lastMeasurements.accX.has_value());
}

TEST(DriverIntegrationTest, Xor8MultiplePackets) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030x";
    driver.configureKebniDriver(config);

    // First packet: accX = 1000000
    std::vector<uint8_t> payload1 = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};
    feedPacketXor8(driver, payload1);
    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);

    // Second packet: accX = 2000000 (0x001E8480)
    driver.received = false;
    std::vector<uint8_t> payload2 = {0x00, 0x1E, 0x84, 0x80, 0x00, 0x0F, 0x42, 0x40};
    feedPacketXor8(driver, payload2);
    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 2.0 * 9.80665, 1e-3);
}

// CRC16 checksum tests

TEST(DriverIntegrationTest, Crc16FullPacketDecoding) {
    TestDriver driver;

    // Same sensors as XOR8 test but with CRC16 checksum ('X')
    std::string config = "o0002s08003002001000033032031030X";
    auto result = driver.configureKebniDriver(config);
    ASSERT_EQ(result.configError, kebni::ConfigError::None);
    ASSERT_TRUE(result.configured);

    // accX = 1000000 (0x000F4240), gyroX = 1000000 (0x000F4240)
    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};
    feedPacketCrc16(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    ASSERT_TRUE(driver.lastMeasurements.gyroX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
    EXPECT_NEAR(driver.lastMeasurements.gyroX.value(), std::numbers::pi / 180.0, 1e-6);
}

TEST(DriverIntegrationTest, Crc16InvalidChecksum) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030X";
    driver.configureKebniDriver(config);

    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};

    // Feed with wrong CRC16 (send 0x0000 instead of real CRC)
    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(0x00); // wrong MSB
    driver.processByte(0x00); // wrong LSB

    ASSERT_TRUE(driver.received);
    EXPECT_TRUE(driver.lastMeasurements.invalidChecksum);
    EXPECT_FALSE(driver.lastMeasurements.accX.has_value());
}

TEST(DriverIntegrationTest, Crc16MultiplePackets) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030X";
    driver.configureKebniDriver(config);

    // First packet
    std::vector<uint8_t> payload1 = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};
    feedPacketCrc16(driver, payload1);
    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);

    // Second packet with different data
    driver.received = false;
    std::vector<uint8_t> payload2 = {0x00, 0x1E, 0x84, 0x80, 0x00, 0x0F, 0x42, 0x40};
    feedPacketCrc16(driver, payload2);
    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 2.0 * 9.80665, 1e-3);
}

TEST(DriverIntegrationTest, Crc16RecoveryAfterInvalidChecksum) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030X";
    driver.configureKebniDriver(config);

    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};

    // Send a bad packet first
    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(0xFF); // wrong MSB
    driver.processByte(0xFF); // wrong LSB

    ASSERT_TRUE(driver.received);
    EXPECT_TRUE(driver.lastMeasurements.invalidChecksum);

    // Now send a valid packet — driver should recover
    driver.received = false;
    feedPacketCrc16(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
}

TEST(DriverIntegrationTest, Xor8RecoveryAfterInvalidChecksum) {
    TestDriver driver;

    std::string config = "o0002s08003002001000033032031030x";
    driver.configureKebniDriver(config);

    std::vector<uint8_t> payload = {0x00, 0x0F, 0x42, 0x40, 0x00, 0x0F, 0x42, 0x40};

    // Send a bad packet
    driver.processByte(0xFA);
    for (uint8_t b : payload)
        driver.processByte(b);
    driver.processByte(0xFF); // wrong checksum

    ASSERT_TRUE(driver.received);
    EXPECT_TRUE(driver.lastMeasurements.invalidChecksum);

    // Send a valid packet — driver should recover
    driver.received = false;
    feedPacketXor8(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
}

TEST(DriverIntegrationTest, Crc16WithMoreSensors) {
    TestDriver driver;

    // 16-byte payload: accX(4) + gyroX(4) + roll(4) + heading(4), CRC16
    std::string config = "o0002s10003002001000033032031030373372371370393392391390X";
    auto result = driver.configureKebniDriver(config);
    ASSERT_EQ(result.configError, kebni::ConfigError::None);
    ASSERT_TRUE(result.configured);

    // accX = 1000000 (0x000F4240)
    // gyroX = 500000  (0x0007A120)
    // roll = 45000000 (0x02AEA540) = 45 deg -> pi/4 rad
    // heading = 90000000 (0x055D4A80) = 90 deg -> pi/2 rad
    std::vector<uint8_t> payload = {
        0x00,
        0x0F,
        0x42,
        0x40, // accX
        0x00,
        0x07,
        0xA1,
        0x20, // gyroX
        0x02,
        0xAE,
        0xA5,
        0x40, // roll
        0x05,
        0x5D,
        0x4A,
        0x80, // heading
    };
    feedPacketCrc16(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);

    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
    ASSERT_TRUE(driver.lastMeasurements.gyroX.has_value());
    EXPECT_NEAR(driver.lastMeasurements.gyroX.value(), 0.5 * std::numbers::pi / 180.0, 1e-6);
    ASSERT_TRUE(driver.lastMeasurements.roll.has_value());
    EXPECT_NEAR(driver.lastMeasurements.roll.value(), std::numbers::pi / 4.0, 1e-4);
    ASSERT_TRUE(driver.lastMeasurements.heading.has_value());
    EXPECT_NEAR(driver.lastMeasurements.heading.value(), std::numbers::pi / 2.0, 1e-4);
}

TEST(DriverIntegrationTest, ScrambledByteOrder) {
    TestDriver driver;

    // accX encoded as bytes [1,3,0,2] — intentionally scrambled order
    // Config: o0001 s04 001 003 000 002 x
    std::string config = "o0001s04001003000002x";
    auto result = driver.configureKebniDriver(config);
    ASSERT_EQ(result.configError, kebni::ConfigError::None);
    ASSERT_TRUE(result.configured);
    ASSERT_EQ(result.messageByteMapping[0].byteOrder, (std::vector<uint8_t>{1, 3, 0, 2}));

    // Build payload: positions [0,1,2,3] carry bytes [1,3,0,2] of raw=0x000F4240
    // byte0=0x40 byte1=0x42 byte2=0x0F byte3=0x00
    // payload = [byte1, byte3, byte0, byte2] = [0x42, 0x00, 0x40, 0x0F]
    std::vector<uint8_t> payload = {0x42, 0x00, 0x40, 0x0F};
    feedPacketXor8(driver, payload);

    ASSERT_TRUE(driver.received);
    ASSERT_FALSE(driver.lastMeasurements.invalidChecksum);
    ASSERT_TRUE(driver.lastMeasurements.accX.has_value());
    // raw = 1,000,000 µg = 1g = 9.80665 m/s²
    EXPECT_NEAR(driver.lastMeasurements.accX.value(), 9.80665, 1e-4);
}
