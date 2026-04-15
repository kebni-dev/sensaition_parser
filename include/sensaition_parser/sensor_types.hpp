#ifndef PARSER_TYPES_H
#define PARSER_TYPES_H

#include <array>
#include <bitset>
#include <cassert>

namespace sepa
{
    // CONSTANTS =======================================================================================================

    constexpr int SENSOR_DATA_BYTES = 320;         // Maximum number of parsed bytes
    constexpr int SENSOR_DATA_NUM_MESSAGES = 8;    // Maximum number of interleaved messages

    // ENUMERATIONS ===============================================================================================

    /// @brief The different hardware models available
    /// Values must match those in the SensAItion datasheet!
    enum SensaitionHardwareModel
    {
        IMU = 0,  // IMU product, only supports raw IMU signals
        AHRS = 1, // AHRS product, supports attitude readings
        INS = 2,  // INS product, supports attitude and position readings
        DEV = 3   // "special" product with extra dev tools enabled
    }; // SensaitionHardwareModel

    /// @brief Enum of all supported unique sensor data values
    /// The value for each enum must match the sensor value index in the SensAItion User Manual.
    enum SensorDataValueId
    {
        ACC_X = 0x00,      // Accelerometer X raw value [g]
        ACC_Y = 0x01,      // Accelerometer Y raw value [g]
        ACC_Z = 0x02,      // Accelerometer Z raw value [g]
        GYRO_X = 0x03,     // Gyro X raw value [°/s]
        GYRO_Y = 0x04,     // Gyro Y raw value [°/s]
        GYRO_Z = 0x05,     // Gyro Z raw value [°/s]
        INCL_X = 0x06,     // Inclinometer X raw value [g]
        INCL_Y = 0x07,     // Inclinometer Y raw value [g]
        INCL_Z = 0x08,     // Inclinometer Z raw value [g]
        TEMP = 0x09,       // IMU temperature (internal) [°C]
        MAG_X = 0x0A,      // Magnetometer X raw value [gauss]
        MAG_Y = 0x0B,      // Magnetometer Y raw value [gauss]
        MAG_Z = 0x0C,      // Magnetometer Z raw value [gauss]
        BAROMETER = 0x0D,  // Barometer raw value [hPa]
        ODOMETER = 0x0E,   // Odometer speed raw value [m/s]
        TEMP_CALIB = 0x0F, // Calibrated IMU temperature (estimated external) [°C]

// For dev testing we can manually swap to sensor calibration data
// Normally this space is used for raw GNSS data
// #define USE_SENSOR_CALIB_DATA
#ifndef USE_SENSOR_CALIB_DATA
        GNSS_LATITUDE = 0x10,
        GNSS_LONGITUDE = 0x11,
        GNSS_ALTITUDE = 0x12,
        GNSS_VEL_NORTH = 0x13,
        GNSS_VEL_EAST = 0x14,
        GNSS_VEL_DOWN = 0x15,
        GNSS_SPEED = 0x16,
        GNSS_COURSE = 0x17,
        GNSS_STD_HORZ = 0x18,
        GNSS_STD_VERT = 0x19,
        GNSS_STD_SPEED = 0x1A,
        GNSS_STD_COURSE = 0x1B,
        GNSS_PDOP = 0x1C,
        GNSS_GEOID_SEP = 0x1D,
        GNSS_UTC_YEAR_MONTH = 0x1E,
        GNSS_UTC_DAY_H_MIN_S = 0x1F,
        GNSS_FLAGS = 0x20,
        GNSS_RELPOS_FLAGS = 0x21,
#else
        // Calibration specific data, for developer testing
        MEAN_ACC_X = 0x10,
        MEAN_ACC_Y = 0x11,
        MEAN_ACC_Z = 0x12,
        MEAN_GYRO_X = 0x13,
        MEAN_GYRO_Y = 0x14,
        MEAN_GYRO_Z = 0x15,
        MEAN_MAG_X = 0x16,
        MEAN_MAG_Y = 0x17,
        MEAN_MAG_Z = 0x18,
        STD_ACC_X = 0x19,
        STD_ACC_Y = 0x1A,
        STD_ACC_Z = 0x1B,
        STD_GYRO_X = 0x1C,
        STD_GYRO_Y = 0x1D,
        STD_GYRO_Z = 0x1E,
        STD_MAG_X = 0x1F,
        STD_MAG_Y = 0x20,
        STD_MAG_Z = 0x21,
        COUNTER = 0x22,
#endif

        FRTK_ITOW = 0x23,  // GNSS fixed base relpos time-of-week timestamp [ms]
        FRTK_NORTH = 0x24, // GNSS fixed base relpos north offset [m]
        FRTK_EAST = 0x25,  // GNSS fixed base relpos east offset [m]
        FRTK_DOWN = 0x26,  // GNSS fixed base relpos vertical offset [m]

        MRTK_ITOW = 0x27,  // GNSS moving base relpos time-of-week timestamp [ms]
        MRTK_NORTH = 0x28, // GNSS moving base relpos north offset [m]
        MRTK_EAST = 0x29,  // GNSS moving base relpos east offset [m]
        MRTK_DOWN = 0x2A,  // GNSS moving base relpos vertical offset [m]

        NUM_SAT = 0x2B, // GNSS number of satellites used in navigation solution

        SENSORS_USED = 0x2E, // Sensors used in navigation update
        ERROR_FLAGS = 0x2F,  // Currently flagged errors

        SENSOR_VALID = 0x30, // Available sensor signals

        POS_LATITUDE = 0x31,  // Estimated horizontal position latitude [°]
        POS_LONGITUDE = 0x32, // Estimated horizontal position longitude [°]
        VEL_NORTH = 0x33,     // Estimated horizontal speed north
        VEL_EAST = 0x34,      // Estimated horizontal speed east [m/s]
        POS_VERTICAL = 0x35,  // Estimated vertical position/altitude [m]
        VEL_VERTICAL = 0x36,  // Estimated vertical speed [m/s]

        ROLL = 0x37,    // Estimated roll [°]
        PITCH = 0x38,   // Estimated pitch [°]
        HEADING = 0x39, // Estimated heading/yaw [°]

        CORR_ACC_X = 0x3A,  // Estimated corrected accelerometer X values [g]
        CORR_ACC_Y = 0x3B,  // Estimated corrected accelerometer Y values [g]
        CORR_ACC_Z = 0x3C,  // Estimated corrected accelerometer Z values [g]
        CORR_GYRO_X = 0x3D, // Estimated corrected gyro X values [°/s]
        CORR_GYRO_Y = 0x3E, // Estimated corrected gyro Y values [°/s]
        CORR_GYRO_Z = 0x3F, // Estimated corrected gyro Z values [°/s]

        ALIGNMENT_INS = 0x40,   // Alignment status, set if unit is in INS mode
        ATTITUDE_STATUS = 0x41, // Attitude status, set if unit has valid roll, pitch and heading
        TICK = 0x42,            // System time since startup [ms]

        GNSS1_ITOW = 0x43, // GNSS1 time-of-week timestamp [ms]
        GNSS2_ITOW = 0x44, // GNSS2 time-of-week timestamp [ms]

        UTC_YEAR_MONTH = 0x45,  // UTC year and month timestamp [year,month]
        UTC_DAY_H_MIN_S = 0x46, // UTC day, hours, minutes and seconds timestamp [day,h,min,s]

        GNSS_FIXTYPE = 0x47, // Current fix type by GNSS1 & GNSS2 (none,2D,3D fix)

        SYNC_IN_COUNT = 0x48, // Number of sync-in pulses detected
        SYNC_IN_TIME = 0x49,  // Time since last read sync-in pulse [µs]

        UTC_US = 0x4A,     // UTC subsecond timestamp [µs]
        UTC_STATUS = 0x4B, // UTC timestamp status

        Q_W = 0x4C, // Estimated attitude quaternion scalar component
        Q_X = 0x4D, // Estimated attitude quaternion X component
        Q_Y = 0x4E, // Estimated attitude quaternion Y component
        Q_Z = 0x4F, // Estimated attitude quaternion Z component

        ROTMAT_11 = 0x50, // Estimated attitude rotation matrix (1,1) element
        ROTMAT_12 = 0x51, // Estimated attitude rotation matrix (1,2) element
        ROTMAT_13 = 0x52, // Estimated attitude rotation matrix (1,3) element
        ROTMAT_21 = 0x53, // Estimated attitude rotation matrix (2,1) element
        ROTMAT_22 = 0x54, // Estimated attitude rotation matrix (2,2) element
        ROTMAT_23 = 0x55, // Estimated attitude rotation matrix (2,3) element
        ROTMAT_31 = 0x56, // Estimated attitude rotation matrix (3,1) element
        ROTMAT_32 = 0x57, // Estimated attitude rotation matrix (3,2) element
        ROTMAT_33 = 0x58, // Estimated attitude rotation matrix (3,3) element

        ECEF_POS_X = 0x59, // ECEF position X [m]
        ECEF_POS_Y = 0x5A, // ECEF position Y [m]
        ECEF_POS_Z = 0x5B, // ECEF position Z [m]

        PERF_TICK = 0x60, // High performance time tick [µs]

        STD_LATITUDE = 0x61,     // Estimated quality horizontal position latitude [m]
        STD_LONGITUDE = 0x62,    // Estimated quality horizontal position longitude [m]
        STD_VEL_NORTH = 0x63,    // Estimated quality horizontal speed north [m/s]
        STD_VEL_EAST = 0x64,     // Estimated quality horizontal speed east [m/s]
        STD_POS_VERTICAL = 0x65, // Estimated quality vertical position/altitude [m]
        STD_VEL_VERTICAL = 0x66, // Estimated quality vertical speed [m/s]
        STD_ROLL = 0x67,         // Estimated quality roll [°]
        STD_PITCH = 0x68,        // Estimated quality pitch [°]
        STD_HEADING = 0x69,      // Estimated quality heading/yaw [°]

#if 0
        NAV02S_ACC_X  = 0x70, // NAV02S scaled accelerometer X [g]
        NAV02S_ACC_Y  = 0x71, // NAV02S scaled accelerometer Y [g]
        NAV02S_ACC_Z  = 0x72, // NAV02S scaled accelerometer Z [g]
        NAV02S_GYRO_X = 0x73, // NAV02S scaled gyro X [°/sec]
        NAV02S_GYRO_Y = 0x74, // NAV02S scaled gyro Y [°/sec]
        NAV02S_GYRO_Z = 0x75, // NAV02S scaled gyro Z [°/sec]
        NAV02S_TEMP   = 0x76, // NAV02S scaled temperature [°C]
#endif

        // Developer debug data
        UTC_SEC_PERIOD = 0x7A,
        TEMP_MAG = 0x7B,
        WDO_PIN = 0x7C,
        GNSS_DELAY = 0x7D,
        KALMAN_TIME = 0x7E,
        NAV_TIME = 0x7F,

        SENSOR_DATA_NUM = 0x80 // Max number of parsable data elements available from sensor
    }; // SensorDataValueId

    /// @brief Available selectable standard NMEA messages on User UART
    enum StandardNmeaMessages
    {
        STANDARD_NMEA_GGA = 0, // Will output standard GNGGA messages
        STANDARD_NMEA_RMC,     // Will output standard GNRMC messages
        STANDARD_NMEA_GLL,     // Will output standard GNGLL messages
        STANDARD_NMEA_GST,     // Will output standard GNGST messages
        STANDARD_NMEA_NUM      // Max number of available standard NMEA messages
    };

    /// @brief Available sources for standard NMEA messages on User UART
    enum StandardNmeaSources
    {
        STANDARD_NMEA_SOURCE_NAV = 0, // Standard NMEA messages will contain NAV INS data
        STANDARD_NMEA_SOURCE_GNSS1,   // Standard NMEA messages will contain raw GNSS1 data
        STANDARD_NMEA_SOURCE_GNSS2,   // Standard NMEA messages will contain raw GNSS2 data
        STANDARD_NMEA_SOURCE_NUM      // Max number of standard NMEA sources
    };

    enum CanIdentifierTypes
    {
        CAN_STANDARD_ID,             // Standard 11-bit id
        CAN_EXTENDED_ID,             // Extended full 29-bit id
        CAN_EXTENDED_ID_WITH_CHIP_ID // 5-bit custom id (bit 24:28) + 24 bit chip id (bit 0:23)
    };

    // CUSTOM DATA TYPES ===============================================================================================

    /// @brief Bitset that says which standard NMEA messages from which sources are enabled
    class StandardNmeaSelection
    {
    public:
        StandardNmeaSelection() { data.reset(); }
        ~StandardNmeaSelection() {}

        bool operator==(const StandardNmeaSelection &other) { return data == other.data; }
        bool operator!=(const StandardNmeaSelection &other) { return data != other.data; }

        /// @brief Maps message & source to internal storage array index
        static int index(StandardNmeaMessages msg, StandardNmeaSources src)
        {
            assert(msg < StandardNmeaMessages::STANDARD_NMEA_NUM && src < StandardNmeaSources::STANDARD_NMEA_SOURCE_NUM);
            return (static_cast<int>(msg) * StandardNmeaSources::STANDARD_NMEA_SOURCE_NUM) + static_cast<int>(src);
        }

        void set(StandardNmeaMessages msg, StandardNmeaSources src) { data.set(index(msg, src), true); }
        void set(int idx) { data.set(idx, true); }
        bool get(StandardNmeaMessages msg, StandardNmeaSources src) const { return data.test(index(msg, src)); }
        bool get(int idx) const { return data.test(idx); }
        void clear(StandardNmeaMessages msg, StandardNmeaSources src) { data.set(index(msg, src), false); }
        void clear(int idx) { data.set(idx, false); }
        void clear_all(void) { data.reset(); }
        bool empty(void) const { return !data.any(); }
        inline int size(void) const { return static_cast<int>(data.size()); }

    private:
        std::bitset<static_cast<std::size_t>(StandardNmeaMessages::STANDARD_NMEA_NUM) *
                    static_cast<std::size_t>(StandardNmeaSources::STANDARD_NMEA_SOURCE_NUM)>
            data;
    };

    /// @brief Datatype for generating and parsing config strings, a bitfield where each entry determines
    /// if that sensor data should be included in the string
    using DataSelection = std::bitset<SensorDataValueId::SENSOR_DATA_NUM>;

    /// @brief Contains all the information required to assemble a valid data uart config string.
    ///        For interleaved messages, multiple of these are required
    struct DataUartMessage
    {
        DataSelection selection; // Selected elements

        int dataRate;        // output rate (1000 Hz divisor)
        int checksumBytes;   // number of checksum bytes (0 - 2)
        int numIdentifier;   // number of identifier bytes following the header (0 - 4)
        uint32_t identifier; // identifier following the header (always in big endian format)

        bool useBigEndian; // true if we should send bytes in big endian order, false if we should send in little endian
        bool useCompact;   // If true, uses the new compact config string format, if false uses the standard format

        void reset()
        {
            selection.reset();
            dataRate = 10;
            checksumBytes = 0;
            numIdentifier = 0;
            identifier = 0;
            useBigEndian = false;
            useCompact = false;
        }
    };

    struct DataUartMessageSet
    {
        std::array<DataUartMessage, SENSOR_DATA_NUM_MESSAGES> message;
        int numMessages;
    };

    /// @brief Contains all the information required to assemble ONE can message
    struct CanMessage
    {
        DataSelection selection;   // Selected elements (should only contain a max of 8 bytes of raw data)
        int dataRate;              // output rate (1000 Hz divisor)
        int phase;                 // custom phase (0-255) (0 = 0%/0°, 128 = 50%/180° etc.)
        uint32_t identifier;       // identifier (either 11-bit or 29-bit)
        CanIdentifierTypes idType; // Type of identifier
        bool useBigEndian;         // true if we should send bytes in big endian order, false if we should send in little endian

        void reset()
        {
            selection.reset();
            dataRate = 10;
            phase = 0;
            idType = CanIdentifierTypes::CAN_STANDARD_ID;
            identifier = 0;
            useBigEndian = false;
        }
    };

    // CONFIG STRING PARSERS ===========================================================================================

    // Parse data entry for a single data byte
    struct ParsedDataEntry
    {
        uint8_t row; // Index of the sensor data value (0 - 127)
        uint8_t col; // Index of which byte of the value this represents (0 - 3)
    };

    /// @brief Info about the parsed data uart config string
    struct DataUartParseInfo
    {
        // Parsed message
        struct DataUartParsedMessage
        {
            uint16_t dataRate;     // output rate (1000 Hz divisor)
            uint16_t startIndex;   // start index of the data[] array
            uint16_t numBytes;     // number of parsed bytes
            uint8_t checksumBytes; // number of checksum bytes (0 - 2)
            uint8_t numIdentifier; // number of identifier bytes following the header (0 - 4), mandatory if using multiple messages
            uint32_t identifier;   // identifier following the header (always in big endian format)
        };

        uint16_t numMessages; // Number of parsed messages, is always 0 for an invalid parse
        bool usingBigEndian;  // True if using big endian order (only informative)

        // One config string can define multiple messages
        std::array<DataUartParsedMessage, SENSOR_DATA_NUM_MESSAGES> messages;

        // data[i] describes how to interpret byte i of the message,
        // where i = 0 for the first byte AFTER the header.
        std::array<ParsedDataEntry, SENSOR_DATA_BYTES> data;
    };

    /// @brief Info about a single parsed CAN message
    struct CanParseInfo
    {
        int dataRate; // output rate (1000 Hz divisor)
        int phase;    // custom phase (0-255) (0 = 0%/0°, 128 = 50%/180° etc.)

        // todo: group? Should group related messages together (i.e. same data-rate and phase)
        // so that we only inform about an update sample when all messages of that group is received.

        uint32_t identifier;       // identifier (either 11-bit or 29-bit)
        CanIdentifierTypes idType; // type of identifier

        bool usingBigEndian; // True if using big endian order (only informative)

        uint8_t dataLength; // number of data bytes
        std::array<ParsedDataEntry, 8> data;
    };

    /// @brief Calculates the 16-bit crc for a given range of bytes
    inline uint16_t crc16(uint8_t const *data, std::size_t length)
    {
        const uint16_t poly = 0x1021; // polynomial used by sensaition
        uint16_t crc = 0;

        for (std::size_t i = 0; i < length; i++)
        {
            crc ^= *(data + i) << 8;
            for (std::size_t j = 0; j < 8; j++)
            {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ poly;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

} // namespace
#endif // PARSER_TYPES_H
