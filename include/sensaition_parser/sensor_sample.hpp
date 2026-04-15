#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#include <cstdint>
#include <string>
#include <map>
#include <variant>
#include <vector>

namespace sepa
{
    class SensorSample
    {
    public:
        /**
         * @brief The physical quantities we can read from the device.
         * Related entries are grouped together and ordered by priority (highest first).
         * Several entries are also unpacked from their original data representation.
         */
        enum MeasurementType
        {
            /* Timestamps */
            TICK,       // System time since startup [ms]
            PERF_TICK,  // High performance time tick [µs]
            UTC_YEAR,   // UTC timestamp year
            UTC_MONTH,  // UTC timestamp month
            UTC_DAY,    // UTC timestamp day
            UTC_HOUR,   // UTC timestamp hour
            UTC_MINUTE, // UTC timestamp minute
            UTC_SECOND, // UTC timestamp second
            UTC_US,     // UTC timestamp subsecond [µs]

            /* System status */
            SENSOR_VALID,       // Available sensor signals (bitfield)
            SENSORS_USED,       // Sensors used in nav update (bitfield)
            ALIGNMENT_INS,      // Alignment status, 1 if unit is in INS mode
            ATTITUDE_STATUS,    // Attitude status, 1 if unit has valid roll, pitch and heading
            UTC_STATUS,         // UTC timestamp status (0 = no fix, 1 = possibly invalid, 2 = valid)
            ERROR_FLAGS,        // Currently flagged errors (bitfield)

            SYNC_IN_COUNT,      // Number of sync-in pulses detected
            SYNC_IN_TIME,       // Time since last read sync-in pulse [µs]

            /* IMU data */
            ACCELERATION_X,     // Raw X accelerometer value [g]
            ACCELERATION_Y,     // Raw Y accelerometer value [g]
            ACCELERATION_Z,     // Raw X accelerometer value [g]
            GYRO_RATE_X,        // Angular rate around X-axis [°/s]
            GYRO_RATE_Y,        // Angular rate around Y-axis [°/s]
            GYRO_RATE_Z,        // Angular rate around Z-axis [°/s]
            INCLINOMETER_X,     // Raw X inclinometer value [g]
            INCLINOMETER_Y,     // Raw X inclinometer value [g]
            INCLINOMETER_Z,     // Raw X inclinometer value [g]
            INTERNAL_TEMP,      // Internal IMU temperature [°C]
            MAGNETIC_FIELD_X,   // Raw X magnetometer value [gauss]
            MAGNETIC_FIELD_Y,   // Raw Y magnetometer value [gauss]
            MAGNETIC_FIELD_Z,   // Raw Z magnetometer value [gauss]
            AIR_PRESSURE,       // Barometer reading [hPa]
            ODOMETER_SPEED,     // Odometer reading [m/s]
            EXTERNAL_TEMP,      // Calibrated estimated external temperature [°C]

            CORR_ACCELERATION_X,    // Estimated corrected accelerometer X value [g]
            CORR_ACCELERATION_Y,    // Estimated corrected accelerometer Y value [g]
            CORR_ACCELERATION_Z,    // Estimated corrected accelerometer Z value [g]
            CORR_GYRO_RATE_X,       // Estimated corrected angular rate around X-axis [°/s]
            CORR_GYRO_RATE_Y,       // Estimated corrected angular rate around Y-axis [°/s]
            CORR_GYRO_RATE_Z,       // Estimated corrected angular rate around Z-axis [°/s]

            /* GNSS */
            GNSS_LATITUDE,          // GNSS1 raw latitude value [°]
            GNSS_LONGITUDE,         // GNSS1 raw longitude value [°]
            GNSS_ALTITUDE,          // GNSS1 raw altitude over ellipsoid value [m]
            GNSS_VEL_NORTH,         // GNSS1 raw velocity north value [m/s]
            GNSS_VEL_EAST,          // GNSS1 raw velocity east value [m/s]
            GNSS_VEL_DOWN,          // GNSS1 raw velocity down value [m/s]
            GNSS_SPEED,             // GNSS1 raw speed value [m/s]
            GNSS_COURSE,            // GNSS1 raw course over ground value [°]
            GNSS_STD_HORZIONTAL,    // GNSS1 raw quality, horizontal position, standard deviation [m]
            GNSS_STD_VERTICAL,      // GNSS1 raw quality, vertical position, standard deviation [m]
            GNSS_STD_SPEED,         // GNSS1 raw quality, velocity, standard deviation [m/s]
            GNSS_STD_COURSE,        // GNSS1 raw quality, course over ground standard deviation [°]
            GNSS_PDOP,              // GNSS1 positional dilution of precision
            GNSS_GEOID_SEP,         // GNSS1 geoid separation: height above MSL = altitude - geoid_sep [m]
            GNSS_UTC_YEAR,          // GNSS1 UTC year
            GNSS_UTC_MONTH,         // GNSS1 UTC month
            GNSS_UTC_DAY,           // GNSS1 UTC day
            GNSS_UTC_HOUR,          // GNSS1 UTC hour
            GNSS_UTC_MINUTE,        // GNSS1 UTC minute
            GNSS_UTC_SECOND,        // GNSS1 UTC seconds

            GNSS1_FLAGS,    // GNSS1 raw ublox receiver position flags
            GNSS2_FLAGS,    // GNSS2 raw ublox receiver position flags
            FRTK_FLAGS,     // GNSS1 raw ublox receiver fixed base RTK flags
            MRTK_FLAGS,     // GNSS2 raw ublox receiver moving base RTK flags
            GNSS1_ITOW,     // GNSS1 time-of-week timestamp [ms]
            GNSS2_ITOW,     // GNSS2 time-of-week timestamp [ms]
            GNSS1_FIXTYPE,  // Current fix type for GNSS 1 (none, 2D or 3D fix)
            GNSS2_FIXTYPE,  // Current fix type for GNSS 2 (none, 2D or 3D fix)
            GNSS1_NUM_SAT,  // Number of satellites used in GNSS 1 navigation solution
            GNSS2_NUM_SAT,  // Number of satellites used in GNSS 2 navigation solution

            FRTK_ITOW,      // GNSS fixed base relative pos time-of-week timestamp [ms]
            FRTK_NORTH,     // GNSS fixed base relative pos north offset [m]
            FRTK_EAST,      // GNSS fixed base relative pos east offset [m]
            FRTK_DOWN,      // GNSS fixed base relative pos vertical offset [m]
            MRTK_ITOW,      // GNSS moving base relative pos time-of-week timestamp [ms]
            MRTK_NORTH,     // GNSS moving base relative pos north offset [m]
            MRTK_EAST,      // GNSS moving base relative pos east offset [m]
            MRTK_DOWN,      // GNSS moving base relative pos vertical offset [m]

            /* Attitude */
            ROLL,           // Estimated roll [°]
            PITCH,          // Estimated pitch [°]
            HEADING,        // Estimated heading/yaw [°]

            Q_W, // Estimated attitude quaternion scalar component
            Q_X, // Estimated attitude quaternion X or i component
            Q_Y, // Estimated attitude quaternion Y or j component
            Q_Z, // Estimated attitude quaternion Z or k component

            ROTMAT_11,  // Estimated attitude rotation matrix (1,1) element
            ROTMAT_12,  // Estimated attitude rotation matrix (1,2) element
            ROTMAT_13,  // Estimated attitude rotation matrix (1,3) element
            ROTMAT_21,  // Estimated attitude rotation matrix (2,1) element
            ROTMAT_22,  // Estimated attitude rotation matrix (2,2) element
            ROTMAT_23,  // Estimated attitude rotation matrix (2,3) element
            ROTMAT_31,  // Estimated attitude rotation matrix (3,1) element
            ROTMAT_32,  // Estimated attitude rotation matrix (3,2) element
            ROTMAT_33,  // Estimated attitude rotation matrix (3,3) element

            STD_ROLL,           // Estimated quality roll [°]
            STD_PITCH,          // Estimated quality pitch [°]
            STD_HEADING,        // Estimated quality heading/yaw [°]

            /* Position */
            POS_LATITUDE,   // Estimated horizontal position latitude [°]
            POS_LONGITUDE,  // Estimated horizontal position longitude [°]
            VEL_NORTH,      // Estimated velocity north [m/s]
            VEL_EAST,       // Estimated velocity east [m/s]
            POS_VERTICAL,   // Estimated vertical position/altitude [m]
            VEL_DOWN,       // Estimated velocity down [m/s]

            ECEF_POS_X, // ECEF position X [m]
            ECEF_POS_Y, // ECEF position Y [m]
            ECEF_POS_Z, // ECEF position Z [m]

            /* Quality Estimates */
            STD_LATITUDE,       // Estimated quality horizontal position latitude [m]
            STD_LONGITUDE,      // Estimated quality horizontal position longitude [m]
            STD_VEL_NORTH,      // Estimated quality velocity north [m/s]
            STD_VEL_EAST,       // Estimated quality velocity east [m/s]
            STD_POS_VERTICAL,   // Estimated quality vertical position/altitude [m]
            STD_VEL_DOWN,       // Estimated quality velocity down [m/s]

            /* Various unlabeled debug data */
            DEBUG1,
            DEBUG2,
            DEBUG3,
            DEBUG4,
            DEBUG5,
            DEBUG6,

            INVALID // Only used for SensorDataValue items that should not be included in a SensorSample, NEED TO BE LAST ENTRY
        };

        /// @brief The extracted data from a single time instance of the SensAItion sensor.
        struct MeasurementValue
        {
            using DataType = std::variant<double, uint32_t, int32_t>;

            // Empty constructor
            MeasurementValue()
                :
                value(static_cast<double>(0.0)),
                name(""),
                unit(""),
                nDec(0),
                nBytes(0),
                isInferred(false),
                isUpdated(false)
            {}

            // Value is a double
            MeasurementValue(double measurementValue, std::string valueName, std::string valueUnit = "",
                            int numberOfDecimals = 6, bool isInferredValue = false, bool isRecentlyUpdated = true)
                :
                value(measurementValue),
                name(valueName),
                unit(valueUnit),
                nDec(numberOfDecimals),
                nBytes(0),
                isInferred(isInferredValue),
                isUpdated(isRecentlyUpdated)
            {}

            // Value is an uint32_t, potential bitfield
            MeasurementValue(uint32_t measurementValue, std::string valueName, std::string valueUnit = "",
                            int numberBytes = 0, bool isInferredValue = false, bool isRecentlyUpdated = true)
                :
                value(measurementValue),
                name(valueName),
                unit(valueUnit),
                nDec(0),
                nBytes(numberBytes),
                isInferred(isInferredValue),
                isUpdated(isRecentlyUpdated)
            {}

            // value is an int32_t
            MeasurementValue(int32_t measurementValue, std::string valueName, std::string valueUnit = "",
                            bool isInferredValue = false, bool isRecentlyUpdated = true)
                :
                value(measurementValue),
                name(valueName),
                unit(valueUnit),
                nDec(0),
                nBytes(0),
                isInferred(isInferredValue),
                isUpdated(isRecentlyUpdated)
            {}

            DataType value;
            std::string name; // Short name, used in the CSV file header
            std::string unit; // unit of this value
            int nDec; // number of decimals of precision for floating point data
            int nBytes; // number of relevant bytes, if 0 assumes raw integer if > 0 assumes bitfield with hex representation
            bool isInferred; // If this data has been calculated from other data sources
            bool isUpdated; // true if this is updated in this sample, false if it hasn't changed since last update

            // Convert the value to string, not including the name
            std::string toString() const;

            // Converts the value to a formatted string suitable for displaying in data fields
            std::string toFormattedString() const;

            /// @brief Gets the data value always as a double
            double toValueData() const noexcept;

            /// @brief Gets the measurement value in double format
            /// @throws std::bad_variant_access if value is not a double
            double toDouble() const;

            /// @brief Gets the measurement value in uint32_t format
            /// @throws std::bad_variant_access if value is not a uint32_t
            uint32_t toUint32() const;

            /// @brief Gets the measurement value in int32_t format
            /// @throws std::bad_variant_access if value is not a int32_t
            int32_t toInt32() const;
        };

        /// Get the measurement value with the given type, throws if not available
        MeasurementValue at(const MeasurementType type) const;

        /// Add/update measurement value with given type, throws if type is INVALID
        void set(const MeasurementType type, MeasurementValue value);

        /// True if the sample contains this measurement type
        bool contains(const MeasurementType type) const { return measurements.find(type) != measurements.end(); }

        /// True if no samples are available
        bool empty(void) const {  return measurements.empty(); }

        // Returns number of samples available
        std::size_t size(void) const { return measurements.size(); }

        /// Get all measurement types available in this sample,
        /// sorted as in the MeasurementType enum definition.
        std::vector<MeasurementType> availableTypes() const;

        /// Return UTC datetime as a string formatted according to ISO 8601
        /// or an empty string if the sample does not contain UTC time.
        std::string getUtcString() const;

        // Exposed iterators, can loop over these with: for (const auto& [type, value] : sample)
        std::map<MeasurementType, MeasurementValue>::const_iterator begin() const noexcept { return measurements.cbegin(); }
        std::map<MeasurementType, MeasurementValue>::const_iterator end() const noexcept { return measurements.cend(); }

    private:

        // All available measurements from the SensAItion sensor at a single time instance
        // (Typically only contains a subset of all measurement types.)
        std::map<MeasurementType, MeasurementValue> measurements;
    };

} // namespace
#endif // SENSOR_SAMPLE_H
