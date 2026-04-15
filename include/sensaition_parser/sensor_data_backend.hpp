#ifndef SENSOR_DATA_BACKEND_H
#define SENSOR_DATA_BACKEND_H

#include <stdint.h>
#include <array>
#include <vector>
#include <bitset>
#include <chrono>
#include <functional>

#include "sensaition_parser/sensor_types.hpp"
#include "sensaition_parser/sensor_sample.hpp"

namespace sepa
{
    /// @brief Class for parsing various sensor data values
    class SensorDataBackend
    {
    public:
        SensorDataBackend();
        ~SensorDataBackend();

        // DATA ACCESS =====================================================================================================

        /// @brief Check if there exists a value at the index location.
        ///        This determines if ALL following value getters are valid.
        bool getValueExists(std::size_t idx) const noexcept { return idx < values.size() && values[idx] != nullptr; }

        /// @brief Get the name of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        std::string getValueName(std::size_t idx) const { return values.at(idx)->name; }

        /// @brief Get the name and unit of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        std::string getValueNameAndUnit(std::size_t idx) const
        {
            auto value = values.at(idx);
            return value->unit.empty() ? value->name : value->name + " [" + value->unit + "]";
        }

        /// @brief Get the tooltip information of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        std::string getValueTooltip(std::size_t idx) const { return values.at(idx)->tooltip; }

        /// @brief Get the MeasurementType enumeration of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        SensorSample::MeasurementType getValueMeasurementType(std::size_t idx) const { return values.at(idx)->measurementType; }

        /// @brief Get the supported hardware model of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        SensaitionHardwareModel getValueHwModel(std::size_t idx) const { return values.at(idx)->hwModel; }

        /// @brief Gets the minimum valid sw version which supports this data field, or 0 if always valid
        /// @param idx A valid value index as determined by getValueExists(idx)
        uint16_t getValueMinimumSwVersion(std::size_t idx) const { return values.at(idx)->minVersion; }

        /// @brief Get the data value represented as a double of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        double getValueData(std::size_t idx) const { return values.at(idx)->getData(); }

        /// @brief Get the formatted data display string of the value at the index location
        /// @param idx A valid value index as determined by getValueExists(idx)
        std::string getValueFormattedData(std::size_t idx) const { return values.at(idx)->getFormattedData(); }

        /// @brief Gets the size of the data entry, as in the number of relevant bytes
        /// @param idx A valid value index as determined by getValueExists(idx)
        int getValueSize(std::size_t idx) const { return values.at(idx)->getSize(); }

        /// @brief Check if the value at the index location has been updated in the most recent data parsing
        /// @param idx A valid value index as determined by getValueExists(idx)
        bool isValueUpdated(std::size_t idx) const { return values.at(idx) != nullptr && values.at(idx)->parsed; }

        /// @brief Checks if the data value at the index location has recently been updated
        /// @param idx A valid value index as determined by getValueExists(idx)
        /// @param data_only if true, only checks the data channels for recent values, if false also includes the user uart channel
        bool isValueRecent(std::size_t idx, bool data_channels_only = false) const { return values.at(idx)->isRecent(data_channels_only); }

        /// @brief Gets a list of SensorDataValueId's ordered to follow the ordering in SensorSample::MeasurementType.
        /// Used to get a more intuitive grouping of related sensor values with priority ordering that's used by the
        /// the csv logs and generated config strings.
        const std::array<std::size_t, SensorDataValueId::SENSOR_DATA_NUM> &getValuesSorted() const { return values_priority; }

        // CONFIG STRING GENERATORS ========================================================================================

        std::string getUserUartConfigString(const DataSelection &selected, const StandardNmeaSelection &nmeaSelection) const;
        std::string getUserUartConfigString(const DataSelection &selected) const;

        std::string getDataUartConfigString(const DataUartMessage &message) const;
        std::string getDataUartConfigString(const DataUartMessageSet &messages) const;
        std::string getDataUartConfigString(const DataSelection &selected, int rateDivisor, int checksumLength = 1, bool useBigEndian = true, bool compact = false) const;

        std::string getCanConfigString(const std::vector<CanMessage> &messages) const;

        // CONFIG STRING PARSERS ===========================================================================================

        DataSelection parseUserUartConfigString(const std::string &str, StandardNmeaSelection &nmeaSelection) const;
        DataSelection parseUserUartConfigString(const std::string &str) const;

        DataSelection parseDataUartConfigString(const std::string &str, DataUartParseInfo &parseInfo, DataUartMessageSet &selection) const;
        DataSelection parseDataUartConfigString(const std::string &str, DataUartParseInfo &parseInfo) const;

        DataSelection parseCanConfigString(const std::string &str, std::vector<CanParseInfo> &parseInfo, std::vector<CanMessage> &selection) const;
        DataSelection parseCanConfigString(const std::string &str) const;

        // RAW DATA PARSERS ================================================================================================

        bool parseUserUartData(const std::string &str);
        bool parseDataUartData(const std::vector<uint8_t> &data, const DataUartParseInfo &parseInfo);

        bool validateDataUartData(const std::vector<uint8_t> &data, const DataUartParseInfo &parseInfo) const;

        // SENSOR SAMPLE GETTERS ===========================================================================================

        SensorSample getSensorSample(void);
        SensorSample getSensorSample(const DataSelection selection);

    private:
        /// @brief Container class for each sensor data value, one for each index in the sensor data array
        class SensorDataValue
        {
        public:
            /// @brief Format of how the data should be presented after conversion from raw int32, size format is relevant bytes in data
            enum DataType
            {
                FLOAT,     // Data is a float scaled with a scalefactor represented as an int32
                FLOAT16,   // Data is a float scaled with a scalefactor represented as an int16
                INT32,     // Data is an 32-bit integer
                INT16,     // Data is an 16-bit integer
                INT8,      // Data is an 8-bit integer
                UINT32,    // Data is an 32-bit unsigned integer
                UINT16,    // Data is an 16-bit unsigned integer
                UINT8,     // Data is an 8-bit unsigned integer
                FLAGS32,   // Data is an 32-bit bitfield
                FLAGS16,   // Data is an 16-bit bitfield
                FLAGS8,    // Data is an 8-bit bitfield
                UINT8x4,   // Data is a vector of 4 8-bit unsigned integers
                UINT16x2,  // Data is a vector of 2 16-bit unsigned integers
                UINT8x2,   // Data is a vector of 2 8-bit unsigned integers
                FLAGS16x2, // Data is a vector of 2 16-bit bitfields
            };

            SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                            SensaitionHardwareModel hwModel, std::string tooltip = "", uint16_t minVersion = 0);
            SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                            double ScaleFactor, SensaitionHardwareModel hwModel, std::string tooltip = "",
                            uint16_t minVersion = 0);
            SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                            std::function<double(int)> convert, SensaitionHardwareModel hwModel, std::string tooltip = "",
                            uint16_t minVersion = 0);
            ~SensorDataValue();

            void convertFromRaw(int32_t raw);
            int getSize(void) const;
            uint32_t getByteMask(void) const;
            double getData(void) const;
            std::string getFormattedData(void) const;

            /// @brief Insert measurements into the given SensorSample
            void insertMeasurementValue(SensorSample *sample) const;

            void timestampData(void);
            void timestampUser(void);
            bool isRecent(bool data_channels_only = false) const;

            std::string name;                              // Displayed name of this data entry
            std::string unit;                              // Displayed unit for this data entry
            std::string tooltip;                           // Tooltip to display when hovering over this unit
            DataType type;                                 // Type of data after conversion from raw int
            SensorSample::MeasurementType measurementType; // Used for packing values into a SensorSample
            SensaitionHardwareModel hwModel;               // Supported hardware model
            uint16_t minVersion;                           // Minimum fw version which supports this data

            int nDec;                           // Number of relevant decimals for FLOAT data
            double scaleFactor;                 // Scale factor for converting raw data -> data
            std::function<double(int)> convert; // Custom conversion from raw -> data

            union
            {
                double dfloat;
                int32_t dint;
                uint32_t duint;
                uint16_t duint16x2[2];
                uint8_t duint8x4[4];
            } data; // Different data representation depending on the datatype, todo: std::variant?

            // Parsed data handler
            bool parsed;   // true if this data entry was parsed in the latest data stream
            bool inferred; // true if this data entry was inferred from other data in the latest data stream
            std::array<uint8_t, 4> parseData;

        private:
            void init(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                      SensaitionHardwareModel hwModel, std::string tooltip, uint16_t minVersion);

            static const int64_t RECENT_TIMEOUT = 1100;                   // how old the data can get to make us consider it "recent", in ms
            std::chrono::time_point<std::chrono::steady_clock> last_data; // timestamp of last received data from the data stream
            std::chrono::time_point<std::chrono::steady_clock> last_user; // timestamp of the last received data for the user uart stream
        };

        void populateSensorData(void);
        void calculateInferredData(void);

        uint8_t char_to_hex(const char c) const;
        bool found_message_identifier(const std::vector<uint8_t> &data, const DataUartParseInfo::DataUartParsedMessage *message, int numMessages) const;

        // MEMBER VARIABLES ================================================================================================

        ///@brief Mapping msg_id -> identifier, must match the order in StandardNmeaMessages
        const std::array<const char *, StandardNmeaMessages::STANDARD_NMEA_NUM> nmea_identifiers = {"GGA", "RMC", "GLL", "GST"};

        // Main storage of all individual sensor data values
        std::array<SensorDataValue *, SensorDataValueId::SENSOR_DATA_NUM> values;

        // Sorted values list based their corresponding SensorSample::MeasurementType
        std::array<std::size_t, SensorDataValueId::SENSOR_DATA_NUM> values_priority;
    };

} // namespace
#endif // SENSOR_DATA_BACKEND_H
