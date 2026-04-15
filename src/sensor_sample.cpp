
#include "sensaition_parser/sensor_sample.hpp"

#include <exception>
#include <iomanip>

using namespace sepa;

std::string SensorSample::MeasurementValue::toString() const
{
    return std::visit([this](auto&& arg) -> std::string
    {
        using T = std::decay_t<decltype(arg)>;
        std::ostringstream stream;

        if constexpr (std::is_same_v<T, double>)
        {
            stream << std::fixed << std::setprecision(nDec) << arg;
            return stream.str();
        } 
        else if constexpr (std::is_same_v<T, uint32_t> || std::is_same_v<T, int32_t>)
        {
            return std::to_string(arg);
        } 
        else
        {
            return "";
        }
    }, value);
}

std::string SensorSample::MeasurementValue::toFormattedString() const
{
    return std::visit([this](auto&& arg) -> std::string
    {
        using T = std::decay_t<decltype(arg)>;
        std::ostringstream stream;

        if constexpr (std::is_same_v<T, double>)
        {
            stream << std::fixed << std::setprecision(nDec) << arg;
        } 
        else if constexpr (std::is_same_v<T, uint32_t>)
        {
            if (nBytes > 0) // bitfield
                stream << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2*nBytes) << arg;
            else // normal int
                stream << std::dec << arg;
        } 
        else if constexpr (std::is_same_v<T, int32_t>)
        {
            stream << std::dec << arg; // always normal int
        }

        if (!unit.empty()) stream << " " << unit; // append unit
        return stream.str();
    }, value);
}

double SensorSample::MeasurementValue::toValueData() const noexcept
{
    return std::visit([this](auto&& arg) -> double
    {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, double>)
            return arg; 
        else if constexpr (std::is_same_v<T, uint32_t>)
            return static_cast<double>(arg);
        else if constexpr (std::is_same_v<T, int32_t>)
            return static_cast<double>(arg);
        return 0.0;
    }, value);
}

double SensorSample::MeasurementValue::toDouble() const
{
    return std::get<double>(value);
}

uint32_t SensorSample::MeasurementValue::toUint32() const
{
    return std::get<uint32_t>(value);
}

int32_t SensorSample::MeasurementValue::toInt32() const
{
    return std::get<int32_t>(value);
}

SensorSample::MeasurementValue SensorSample::at(const MeasurementType type) const
{
    return measurements.at(type);
}

void SensorSample::set(const SensorSample::MeasurementType type, SensorSample::MeasurementValue value)
{
    if (type == MeasurementType::INVALID)
    {
        throw(std::invalid_argument("MeasurementType::INVALID should not be accessed as a normal value!"));
    }
    if (contains(type))
    {
        measurements.at(type) = value;
    }
    else
    {
        // Using emplace removes the need for a default MeasurementValue constructor
        measurements.emplace(type, value);
    }
}

std::vector<SensorSample::MeasurementType> SensorSample::availableTypes() const
{
    std::vector<MeasurementType> types;
    types.reserve(measurements.size());
    for (const auto& [key, _] : measurements) {
        types.push_back(key);
    }
    return types;
}

std::string SensorSample::getUtcString() const
{
    const bool hasAllValues = contains(MeasurementType::UTC_YEAR)
        && contains(MeasurementType::UTC_MONTH)
        && contains(MeasurementType::UTC_DAY)
        && contains(MeasurementType::UTC_HOUR)
        && contains(MeasurementType::UTC_MINUTE)
        && contains(MeasurementType::UTC_SECOND);
    if (!hasAllValues)
    {
        return "";
    }

    try
    {
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(4) << at(MeasurementType::UTC_YEAR).toUint32() << '-'
            << std::setw(2) << at(MeasurementType::UTC_MONTH).toUint32() << '-'
            << std::setw(2) << at(MeasurementType::UTC_DAY).toUint32() << 'T'
            << std::setw(2) << at(MeasurementType::UTC_HOUR).toUint32() << ':'
            << std::setw(2) << at(MeasurementType::UTC_MINUTE).toUint32() << ':'
            << std::setw(2) << at(MeasurementType::UTC_SECOND).toUint32();

        if (contains(MeasurementType::UTC_US))
        {
            oss << "." << std::setw(6) << at(MeasurementType::UTC_US).toUint32();
        }

        oss << 'Z';
        return oss.str();
    }
    catch (const std::out_of_range& ex)
    {
        (void) ex; // Avoid complaints for unused variable
        return "";
    }
}
