// Silence warning that seems to come from Boost unit test framework
#pragma GCC diagnostic ignored "-Wcast-function-type"

#include <boost/test/unit_test.hpp>

#include <exception>
#include <iostream>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include "sensaition_parser/sensor_sample.hpp"

using namespace sepa;

BOOST_AUTO_TEST_SUITE(SensorSampleTestSuite);

BOOST_AUTO_TEST_CASE(ThrowsWhenAccessingNonexistentMeasurementType)
{
    SensorSample sample;
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::ACCELERATION_X), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(CanSetAndAccessMeasurementValue)
{
    SensorSample sample;
    const SensorSample::MeasurementValue val(static_cast<double>(7.2), "acc_x", "", 5, false);
    sample.set(SensorSample::MeasurementType::ACCELERATION_X, val);

    SensorSample::MeasurementValue fetchedVal = sample.at(SensorSample::MeasurementType::ACCELERATION_X);
    BOOST_CHECK(std::abs(fetchedVal.toDouble() - val.toDouble()) < 1e-6);
    BOOST_CHECK_EQUAL(fetchedVal.nDec, val.nDec);
    BOOST_CHECK_EQUAL(fetchedVal.isInferred, val.isInferred);
    BOOST_CHECK_EQUAL(fetchedVal.name, val.name);
}

BOOST_AUTO_TEST_CASE(ThrowsWhenSettingInvalidMeasurementValue)
{
    SensorSample sample;
    const SensorSample::MeasurementValue val(static_cast<double>(7.2), "generic_test_value");

    BOOST_CHECK_THROW(sample.set(SensorSample::MeasurementType::INVALID, val), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(ContainsFunctionWorksCorrectly)
{
    SensorSample sample;
    const SensorSample::MeasurementValue val(static_cast<double>(7.2), "generic_test_value");
    sample.set(SensorSample::MeasurementType::ACCELERATION_X, val);
    sample.set(SensorSample::MeasurementType::ACCELERATION_Y, val);
    sample.set(SensorSample::MeasurementType::STD_HEADING, val);

    SensorSample::MeasurementValue fetchedVal = sample.at(SensorSample::MeasurementType::ACCELERATION_X);
    BOOST_CHECK(sample.contains(SensorSample::MeasurementType::ACCELERATION_X));
    BOOST_CHECK(sample.contains(SensorSample::MeasurementType::ACCELERATION_Y));
    BOOST_CHECK(sample.contains(SensorSample::MeasurementType::STD_HEADING));

    BOOST_CHECK(!sample.contains(SensorSample::MeasurementType::ACCELERATION_Z));
    BOOST_CHECK(!sample.contains(SensorSample::MeasurementType::INVALID));
}

BOOST_AUTO_TEST_CASE(ProducesCorrectListOfAvailableTypes)
{
    SensorSample sample;
    using MV = SensorSample::MeasurementValue;
    const SensorSample::MeasurementValue val(static_cast<double>(7.2), "generic_test_value");

    BOOST_CHECK(sample.empty());

    // Add samples in OTHER order than the MeasurementType enum list
    sample.set(SensorSample::MeasurementType::STD_HEADING, MV(static_cast<double>(1.23), "std_heading"));
    sample.set(SensorSample::MeasurementType::ACCELERATION_X, MV(static_cast<double>(4.56), "acc_x"));
    sample.set(SensorSample::MeasurementType::ACCELERATION_Y, MV(static_cast<double>(7.89), "acc_y"));

    const auto types = sample.availableTypes();
    BOOST_REQUIRE_EQUAL(types.size(), static_cast<std::size_t>(3));

    // The list should be ordered according to the MeasurementType enum list
    // (Cannot use BOOST_CHECK_EQUAL, because the enum is not printable.)
    BOOST_CHECK(types[0] == SensorSample::MeasurementType::ACCELERATION_X);
    BOOST_CHECK(types[1] == SensorSample::MeasurementType::ACCELERATION_Y);
    BOOST_CHECK(types[2] == SensorSample::MeasurementType::STD_HEADING);

    // Test iterator
    BOOST_REQUIRE_EQUAL(sample.size(), static_cast<std::size_t>(3));
    int count = 0;
    for (const auto& [type, value] : sample)
    {
        if (count == 0)
        {
            BOOST_CHECK_EQUAL(type, SensorSample::MeasurementType::ACCELERATION_X);
            BOOST_CHECK_EQUAL(value.name, "acc_x");
        }
        else if (count == 1)
        {
            BOOST_CHECK_EQUAL(type, SensorSample::MeasurementType::ACCELERATION_Y);
            BOOST_CHECK_EQUAL(value.name, "acc_y");
        }
        else if (count == 2)
        {
            BOOST_CHECK_EQUAL(type, SensorSample::MeasurementType::STD_HEADING);
            BOOST_CHECK_EQUAL(value.name, "std_heading");
        }
        else
        {
            BOOST_CHECK(false);
        }
        count++;
    }
    BOOST_CHECK_EQUAL(count, 3);
}

BOOST_AUTO_TEST_CASE(CanHandleEmptyListOfAvailableTypes)
{
    SensorSample sample;

    const auto types = sample.availableTypes();
    BOOST_CHECK(types.empty());
    BOOST_CHECK(!sample.contains(SensorSample::MeasurementType::ACCELERATION_Z));
}

BOOST_AUTO_TEST_CASE(ProducesCorrectUtcString)
{
    SensorSample sample;
    using MV = SensorSample::MeasurementValue;

    sample.set(SensorSample::MeasurementType::UTC_YEAR, MV(static_cast<uint32_t>(2025), "utc_year"));
    sample.set(SensorSample::MeasurementType::UTC_MONTH, MV(static_cast<uint32_t>(11), "utc_month"));
    sample.set(SensorSample::MeasurementType::UTC_DAY, MV(static_cast<uint32_t>(20), "utc_day"));
    sample.set(SensorSample::MeasurementType::UTC_HOUR, MV(static_cast<uint32_t>(10), "utc_hour"));
    sample.set(SensorSample::MeasurementType::UTC_MINUTE, MV(static_cast<uint32_t>(55), "utc_minute"));
    sample.set(SensorSample::MeasurementType::UTC_SECOND, MV(static_cast<uint32_t>(57), "utc_second"));

    BOOST_CHECK_EQUAL(sample.getUtcString(), "2025-11-20T10:55:57Z");

    sample.set(SensorSample::MeasurementType::UTC_US, MV(static_cast<uint32_t>(1234), "utc_us"));

    BOOST_CHECK_EQUAL(sample.getUtcString(), "2025-11-20T10:55:57.001234Z");
}

BOOST_AUTO_TEST_CASE(BadVariantAccesses)
{
    SensorSample sample;
    using MV = SensorSample::MeasurementValue;

    sample.set(SensorSample::MeasurementType::TICK, MV(static_cast<uint32_t>(12345), "tick"));
    sample.set(SensorSample::MeasurementType::PERF_TICK, MV(static_cast<int32_t>(12345), "perf_tick")); // not actually true but...
    sample.set(SensorSample::MeasurementType::ROLL, MV(static_cast<double>(12.3456), "roll"));

    BOOST_CHECK_NO_THROW(sample.at(SensorSample::MeasurementType::TICK).toUint32());
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::TICK).toInt32(), std::bad_variant_access);
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::TICK).toDouble(), std::bad_variant_access);

    BOOST_CHECK_NO_THROW(sample.at(SensorSample::MeasurementType::PERF_TICK).toInt32());
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::PERF_TICK).toUint32(), std::bad_variant_access);
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::PERF_TICK).toDouble(), std::bad_variant_access);

    BOOST_CHECK_NO_THROW(sample.at(SensorSample::MeasurementType::ROLL).toDouble());
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::ROLL).toInt32(), std::bad_variant_access);
    BOOST_CHECK_THROW(sample.at(SensorSample::MeasurementType::ROLL).toUint32(), std::bad_variant_access);
}

BOOST_AUTO_TEST_SUITE_END();
