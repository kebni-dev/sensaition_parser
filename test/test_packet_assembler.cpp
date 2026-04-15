#include <boost/test/unit_test.hpp>
#include <iostream>
#include "sensaition_parser/packet_assembler.hpp"
#include "sensaition_parser/sensor_data_backend.hpp"

//
// Tests packing of data-uart bytes into messages
//
// build/test/test_parser2.exe
// build/test/test_parser2.exe --gtest_list_tests
// build/test/test_parser2.exe --gtest_filter=PacketAssemblerTest.*
//

using namespace sepa;

namespace
{
    // Type used for both uart input and assembled packet
    using Packet = std::vector<std::uint8_t>;

    // Wraps the tested class PacketAssembler
    class PacketAssemblerTester: public PacketProcessor
    {
    public:
        PacketAssemblerTester() { packetAssembler.setListener(this); }

        void processPacket(std::vector<std::uint8_t>& data) override {
            ++callbackCount;
            lastPacket.assign(data.begin(), data.end());
        }

        void testProcessBytes(const Packet& uartInput) {
            size_t uartIndex = 0;
            uint8_t header = uartInput.at(uartIndex++);
            packetAssembler.inputBuffer()[0] = header;
            int maxLoopCount = 8;
            size_t parseSize = packetAssembler.processBytes(parseInfo, 1); // process header
            while (maxLoopCount-- > 0 && uartIndex + parseSize <= uartInput.size()) {
                // Copy as many bytes as requested by packetAssembler
                uint8_t* parseBuffer = packetAssembler.inputBuffer();
                for (std::size_t i = 0; i < parseSize; ++i) {
                    parseBuffer[i] = uartInput.at(uartIndex++);
                }
                // Process copied bytes
                parseSize = packetAssembler.processBytes(parseInfo, parseSize);
            }
        }

        void parseConfig(const char* config) {
            backend.parseDataUartConfigString(config, parseInfo, selection);

            // Compare DataUartHandler::updateParser and readCompletionHandler on updatePending_ 
            std::size_t max_size = 0;
            for (int i = 0; i < parseInfo.numMessages; i++)
                max_size = std::max(max_size,
                    static_cast<std::size_t>(parseInfo.messages[i].numIdentifier +
                                                parseInfo.messages[i].numBytes +
                                                parseInfo.messages[i].checksumBytes));
            packetAssembler.resetBuffer(parseInfo);
        }

        bool verifyLastPacket(const Packet& uartInput) {
            return lastPacket == uartInput && lastPacket.size() > 0;
        }

        bool isUartInputValid(const Packet& data) {
            return backend.validateDataUartData(data, parseInfo);
        }

        uint8_t getChecksumSize() { return parseInfo.messages[0].checksumBytes; }

        int callbackCount { 0 };
        
    private:
        PacketAssembler packetAssembler; // unit under test
        SensorDataBackend backend; // for parsing config string
        DataUartMessageSet selection; // intermediate states
        DataUartParseInfo parseInfo; // intermediate states
        Packet lastPacket; // output from unit under test (for verification)
    };

    const char* checksumConfiguration1 = "o000Ar0600F01F02F03F04F05Fx";
    const char* crcConfiguration1 = "o000Ar0600F01F02F03F04F05FX";

    // Header (1) + Data (24) + Checksum (1) = total 26 bytes
    const Packet checksumInputDataValid1 { 0xFA,
        0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20,
        0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35,
        0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x05 };

    // First data byte differs from checksumInputDataValid1
    const Packet checksumInputDataInvalid1 { 0xFA,
        0x06, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20,
        0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35,
        0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x05 };

    // Header (1) + Data (24) + CRC (2) = total 27 bytes
    const Packet crcInputDataValid1 { 0xFA,
        0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20,
        0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35,
        0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97, 0x86 };

    // Last data byte differs from crcInputDataValid1
    const Packet crcInputDataInvalid1 { 0xFA,
        0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20,
        0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35,
        0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x06, 0x97, 0x86 };

} // namespace

BOOST_AUTO_TEST_SUITE(PacketAssemblerTest)

BOOST_AUTO_TEST_CASE(VerifyChecksumInputDataValid1) {
    PacketAssemblerTester tester;
    tester.parseConfig(checksumConfiguration1);
    const Packet& uartInput = checksumInputDataValid1;
    BOOST_REQUIRE_MESSAGE(tester.getChecksumSize() == 1, "Prerequisite checkum size from parseDataUartConfigString");
    BOOST_REQUIRE_MESSAGE(tester.isUartInputValid(uartInput), "Prerequisite input-data validity (checksumInputDataValid1)");

    tester.testProcessBytes(uartInput);
    BOOST_CHECK_MESSAGE(tester.callbackCount == 1, "Checking number of processPacket callbacks from PacketAssembler");
    BOOST_CHECK_MESSAGE(tester.verifyLastPacket(uartInput), "Matching packed message against the byte-data input");
}

BOOST_AUTO_TEST_CASE(VerifyCrcInputDataValid1) {
    PacketAssemblerTester tester;
    tester.parseConfig(crcConfiguration1);
    const Packet& uartInput = crcInputDataValid1;
    BOOST_REQUIRE_MESSAGE(tester.getChecksumSize() == 2, "Prerequisite CRC size from parseDataUartConfigString");
    BOOST_REQUIRE_MESSAGE(tester.isUartInputValid(uartInput), "Prerequisite input-data validity (crcInputDataValid1)");

    tester.testProcessBytes(uartInput);
    BOOST_CHECK_MESSAGE(tester.callbackCount == 1, "Checking number of processPacket callbacks from PacketAssembler");
    BOOST_CHECK_MESSAGE(tester.verifyLastPacket(uartInput), "Matching packed message against the byte-data input");
}

BOOST_AUTO_TEST_CASE(VerifyChecksumInputDataInvalid1) {
    PacketAssemblerTester tester;
    tester.parseConfig(checksumConfiguration1);
    const Packet& uartInput = checksumInputDataInvalid1;
    BOOST_REQUIRE_MESSAGE(tester.getChecksumSize() == 1, "Prerequisite checkum size from parseDataUartConfigString");
    BOOST_REQUIRE_MESSAGE(!tester.isUartInputValid(uartInput), "Prerequisite input-data validity (checksumInputDataValid1)");

    tester.testProcessBytes(uartInput);
    BOOST_CHECK_MESSAGE(tester.callbackCount == 0, "Checking number of processPacket callbacks from PacketAssembler");
    BOOST_CHECK_MESSAGE(!tester.verifyLastPacket(uartInput), "Matching packed message against the byte-data input");
}

BOOST_AUTO_TEST_CASE(VerifyCrcInputDataInvalid1) {
    PacketAssemblerTester tester;
    tester.parseConfig(crcConfiguration1);
    const Packet& uartInput = crcInputDataInvalid1;
    BOOST_REQUIRE_MESSAGE(tester.getChecksumSize() == 2, "Prerequisite CRC size from parseDataUartConfigString");
    BOOST_REQUIRE_MESSAGE(!tester.isUartInputValid(uartInput), "Prerequisite input-data validity (crcInputDataValid1)");

    tester.testProcessBytes(uartInput);
    BOOST_CHECK_MESSAGE(tester.callbackCount == 0, "Checking number of processPacket callbacks from PacketAssembler");
    BOOST_CHECK_MESSAGE(!tester.verifyLastPacket(uartInput), "Matching packed message against the byte-data input");
}

BOOST_AUTO_TEST_SUITE_END()
