// Silence warning that seems to come from Boost unit test framework
#pragma GCC diagnostic ignored "-Wcast-function-type"

#include <boost/test/unit_test.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <iostream>
#include <sstream>
#include "sensaition_parser/sensor_data_backend.hpp"

using namespace sepa;

namespace {
using MT = SensorSample::MeasurementType;
}

BOOST_AUTO_TEST_SUITE(SensorDataBackendTests);

BOOST_AUTO_TEST_CASE(SensorDataIndexCheck)
{
  SensorDataBackend backend;
  BOOST_CHECK(backend.getValueExists(0x00));
  BOOST_CHECK(backend.getValueExists(0x01));
  BOOST_CHECK(backend.getValueExists(0x02));
  BOOST_CHECK(!backend.getValueExists(0xFF));
  BOOST_CHECK(backend.getValueExists(SensorDataValueId::NAV_TIME)); // Last entry

#if 0
  int tot_size = 0;
  for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
  {
    if (backend.getValueExists(idx) && backend.getValueHwModel(idx) <= SensaitionHardwareModel::INS)
      tot_size += backend.getValueSize(idx);
  }
  BOOST_TEST_MESSAGE("Total size: " << tot_size << " bytes");
#endif

}

BOOST_AUTO_TEST_CASE(DataUartGenerateAndParse)
{
  // Randomly generate a message selection, generate a config string and read it back
  // Should get the same selection back
  SensorDataBackend backend;
  DataUartParseInfo parseInfo;
  DataUartMessageSet message, messageReadback;

  boost::random::mt19937 gen(123);
  boost::random::uniform_int_distribution<> rateNumMessages(1,3);
  boost::random::uniform_int_distribution<> rateNumMessagesCompact(1,8);
  boost::random::uniform_int_distribution<> rateDivDist(1,1000);
  boost::random::uniform_int_distribution<> rateChecksumDist(0, 2);
  boost::random::uniform_int_distribution<> rateBoolDist(0, 1);
  boost::random::uniform_int_distribution<> rateSelection(0, 7);
  boost::random::uniform_int_distribution<> rateNumIdentifier(0, 4);
  boost::random::uniform_int_distribution<> rateIdentifier(0, 255);
  boost::random::uniform_int_distribution<> rateIdentifierAcc(1, 31);
  boost::random::uniform_int_distribution<> rateIdxOffset(0, 127);

  std::size_t cfg_string_maxlen = 0;

  for (int test_idx = 0; test_idx < 500; test_idx++)
  {
    // GENERATE SELECTION ====================================================

    // Big endian and compact mode are shared for all messages
    bool useBigEndian = rateBoolDist(gen) == 0;
    bool useCompact = rateBoolDist(gen) == 0;

    // Random number of messages
    // Limit max nbr of message if not using compact formatting
    message.numMessages = useCompact ? rateNumMessagesCompact(gen) : rateNumMessages(gen);

    // Ensure all messages has the same number of identifiers and each are unique
    int numIdentifiers;
    do {
      numIdentifiers = rateNumIdentifier(gen);
    } while (numIdentifiers == 0 && message.numMessages > 1);

    uint32_t identifier = 0;
    uint32_t identifier_mask = 0;
    for (int i = 0; i < numIdentifiers; i++)
    {
      identifier = (identifier << 8) | rateIdentifier(gen);
      identifier_mask = (identifier_mask << 8) | 0xFF;
    }

    // Make sure we don't overdo string size....
    const int MSG_OVERHEAD = message.numMessages * (5 + 3 * numIdentifiers + 3 + 1); // total overhead...
    const int MSG_CHAR_PER_ENTRY = (useCompact ? 3 : 12);
    const int MSG_MAX_SELECTIONS = (255 - MSG_OVERHEAD) / MSG_CHAR_PER_ENTRY;

    int sel_num = 0;
    for (int msg_idx = 0; msg_idx < message.numMessages; msg_idx++)
    {
      message.message[msg_idx].reset();
      message.message[msg_idx].dataRate = rateDivDist(gen);
      message.message[msg_idx].checksumBytes = rateChecksumDist(gen);

      message.message[msg_idx].useBigEndian = useBigEndian;
      message.message[msg_idx].useCompact = useCompact;

      // Ensure all messages has the same number of identifiers and each are unique
      message.message[msg_idx].numIdentifier = numIdentifiers;
      identifier += static_cast<uint32_t>(rateIdentifierAcc(gen));
      identifier &= identifier_mask;
      message.message[msg_idx].identifier = identifier;

      // Enable some random fields
      const int sel_max = (msg_idx + 1) * MSG_MAX_SELECTIONS / message.numMessages;
      const std::size_t idx_offset = rateIdxOffset(gen); // spread value indices
      for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
      {
        const std::size_t value_idx = (idx + idx_offset) % SensorDataValueId::SENSOR_DATA_NUM;
        if (backend.getValueExists(value_idx) && rateSelection(gen) == 0)
        {
          message.message[msg_idx].selection.set(value_idx);
          sel_num++;
          if (sel_num >= sel_max) break;
        }
      }
    }

    // GENERATE AND READBACK PARSE ===========================================
    std::string cfg_string = backend.getDataUartConfigString(message);
    BOOST_CHECK_LT(cfg_string.size(), static_cast<std::size_t>(256));
    backend.parseDataUartConfigString(cfg_string, parseInfo, messageReadback);

    // VERIFY READBACK =======================================================
    BOOST_REQUIRE_GT(message.numMessages, 0);
    BOOST_CHECK_EQUAL(message.numMessages, parseInfo.numMessages);
    BOOST_CHECK_EQUAL(message.numMessages, messageReadback.numMessages);
    for (int i = 0; i < message.numMessages; i++)
    {
      BOOST_CHECK_EQUAL(message.message[i].selection, messageReadback.message[i].selection);
      BOOST_CHECK_EQUAL(message.message[i].dataRate, messageReadback.message[i].dataRate);
      BOOST_CHECK_EQUAL(message.message[i].checksumBytes, messageReadback.message[i].checksumBytes);
      BOOST_CHECK_EQUAL(message.message[i].useBigEndian, messageReadback.message[i].useBigEndian);
      BOOST_CHECK_EQUAL(message.message[i].useCompact, messageReadback.message[i].useCompact);
      BOOST_CHECK_EQUAL(message.message[i].numIdentifier, messageReadback.message[i].numIdentifier);
      BOOST_CHECK_EQUAL(message.message[i].identifier, messageReadback.message[i].identifier);

      BOOST_CHECK_EQUAL(message.message[i].dataRate, parseInfo.messages[i].dataRate);
      BOOST_CHECK_EQUAL(message.message[i].checksumBytes, parseInfo.messages[i].checksumBytes);
      BOOST_CHECK_EQUAL(message.message[i].numIdentifier, parseInfo.messages[i].numIdentifier);
      BOOST_CHECK_EQUAL(message.message[i].identifier, parseInfo.messages[i].identifier);

      // Go through parsed bytes and verify all indices are in increasing measurementType order
      MT prevType = static_cast<MT>(0);
      for (int j = parseInfo.messages[i].startIndex; j < (parseInfo.messages[i].startIndex + parseInfo.messages[i].numBytes); j++)
      {
        const MT newType = backend.getValueMeasurementType(parseInfo.data[j].row);
        BOOST_CHECK_GE(newType, prevType);
        prevType = newType;
      }
    }
    if (cfg_string.size() > cfg_string_maxlen) cfg_string_maxlen = cfg_string.size();

    //BOOST_TEST_MESSAGE("sel_num: " << sel_num << ", len(str): " << cfg_string.length());
  }
  //BOOST_TEST_MESSAGE("Max config string length: " << cfg_string_maxlen);
}

BOOST_AUTO_TEST_CASE(CanGenerateAndParse)
{
  // Randomly generate a message selection, generate a config string and read it back
  // Should get the same selection back
  SensorDataBackend backend;
  std::vector<CanParseInfo> parseInfo;
  std::vector<CanMessage> message, messageReadback;

  boost::random::mt19937 gen(456);
  boost::random::uniform_int_distribution<> rateNumMessages(1,16);
  boost::random::uniform_int_distribution<> rateDivDist(1,1000);
  boost::random::uniform_int_distribution<> ratePhaseDist(1,255);
  boost::random::uniform_int_distribution<> rateBoolDist(0, 1);
  boost::random::uniform_int_distribution<> rateSwapDist(0, 3);
  boost::random::uniform_int_distribution<> rateIdentifierType(0, 2);
  boost::random::uniform_int_distribution<> rateIdentifier_t(0, 0x7FF);
  boost::random::uniform_int_distribution<> rateIdentifier_T(0, 0x1F);
  boost::random::uniform_int_distribution<> rateIdentifier_E(0, 0x1FFFFFFF);
  boost::random::uniform_int_distribution<> rateSelection(0, 127);

  for (int test_idx = 0; test_idx < 500; test_idx++)
  {
    // GENERATE SELECTION ====================================================
    int numMessage = rateNumMessages(gen);
    int dataRate = 0;
    int phase = 0;
    message.clear();
    for (int i = 0; i < numMessage; i++)
    {
      CanMessage msg;
      if (dataRate == 0 || rateSwapDist(gen) == 0)
      {
        dataRate = rateDivDist(gen);
        phase = 0;
      }
      else if (rateSwapDist(gen) == 0)
      {
        phase = ratePhaseDist(gen);
      }
      msg.dataRate = dataRate;
      msg.phase = phase;
      msg.idType = static_cast<CanIdentifierTypes>(rateIdentifierType(gen));
      if (msg.idType == CanIdentifierTypes::CAN_STANDARD_ID)
        msg.identifier = rateIdentifier_t(gen);
      else if (msg.idType == CanIdentifierTypes::CAN_EXTENDED_ID)
        msg.identifier = rateIdentifier_E(gen);
      else
        msg.identifier = rateIdentifier_T(gen);
      
      msg.useBigEndian = rateBoolDist(gen);

      // Select 1-2 unique entries (won't exceed 8-byte limit)
      msg.selection.reset();
      do
      {
        std::size_t idx;
        do
        {
          idx = rateSelection(gen);
        } while (!backend.getValueExists(idx) || msg.selection.test(idx));
        msg.selection.set(idx);
      } while (msg.selection.count() < ((rateBoolDist(gen) == 0) ? 2 : 1));

      message.push_back(msg);
    }

    // GENERATE AND READBACK PARSE ===========================================
    std::string cfg_string = backend.getCanConfigString(message);
    BOOST_CHECK_LT(cfg_string.size(), static_cast<std::size_t>(512));
    backend.parseCanConfigString(cfg_string, parseInfo, messageReadback);

    // VERIFY READBACK =======================================================
    BOOST_REQUIRE_GT(message.size(), static_cast<std::size_t>(0));
    BOOST_REQUIRE_EQUAL(message.size(), static_cast<std::size_t>(numMessage));
    BOOST_REQUIRE_EQUAL(message.size(), parseInfo.size());
    BOOST_REQUIRE_EQUAL(message.size(), messageReadback.size());
    for (int i = 0; i < numMessage; i++)
    {
      BOOST_CHECK_EQUAL(message[i].dataRate, messageReadback[i].dataRate);
      BOOST_CHECK_EQUAL(message[i].phase, messageReadback[i].phase);
      BOOST_CHECK_EQUAL(static_cast<int>(message[i].idType), static_cast<int>(messageReadback[i].idType));
      BOOST_CHECK_EQUAL(message[i].identifier, messageReadback[i].identifier);
      BOOST_CHECK_EQUAL(message[i].selection, messageReadback[i].selection);

      BOOST_CHECK_EQUAL(message[i].dataRate, parseInfo[i].dataRate);
      BOOST_CHECK_EQUAL(message[i].phase, parseInfo[i].phase);
      BOOST_CHECK_EQUAL(static_cast<int>(message[i].idType), static_cast<int>(parseInfo[i].idType));
      BOOST_CHECK_EQUAL(message[i].identifier, parseInfo[i].identifier);

      if (message[i].selection.count() != parseInfo[i].dataLength)
      {
        // if only single byte entries, cannot deduce endianness...
        BOOST_CHECK_EQUAL(message[i].useBigEndian, parseInfo[i].usingBigEndian);
        BOOST_CHECK_EQUAL(message[i].useBigEndian, messageReadback[i].useBigEndian);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(SensorDataInvalidStrings)
{
  SensorDataBackend backend;
  
  // Data UART
  DataUartParseInfo parseInfo;
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064s08003002001000013012011010", parseInfo)); // Valid string, no checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064s08003002001000013012011010x", parseInfo)); // Valid string, 8-bit checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064s08003002001000013012011010X", parseInfo)); // Valid string, 16-bit checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064s524234224214202F32F22F12F0300400313312311310323322321320333332331330343342341340353352351350363362361360373372371370383382381380393392391390613612611610623622621620633632631630643642641640653652651650663662661660673672671670683682681680693692691690x", parseInfo)); // Max length string
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064s08003002001000013012011010s08023022021020033032031030X", parseInfo)); // multiple switches
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("s08003002001000013012011010X", parseInfo)); // no initial 'o'-switch
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].dataRate, 10); // default data rate
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1); // Have a valid parse

  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064s01300+INVALID_STRING", parseInfo), std::invalid_argument); // Bad string
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 0); // A bad string always invalidates the parse-info
  
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08803802801800013012011010x", parseInfo), std::invalid_argument); // Token out of range
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08003002001000013012011010k", parseInfo), std::invalid_argument); // Unknown token
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0As08003002001000013012011010x", parseInfo), std::invalid_argument); // Invalid rate
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0000s08003002001000013012011010x", parseInfo), std::invalid_argument); // Data rate 0
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As0G003002001000013012011010x", parseInfo), std::invalid_argument); // Invalid data len
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08F03F02F01F00013012011010x", parseInfo), std::invalid_argument); // Invalid row
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08004003002001013012011010x", parseInfo), std::invalid_argument); // Invalid column
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As06003002001000013012011010x", parseInfo), std::invalid_argument); // Too short data len
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As09003002001000013012011010x", parseInfo), std::invalid_argument); // Too long data len
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As09003002001000013012011010", parseInfo), std::invalid_argument); // Too long data len 2
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000At1238003002001000013012011010t4568023022021020033032031030", parseInfo), std::invalid_argument); // Accidental CAN config
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08003002001000013012011010Xs08023022021020033032031030", parseInfo), std::invalid_argument); // checksum in middle of string
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08003002001000013012011010xx", parseInfo), std::invalid_argument); // multiple checksums
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08003002001000013012011010Xx", parseInfo), std::invalid_argument); // multiple checksums
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As08003002001000013012011010xX", parseInfo), std::invalid_argument); // multiple checksums
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000As12000", parseInfo), std::invalid_argument); // string stops in the middle

  // Compact data uart
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05F", parseInfo)); // Valid string, no checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000AR0600F01F02F03F04F05F", parseInfo)); // Valid string, no checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05Fx", parseInfo)); // Valid string, 8-bit checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05FX", parseInfo)); // Valid string, 16-bit checksum
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar0200F01Fr0202F03FR0204F05F", parseInfo)); // Multiple switches

  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar04110220330440", parseInfo)); // useless but should pass
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000AR04110220330440", parseInfo)); // useless but should pass
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ar101001111221331441551661771881991AA1BB1CC1DD1EE1FF", parseInfo)); // all possible masks
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0280F11F", parseInfo), std::invalid_argument); // token out of range
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar02F0F11F", parseInfo), std::invalid_argument); // invalid row
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0G11F12F", parseInfo), std::invalid_argument); // invalid data length
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0311F12F", parseInfo), std::invalid_argument); // too long data length
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0111F12F", parseInfo), std::invalid_argument); // too short data length

  // Identifiers
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ai00r0600F01F02F03F04F05Fx", parseInfo)); // Valid string, one identifier
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ai00i01r0600F01F02F03F04F05Fx", parseInfo)); // Valid string, two identifiers
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ai00i01i02r0600F01F02F03F04F05Fx", parseInfo)); // Valid string, three identifiers
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o000Ai00i01i02i03r0600F01F02F03F04F05Fx", parseInfo)); // Valid string, four identifiers
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ai00i01i02i03i04r0600F01F02F03F04F05Fx", parseInfo), std::invalid_argument); // too many identifiers
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05Fi00x", parseInfo), std::invalid_argument); // identifier in middle of string
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05Fxi00", parseInfo), std::invalid_argument); // identifier after checksum

  // Interleaved messages
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00s04423422421420xo00FAi01R0442F37F38F39FXo03E8iFFR0542F3014010932FFX", parseInfo)); // Valid string
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0002i00r0B42F00F01F02F03F04F05F0930A30B30C3Xo000Ai01r0D42F3014014CF4DF4EF4FF31F32F33F34F35F36FXo000Ai02r0A42F61F62F63F64F65F66F67F68F69FX", parseInfo)); // Ardupilot
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04o0064i05", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04o0064i05o0064i06", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04o0064i05o0064i06o0064i07", parseInfo)); // Max nr of messages
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04o0064i05o0064i06o0064i07o0064i08", parseInfo), std::invalid_argument); // Too many messages
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00o0064i01o0064i02o0064i03o0064i04o0064i05o0064i06o0064i07o0064i08o0064i09", parseInfo), std::invalid_argument);// Too many messages

  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00i01o0064i10i11", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00i01i02o0064i10i11i12o0064i20i21i22", parseInfo));
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00i01i02i03o0064i10i11i12i13o0064i20i21i22i23o0064i30i31i32i33", parseInfo));

  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064r0142Fo0064r0142Fo0064r0142F", parseInfo), std::invalid_argument); // Multiple messages but no identifiers
  
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00r0142Fo0064i01r0142Fo0064i02r0142F", parseInfo)); // Baseline sanity check
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064r0142Fo0064i00r0142Fo0064i01r0142F", parseInfo), std::invalid_argument); // One message missing identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00r0142Fo0064r0142Fo0064i01r0142F", parseInfo), std::invalid_argument); // One message missing identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i01r0142Fo0064i00r0142Fo0064r0142F", parseInfo), std::invalid_argument); // One message missing identifier
  
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00r0142Fo0064i01r0142Fo0064i02r0142F", parseInfo)); // Baseline sanity check
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i01r0142Fo0064i00r0142Fo0064i01r0142F", parseInfo), std::invalid_argument); // Two messages with same identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i01r0142Fo0064i01r0142Fo0064i00r0142F", parseInfo), std::invalid_argument); // Two messages with same identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00r0142Fo0064i01r0142Fo0064i01r0142F", parseInfo), std::invalid_argument); // Two messages with same identifier
  
  BOOST_CHECK_NO_THROW(backend.parseDataUartConfigString("o0064i00i01r0142Fo0064i10i11r0142Fo0064i20i21r0142F", parseInfo)); // Baseline sanity check
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00r0142Fo0064i10i11r0142Fo0064i20i21r0142F", parseInfo), std::invalid_argument); // One message with different size identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00i01r0142Fo0064i10r0142Fo0064i20i21r0142F", parseInfo), std::invalid_argument); // One message with different size identifier
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o0064i00i01r0142Fo0064i10i11r0142Fo0064i20r0142F", parseInfo), std::invalid_argument); // One message with different size identifier
  
  // CAN
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o000AT018003002001000013012011010T028023022021020033032031030T038043042041040053052051050T048063062061060073072071070T0580830820810800910900F10F0T0680A30A20A10A00B30B20B10B0T0780C30C20C10C00D30D20D10D0T0880E30E20E10E0413412411410"));
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o0004C018103102101100113112111110C028123122121120133132131130C038143142141140153152151150C048163162161160173172171170C058183182181180091090091090C068163162161160173172171170C0781831821811800D30D20D10D0C0880E30E20E10E0413412411410C0981931921911901A31A21A11A0C0A81B31B21B11B01C31C21C11C0C0B81D31D21D11D01E31E21E11E0C0C81F31F21F11F0203202201200C0D62132122112100F10F0"));
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o0064t0018603602601600373372371370t0028603602601600383382381380t0038603602601600393392391390o00FAt0118603602601600483482481480t0128603602601600493492491490o03E8t10086036026016002F32F22F12F0p80t10186036026016002B32B22B12B0")); // interleave + phase
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o0005E0000F02A8732731730760702701700760E0000F02D8742741740760712711710760E0000F0298752751750760722721720760"));
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o000At1238003002001000013012011010t4568023022021020033032031030"));
  BOOST_CHECK_NO_THROW(backend.parseCanConfigString("o000At1230"));

  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At1238003002001000013012011010t4568023022021020033032031030K"), std::invalid_argument); // unknown token
  BOOST_CHECK_THROW(backend.parseCanConfigString("o0At1238003002001000013012011010t4568023022021020033032031030"), std::invalid_argument); // Invalid rate
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At128003002001000013012011010t458023022021020033032031030"), std::invalid_argument); // Invalid ID's
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At1234003002001000013012011010t4565423022021020033032031030"), std::invalid_argument); // Invalid data-lens
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At1238F03F02F01F00F13F12F11F10t4568023022021020033032031030"), std::invalid_argument); // Invalid row
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At1238004003002001013012011010t4568023022021020033032031030"), std::invalid_argument); // Invalid column
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000As08003002001000013012011010x"), std::invalid_argument); // Accidental UART cfg

  BOOST_CHECK_THROW(backend.parseCanConfigString("o0000t1230"), std::invalid_argument); // Invalid data rate
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000At8000"), std::invalid_argument); // Invalid std id
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000AT200"), std::invalid_argument); // Invalid ext id + chip id
  BOOST_CHECK_THROW(backend.parseCanConfigString("o000AE200000000"), std::invalid_argument); // Invalid ext id
}

BOOST_AUTO_TEST_CASE(SensorDataUserUartStrings)
{
  SensorDataBackend backend;
  DataSelection selection;
  StandardNmeaSelection nmeaSelection, nmeaSelection2;

  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("0,1,2,3,4,5,6,7,8,9,10,11,12,13")); // Valid string
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("0,1,A"), std::invalid_argument); // invalid token
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("0,1,128"), std::invalid_argument); // token out of range
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("0,1,-1"), std::invalid_argument); // negative tokens
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("0.1"), std::invalid_argument); // periods instead of comma

  // Standard NMEA
  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("0,1,2,GGA1,RMC1,GLL1,GST1"));
  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("GGA,GGA0,GGA1,GGA2"));
  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("RMC,RMC0,RMC1,RMC2"));
  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("GLL,GLL0,GLL1,GLL2"));
  BOOST_CHECK_NO_THROW(backend.parseUserUartConfigString("GST,GST0,GST1,GST2"));
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("GGA3,GGA4,GGA5,GGA6,GGA7,GGA8,GGA9"), std::invalid_argument);
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("GGA3"), std::invalid_argument);
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("GA1"), std::invalid_argument);
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("GGA01"), std::invalid_argument);
  BOOST_CHECK_THROW(backend.parseUserUartConfigString("ZDA"), std::invalid_argument);

  // Standard NMEA parsing
  BOOST_REQUIRE_NO_THROW(backend.parseUserUartConfigString("GGA,RMC0,GLL1,GST2", nmeaSelection));
  BOOST_CHECK( nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GGA, StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV));
  BOOST_CHECK( nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_RMC, StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GLL, StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GST, StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GGA, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_RMC, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1));
  BOOST_CHECK( nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GLL, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GST, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GGA, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS2));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_RMC, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS2));
  BOOST_CHECK(!nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GLL, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS2));
  BOOST_CHECK( nmeaSelection.get(StandardNmeaMessages::STANDARD_NMEA_GST, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS2));

  // Standard NMEA string generation
  selection.reset();
  selection.set(0);
  selection.set(1);
  selection.set(2);

  nmeaSelection.clear_all();
  nmeaSelection.set(StandardNmeaMessages::STANDARD_NMEA_GGA, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1);
  nmeaSelection.set(StandardNmeaMessages::STANDARD_NMEA_RMC, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS1);
  nmeaSelection.set(StandardNmeaMessages::STANDARD_NMEA_GLL, StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV);
  nmeaSelection.set(StandardNmeaMessages::STANDARD_NMEA_GST, StandardNmeaSources::STANDARD_NMEA_SOURCE_GNSS2);
  std::string cfg_str = backend.getUserUartConfigString(selection, nmeaSelection);
  
  //BOOST_TEST_MESSAGE("Generated NMEA: " << cfg_str);
  BOOST_CHECK_EQUAL(cfg_str, "0,1,2,GGA1,RMC1,GLL,GST2");

  BOOST_REQUIRE_NO_THROW(backend.parseUserUartConfigString(cfg_str, nmeaSelection2));
  BOOST_CHECK(nmeaSelection == nmeaSelection2);
}

namespace
{
bool checkEndianness(const DataUartParseInfo& parseInfo)
{
  uint8_t prev_col = 0;
  uint8_t prev_row = 255;
  for (int i = 0; i < parseInfo.messages[0].numBytes; i++)
  {
    if (parseInfo.data[i].row == prev_row)
    {
      bool bigEndianCompliant = parseInfo.data[i].col <= prev_col;
      bool littleEndianCompliant = parseInfo.data[i].col >= prev_col;

      if ( (parseInfo.usingBigEndian && !bigEndianCompliant) || (!parseInfo.usingBigEndian && !littleEndianCompliant) )
      {
        BOOST_TEST_MESSAGE("Row " << int(parseInfo.data[i].row) << " and column " << int(parseInfo.data[i].col) << " don't comply with the endianness!");
        return false;
      }
    }
    prev_row = parseInfo.data[i].row;
    prev_col = parseInfo.data[i].col;
  }
  return true;
}
}


BOOST_AUTO_TEST_CASE(SensorDataEndianess)
{
  SensorDataBackend backend;

  // DATA UART
  DataUartParseInfo parseInfo;

  // BIG endian
  BOOST_REQUIRE_NO_THROW(backend.parseDataUartConfigString("o000As0B003002001000013012011010021020030", parseInfo));
  BOOST_CHECK_EQUAL(parseInfo.usingBigEndian, true);
  BOOST_CHECK(checkEndianness(parseInfo));

  // LITTLE endian
  BOOST_REQUIRE_NO_THROW(backend.parseDataUartConfigString("o000As0B000001002003010011012013020021030", parseInfo));
  BOOST_CHECK_EQUAL(parseInfo.usingBigEndian, false);
  BOOST_CHECK(checkEndianness(parseInfo));

  // BIG endian, compact mode
  BOOST_REQUIRE_NO_THROW(backend.parseDataUartConfigString("o000AR0400F01F023031", parseInfo));
  BOOST_CHECK_EQUAL(parseInfo.usingBigEndian, true);
  BOOST_CHECK(checkEndianness(parseInfo));

  // LITTLE endian, compact mode
  BOOST_REQUIRE_NO_THROW(backend.parseDataUartConfigString("o000Ar0400F01F023031", parseInfo));
  BOOST_CHECK_EQUAL(parseInfo.usingBigEndian, false);
  BOOST_CHECK(checkEndianness(parseInfo));
}

BOOST_AUTO_TEST_CASE(TestDataParser)
{
  SensorDataBackend backend;
  DataUartParseInfo parseInfo;

  struct __attribute__ ((packed))
  {
    uint8_t header;
    int32_t acc_x;
    int16_t temp;
    int16_t mag_x;
    uint32_t tick;
    uint32_t error_flags;
    uint8_t sensor_valid;
    uint8_t alignment_status;
    uint32_t utc_year_month;
    uint32_t utc_day_hour_min_sec;
    uint16_t gnss_fixtype;
  } input_data;

  // Assume system endianess is little endian
  BOOST_REQUIRE_NO_THROW(backend.parseDataUartConfigString("o000Ar0A00F0930A342F2FF30140145F46F475", parseInfo));
  BOOST_REQUIRE_EQUAL(parseInfo.numMessages, 1);
  BOOST_REQUIRE_EQUAL(sizeof(input_data), static_cast<std::size_t>(parseInfo.messages[0].numBytes + 1));

  // Populate fields
  input_data.header = 0xFA;
  input_data.acc_x = 1234567;
  input_data.temp = -1234;
  input_data.mag_x = 456;
  input_data.tick = 987643;
  input_data.error_flags = 0xDEADBEEF;
  input_data.sensor_valid = 0xAF;
  input_data.alignment_status = 1;
  input_data.utc_year_month = (2025 << 16) | 5;
  input_data.utc_day_hour_min_sec = (12 << 24) | (13 << 16) | (37 << 8) | 42;
  input_data.gnss_fixtype = (2 << 8) | 1;

  std::vector<uint8_t> input_vector;
  input_vector.assign((uint8_t*)&input_data, (uint8_t*)&input_data + sizeof(input_data));

  BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));

  auto sample = backend.getSensorSample();
  //for (auto it = sample.begin(); it != sample.end(); ++it) BOOST_TEST_MESSAGE(it->first);

  BOOST_CHECK_EQUAL( static_cast<double>(input_data.acc_x)/1e6, backend.getValueData(0x00) );

  // todo: floating point tolerance
  BOOST_CHECK_EQUAL(static_cast<double>(input_data.acc_x)/1e6, sample.at(MT::ACCELERATION_X).toDouble());
  BOOST_CHECK_EQUAL((static_cast<double>(input_data.temp)/10000) * 80 + 20, sample.at(MT::INTERNAL_TEMP).toDouble());
  BOOST_CHECK_EQUAL(static_cast<double>(input_data.mag_x)/1e3, sample.at(MT::MAGNETIC_FIELD_X).toDouble());
  BOOST_CHECK_EQUAL(input_data.tick, sample.at(MT::TICK).toUint32());
  BOOST_CHECK_EQUAL(input_data.error_flags, sample.at(MT::ERROR_FLAGS).toUint32());
  BOOST_CHECK_EQUAL(input_data.sensor_valid, sample.at(MT::SENSOR_VALID).toUint32());
  BOOST_CHECK_EQUAL(input_data.alignment_status, sample.at(MT::ALIGNMENT_INS).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint16_t>(input_data.utc_year_month >> 16), sample.at(MT::UTC_YEAR).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint16_t>(input_data.utc_year_month), sample.at(MT::UTC_MONTH).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.utc_day_hour_min_sec >> 24), sample.at(MT::UTC_DAY).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.utc_day_hour_min_sec >> 16), sample.at(MT::UTC_HOUR).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.utc_day_hour_min_sec >> 8), sample.at(MT::UTC_MINUTE).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.utc_day_hour_min_sec), sample.at(MT::UTC_SECOND).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.gnss_fixtype), sample.at(MT::GNSS1_FIXTYPE).toUint32());
  BOOST_CHECK_EQUAL(static_cast<uint8_t>(input_data.gnss_fixtype >> 8), sample.at(MT::GNSS2_FIXTYPE).toUint32());
}

BOOST_AUTO_TEST_CASE(TestValidateData)
{
  // Tests crc-16 validation and bad data rejection
  SensorDataBackend backend;
  DataUartParseInfo parseInfo;
  std::vector<uint8_t> data;

  // No checksum
  backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05F", parseInfo);
  BOOST_CHECK(parseInfo.messages[0].checksumBytes == 0);

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07});
  BOOST_CHECK_MESSAGE(backend.validateDataUartData(data, parseInfo), "Correct indata");

  data.assign({0xAF, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "Bad header");

  // 8-bit checksum
  backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05Fx", parseInfo);
  BOOST_CHECK(parseInfo.messages[0].checksumBytes == 1);

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x05});
  BOOST_CHECK_MESSAGE(backend.validateDataUartData(data, parseInfo), "Correct indata");

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x06});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "Bad checksum");

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97, 0x86});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "crc16 instead of 8-bit checksum");

  // 16-bit checksum
  backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05FX", parseInfo);
  BOOST_CHECK(parseInfo.messages[0].checksumBytes == 2);

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97, 0x86});
  BOOST_CHECK_MESSAGE(backend.validateDataUartData(data, parseInfo), "Correct indata");

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97, 0x87});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "Bad checksum");

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x05});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "8-bit checksum instead of crc16");

  // Buffer sizes
  backend.parseDataUartConfigString("o000Ar0600F01F02F03F04F05FX", parseInfo);
  BOOST_CHECK(parseInfo.messages[0].checksumBytes == 2);

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97, 0x86, 0x00});
  BOOST_CHECK_MESSAGE(backend.validateDataUartData(data, parseInfo), "Should allow buffer sizes bigger than required");
  BOOST_CHECK_MESSAGE(backend.parseDataUartData(data, parseInfo), "Should parse successfully");

  data.assign({0xFA, 0x07, 0xE9, 0x00, 0x03, 0x03, 0x0F, 0x20, 0x20, 0x00, 0x05, 0x1E, 0xD4, 0x13, 0x7A, 0x27, 0x35, 0x1F, 0x02, 0x0D, 0x47, 0x00, 0x00, 0x00, 0x07, 0x97});
  BOOST_CHECK_MESSAGE(!backend.validateDataUartData(data, parseInfo), "Should reject if buffer is too small");
}

BOOST_AUTO_TEST_CASE(TestIdentifiers)
{
  // Test proper management of identifier switches
  SensorDataBackend backend;
  DataUartParseInfo parseInfo;
  std::vector<uint8_t> data;

  // no identifier (sanity check...)
  backend.parseDataUartConfigString("o000AR0142F", parseInfo);
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].numIdentifier, 0);

  data.assign({0xFA, 0x01, 0x02, 0x03, 0x00});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  BOOST_CHECK_EQUAL(backend.getValueData(0x42), static_cast<double>(0x01020300));

  // one identifier
  backend.parseDataUartConfigString("o000Ai01R0142F", parseInfo);
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].numIdentifier, 1);

  data.assign({0xFA, 0x01, 0x01, 0x02, 0x03, 0x01});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  BOOST_CHECK_EQUAL(backend.getValueData(0x42), static_cast<double>(0x01020301));

  data.assign({0xFA, 0x00, 0x01, 0x02, 0x03, 0x01}); // wrong id, same length
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x03, 0x01}); // no id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));

  // two identifiers
  backend.parseDataUartConfigString("o000Ai01i02R0142F", parseInfo);
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].numIdentifier, 2);

  data.assign({0xFA, 0x01, 0x02, 0x01, 0x02, 0x03, 0x02});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  BOOST_CHECK_EQUAL(backend.getValueData(0x42), static_cast<double>(0x01020302));

  data.assign({0xFA, 0x00, 0x00, 0x01, 0x02, 0x03, 0x02}); // wrong id, same length
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x01, 0x02, 0x03, 0x02}); // one id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x03, 0x02}); // no id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));

  // three identifiers
  backend.parseDataUartConfigString("o000Ai01i02i03R0142F", parseInfo);
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].numIdentifier, 3);

  data.assign({0xFA, 0x01, 0x02, 0x03, 0x01, 0x02, 0x03, 0x03});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  BOOST_CHECK_EQUAL(backend.getValueData(0x42), static_cast<double>(0x01020303));

  data.assign({0xFA, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x03}); // wrong id, same length
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x01, 0x02, 0x03, 0x03}); // two id's
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x01, 0x02, 0x03, 0x03}); // one id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x03, 0x03}); // no id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));

  // four identifiers
  backend.parseDataUartConfigString("o000Ai01i02i03i04R0142F", parseInfo);
  BOOST_CHECK_EQUAL(parseInfo.numMessages, 1);
  BOOST_CHECK_EQUAL(parseInfo.messages[0].numIdentifier, 4);

  data.assign({0xFA, 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  BOOST_CHECK_EQUAL(backend.getValueData(0x42), static_cast<double>(0x01020304));

  data.assign({0xFA, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04}); // wrong id, same length
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x03, 0x01, 0x02, 0x03, 0x04}); // three id's
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x01, 0x02, 0x03, 0x04}); // two id's
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x01, 0x02, 0x03, 0x04}); // one id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));
  data.assign({0xFA, 0x01, 0x02, 0x03, 0x04}); // no id
  BOOST_CHECK(!backend.validateDataUartData(data, parseInfo));

  // five identifiers is not allowed...
  BOOST_CHECK_THROW(backend.parseDataUartConfigString("o000Ai01i02i03i04i05R0142F", parseInfo), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(TestInterleavedData)
{
  // Tests interleaved messages, uses the string:
  // o0064i00s04423422421420xo00FAi01R0442F37F38F39FXo03E8iFFR0542F3014010932FFX
  // =
  // o0064i00s04423422421420x        tick @10 Hz, id: 0x00, len = 1 + 4 + 1
  // o00FAi01R0442F37F38F39FX        tick + rpy @4Hz, id: 0x01, len = 1 + 16 + 2
  // o03E8iFFR0542F3014010932FFX     tick + status + temp + error @1Hz, id: 0xFF, len = 1 + 12 + 2

  SensorDataBackend backend;
  DataUartParseInfo parseInfo;
  DataSelection selection;
  SensorSample sample;
  std::vector<uint8_t> data;

  selection = backend.parseDataUartConfigString("o0064i00s04423422421420xo00FAi01R0442F37F38F39FXo03E8iFFR0542F3014010932FFX", parseInfo);
  BOOST_REQUIRE_EQUAL(parseInfo.numMessages, 3);
  BOOST_REQUIRE_EQUAL(parseInfo.messages[0].numIdentifier, static_cast<uint16_t>(1));
  BOOST_REQUIRE_EQUAL(parseInfo.messages[0].identifier, static_cast<uint32_t>(0x00));
  BOOST_REQUIRE_EQUAL(parseInfo.messages[1].numIdentifier, static_cast<uint16_t>(1));
  BOOST_REQUIRE_EQUAL(parseInfo.messages[1].identifier, static_cast<uint32_t>(0x01));
  BOOST_REQUIRE_EQUAL(parseInfo.messages[2].numIdentifier, static_cast<uint16_t>(1));
  BOOST_REQUIRE_EQUAL(parseInfo.messages[2].identifier, static_cast<uint32_t>(0xFF));

#if 0
  BOOST_TEST_MESSAGE("numMessages: " << parseInfo.numMessages);
  BOOST_TEST_MESSAGE("usingBigEndian: " << parseInfo.usingBigEndian);
  int numBytes = 0;
  for (int i = 0; i < parseInfo.numMessages; i++)
  {
    BOOST_TEST_MESSAGE("messages[" << i << "].dataRate: " << parseInfo.messages[i].dataRate);
    BOOST_TEST_MESSAGE("messages[" << i << "].numIdentifier: " << parseInfo.messages[i].numIdentifier);
    BOOST_TEST_MESSAGE("messages[" << i << "].identifier: " << parseInfo.messages[i].identifier);
    BOOST_TEST_MESSAGE("messages[" << i << "].checksumBytes: " << parseInfo.messages[i].checksumBytes);
    BOOST_TEST_MESSAGE("messages[" << i << "].startIndex: " << parseInfo.messages[i].startIndex);
    BOOST_TEST_MESSAGE("messages[" << i << "].numBytes: " << parseInfo.messages[i].numBytes);
    numBytes += parseInfo.messages[i].numBytes;
  }
  for (int i = 0; i < (numBytes + 2); i++)
  {
    BOOST_TEST_MESSAGE("data[" << i << "]: " << parseInfo.data[i].row << " / " << parseInfo.data[i].col);
  }
#endif

  // Messages yoinked from real log, check they update the correct

  // ID = 0x00, tick
  data.assign({0xFA, 0x00, 0x00, 0x01, 0x2F, 0xBB, 0x95});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  sample = backend.getSensorSample(selection);

  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::TICK));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ROLL));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::PITCH));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::HEADING));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::SENSOR_VALID));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ALIGNMENT_INS));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::TEMP));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ERROR_FLAGS));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::TICK), static_cast<double>(0x00012FBB));

  BOOST_CHECK(sample.contains(MT::TICK) && sample.at(MT::TICK).isUpdated && sample.at(MT::TICK).toUint32() == 0x00012FBB);
  BOOST_CHECK(sample.contains(MT::ROLL) && !sample.at(MT::ROLL).isUpdated);
  BOOST_CHECK(sample.contains(MT::PITCH) && !sample.at(MT::PITCH).isUpdated);
  BOOST_CHECK(sample.contains(MT::HEADING) && !sample.at(MT::HEADING).isUpdated);
  BOOST_CHECK(sample.contains(MT::SENSOR_VALID) && !sample.at(MT::SENSOR_VALID).isUpdated);
  BOOST_CHECK(sample.contains(MT::ALIGNMENT_INS) && !sample.at(MT::ALIGNMENT_INS).isUpdated);
  BOOST_CHECK(sample.contains(MT::INTERNAL_TEMP) && !sample.at(MT::INTERNAL_TEMP).isUpdated);
  BOOST_CHECK(sample.contains(MT::ERROR_FLAGS) && !sample.at(MT::ERROR_FLAGS).isUpdated);

  // ID = 0x01, tick + rpy
  data.assign({0xFA, 0x01, 0x00, 0x01, 0x31, 0x4B, 0xFF, 0xD9, 0x72, 0x9C, 0x00, 0x49, 0xAA, 0x3F, 0xFB, 0x6A, 0xEA, 0x80, 0xCC, 0x5C});
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  sample = backend.getSensorSample(selection);

  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::TICK));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::ROLL));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::PITCH));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::HEADING));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::SENSOR_VALID));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ALIGNMENT_INS));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::TEMP));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ERROR_FLAGS));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::TICK), static_cast<double>(0x0001314B));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::PITCH), static_cast<double>(0x0049AA3F) * 1e-6); // approx...

  BOOST_CHECK(sample.contains(MT::TICK) && sample.at(MT::TICK).isUpdated && sample.at(MT::TICK).toUint32() == 0x0001314B);
  BOOST_CHECK(sample.contains(MT::ROLL) && sample.at(MT::ROLL).isUpdated);
  BOOST_CHECK(sample.contains(MT::PITCH) && sample.at(MT::PITCH).isUpdated && sample.at(MT::PITCH).toDouble() == (static_cast<double>(0x0049AA3F) * 1e-6));
  BOOST_CHECK(sample.contains(MT::HEADING) && sample.at(MT::HEADING).isUpdated);
  BOOST_CHECK(sample.contains(MT::SENSOR_VALID) && !sample.at(MT::SENSOR_VALID).isUpdated);
  BOOST_CHECK(sample.contains(MT::ALIGNMENT_INS) && !sample.at(MT::ALIGNMENT_INS).isUpdated);
  BOOST_CHECK(sample.contains(MT::INTERNAL_TEMP) && !sample.at(MT::INTERNAL_TEMP).isUpdated);
  BOOST_CHECK(sample.contains(MT::ERROR_FLAGS) && !sample.at(MT::ERROR_FLAGS).isUpdated);

  // ID = 0xFF, tick + status + temp + error
  data.assign({0xFA, 0xFF, 0x00, 0x01, 0x35, 0x33, 0x17, 0x01, 0x0D, 0x34, 0x00, 0x00, 0x00, 0x00, 0xF2, 0x7C}); // id: 0xFF
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  sample = backend.getSensorSample(selection);

  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::TICK));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ROLL));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::PITCH));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::HEADING));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::SENSOR_VALID));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::ALIGNMENT_INS));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::TEMP));
  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::ERROR_FLAGS));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::TICK), static_cast<double>(0x00013533));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::SENSOR_VALID), static_cast<double>(0x17));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::ALIGNMENT_INS), static_cast<double>(0x01));

  BOOST_CHECK(sample.contains(MT::TICK) && sample.at(MT::TICK).isUpdated && sample.at(MT::TICK).toUint32() == 0x00013533);
  BOOST_CHECK(sample.contains(MT::ROLL) && !sample.at(MT::ROLL).isUpdated);
  BOOST_CHECK(sample.contains(MT::PITCH) && !sample.at(MT::PITCH).isUpdated);
  BOOST_CHECK(sample.contains(MT::HEADING) && !sample.at(MT::HEADING).isUpdated);
  BOOST_CHECK(sample.contains(MT::SENSOR_VALID) && sample.at(MT::SENSOR_VALID).isUpdated && sample.at(MT::SENSOR_VALID).toUint32() == 0x17);
  BOOST_CHECK(sample.contains(MT::ALIGNMENT_INS) && sample.at(MT::ALIGNMENT_INS).isUpdated && sample.at(MT::ALIGNMENT_INS).toUint32() == 0x01);
  BOOST_CHECK(sample.contains(MT::INTERNAL_TEMP) && sample.at(MT::INTERNAL_TEMP).isUpdated);
  BOOST_CHECK(sample.contains(MT::ERROR_FLAGS) && sample.at(MT::ERROR_FLAGS).isUpdated);

  // ID = 0x00, tick
  data.assign({0xFA, 0x00, 0x00, 0x01, 0x36, 0x5F, 0x68}); // id: 0x00
  BOOST_CHECK(backend.validateDataUartData(data, parseInfo));
  BOOST_CHECK(backend.parseDataUartData(data, parseInfo));
  sample = backend.getSensorSample(selection);

  BOOST_CHECK(backend.isValueUpdated(SensorDataValueId::TICK));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ROLL));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::PITCH));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::HEADING));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::SENSOR_VALID));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ALIGNMENT_INS));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::TEMP));
  BOOST_CHECK(!backend.isValueUpdated(SensorDataValueId::ERROR_FLAGS));
  BOOST_CHECK_EQUAL(backend.getValueData(SensorDataValueId::TICK), static_cast<double>(0x0001365F));

  BOOST_CHECK(sample.contains(MT::TICK) && sample.at(MT::TICK).isUpdated && sample.at(MT::TICK).toUint32() == 0x0001365F);
  BOOST_CHECK(sample.contains(MT::ROLL) && !sample.at(MT::ROLL).isUpdated);
  BOOST_CHECK(sample.contains(MT::PITCH) && !sample.at(MT::PITCH).isUpdated);
  BOOST_CHECK(sample.contains(MT::HEADING) && !sample.at(MT::HEADING).isUpdated);
  BOOST_CHECK(sample.contains(MT::SENSOR_VALID) && !sample.at(MT::SENSOR_VALID).isUpdated);
  BOOST_CHECK(sample.contains(MT::ALIGNMENT_INS) && !sample.at(MT::ALIGNMENT_INS).isUpdated);
  BOOST_CHECK(sample.contains(MT::INTERNAL_TEMP) && !sample.at(MT::INTERNAL_TEMP).isUpdated);
  BOOST_CHECK(sample.contains(MT::ERROR_FLAGS) && !sample.at(MT::ERROR_FLAGS).isUpdated);
}

namespace
{

// Test data with combined quaternion/euler/rotmat entries
// Can generate with MATLAB with the following commands:
// q = quaternion([...]) or q = quaternion([3x3 rotmat], 'rotmat', 'point') or q = quaternion([roll,pitch,yaw], 'eulerd', 'XYZ', 'point')
// quat = q.compact() and rotmat = q.rotmat('point') and euler = q.eulerd('XYZ','point')
struct AttitudeTestData
{
  struct
  {
    double w, x, y, z;
  } quat;
  struct
  {
    double roll,pitch,heading;
  } euler;
  struct
  {
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
  } rotmat;
};

AttitudeTestData testdata_att[] = 
{\
  // Attitude data taken from NGLogger_2024-11-18_111139.csv (modellflyg aggressive)
  { { 0.9879770, -0.0004697,  0.0842126,  0.1296511}, {   1.2150214,    9.5856651,   15.0541582}, { 0.9521977, -0.2562637,  0.1662785,  0.2561055,  0.9663807,  0.0227646, -0.1665221,  0.0209085,  0.9858160} },\
  { { 0.9898985,  0.0068739,  0.0783915,  0.1179344}, {   1.8615481,    8.8343243,   13.7319630}, { 0.9598925, -0.2324085,  0.1568205,  0.2345640,  0.9720884,  0.0048812, -0.1535778,  0.0320991,  0.9876151} },\
  { { 0.8440214, -0.0365172,  0.0804446,  0.5289831}, {   1.3655133,   10.0453695,   64.2741600}, { 0.4274112, -0.8988212,  0.0971600,  0.8870708,  0.4376868,  0.1467503, -0.1744279,  0.0234651,  0.9843903} },\
  { { 0.9913707,  0.0200892,  0.0976388, -0.0851307}, {   1.3564078,   11.3623421,   -9.6811450}, { 0.9664389,  0.1727151,  0.1901722, -0.1648691,  0.9846984, -0.0564559, -0.1970130,  0.0232076,  0.9801262} },\
  { { 0.2583181,  0.0167647,  0.0304704,  0.9654337}, {   3.8706931,   -0.9527742,  150.0086817}, {-0.8659814, -0.4977563,  0.0481125,  0.4997997, -0.8646866,  0.0501731,  0.0166283,  0.0674956,  0.9975810} },\
  { { 0.9182695,  0.1077671, -0.0893875, -0.3703746}, {  15.3715039,   -4.8377974,  -44.5856154}, { 0.7096650,  0.6609413, -0.2439920, -0.6994734,  0.7024178, -0.1317048,  0.0843352,  0.2641322,  0.9607922} },\
  { { 0.9393024, -0.2205615,  0.1349045, -0.2255312}, { -28.7462407,    8.8556286,  -29.2763874}, { 0.8618729,  0.3641746,  0.3529193, -0.4831936,  0.8009766,  0.3534975, -0.1539452, -0.4751982,  0.8663068} },\
  { {-0.5586017, -0.3231753, -0.6377944, -0.4204048}, {  91.4330534,   26.1560322,  100.7253260}, {-0.1670436, -0.0574388,  0.9842750,  0.8819165,  0.4376351,  0.1752110, -0.4408172,  0.8973162, -0.0224479} },\
  { {-0.1721099,  0.1939637, -0.4965070,  0.8283943}, { -64.1072241,   -8.6529629, -151.1015363}, {-0.8655126,  0.0925411,  0.4922643, -0.4777583, -0.4477180, -0.7558410,  0.1504493, -0.8893732,  0.4317179} },\
  { { 0.8357748, -0.1763204,  0.0947806, -0.5112809}, { -23.0628793,   -1.2530471,  -62.6563234}, { 0.4592170,  0.8212079,  0.3387291, -0.8880549,  0.4150059,  0.1978094,  0.0218681, -0.3916474,  0.9198555} },\
  { { 0.4329295,  0.5067668,  0.5083560,  0.5452831}, {  91.7576218,   -6.4593473,   96.4439650}, {-0.1115190,  0.0430976,  0.9928273,  0.9873741, -0.1082926,  0.1156073,  0.1124982,  0.9931844, -0.0304768} },\
  { { 0.7196478, -0.6376107, -0.0746270,  0.2645569}, { -79.5948874,   13.2946210,   29.2780194}, { 0.8488809, -0.2856097, -0.4447790,  0.4759416,  0.0469244,  0.8782242, -0.2299584, -0.9571965,  0.1757667} },\
  { { 0.8964030, -0.0394130,  0.0649847, -0.4366753}, {  -7.3451265,    4.7083372,  -52.2476702}, { 0.6101834,  0.7777516,  0.1509264, -0.7879965,  0.6155226,  0.0139055, -0.0820835, -0.1274144,  0.9884472} },\
  { {-0.7057562, -0.4380509, -0.5413070, -0.1304086}, {  87.7232124,   40.5272992,   60.0076114}, { 0.3799608,  0.2901667,  0.8783126,  0.6583133,  0.5822100, -0.4771322, -0.6498103,  0.7594964,  0.0301963} },\
  { {-0.5329841, -0.1254984,  0.2687669,  0.7924282}, {  34.1869015,   -5.0255535, -113.6970886}, {-0.4003562,  0.7772436, -0.4853939, -0.9121629, -0.2873846,  0.2921796,  0.0876000,  0.5597342,  0.8240290} },\
  { {-0.2231452,  0.2921375,  0.3598067, -0.8575553}, { -52.6535849,   19.9054977,  140.9040480}, {-0.7297238, -0.1724927, -0.6616264,  0.5929448, -0.6414907, -0.4867301, -0.3404698, -0.7474865,  0.5703896} },\
  { {-0.6075319, -0.1431425,  0.2043605,  0.7540902}, {  28.8418359,   -1.8582136, -102.7646372}, {-0.2208304,  0.8577624, -0.4641957, -0.9747730, -0.1782836,  0.1342851,  0.0324263,  0.4821397,  0.8754940} },\
  { {-0.4759027,  0.4603986,  0.4085511, -0.6282003}, { -75.7169451,   10.9285095,   97.2023018}, {-0.1230994, -0.2217318, -0.9673063,  0.9741173, -0.2132052, -0.0750940, -0.1895840, -0.9515138,  0.2422382} },\
  { {-0.6119292, -0.5162790, -0.4388224, -0.4079627}, {  85.2771607,    6.6504263,   73.5061174}, { 0.2820026, -0.0461790,  0.9583016,  0.9523982,  0.1340449, -0.2738059, -0.1158114,  0.9898987,  0.0817818} },\
  { {-0.1288255,  0.0590661,  0.6336737,  0.7605083}, {  78.6775337,  -14.6614528, -172.8097659}, {-0.9598304,  0.2708030, -0.0734262, -0.1210886, -0.1637233,  0.9790466,  0.2531071,  0.9486097,  0.1899378} },\
  { { 0.7886964,  0.4471351, -0.1561787, -0.3919647}, {  56.3324837,    5.9791935,  -49.6491722}, { 0.6439437,  0.4786164, -0.5968775, -0.7579483,  0.2928676, -0.5828747, -0.1041673,  0.8277408,  0.5513568} },\
  { {-0.8166209, -0.5521190, -0.1679944,  0.0085333}, {  69.6226444,   16.4870302,   10.3071903}, { 0.9434101,  0.1994427,  0.2649528,  0.1715690,  0.3901836, -0.9046109, -0.2837983,  0.8988767,  0.3338849} },\
  { { 0.1999149,  0.2266383,  0.6497049,  0.6975332}, {  86.9549137,   -3.2334310,  144.9490297}, {-0.8173382,  0.0156014,  0.5759469,  0.5733906, -0.0758351,  0.8157648,  0.0564041,  0.9969983,  0.0530372} },\
  { { 0.7960494,  0.5395123, -0.0403831, -0.2712953}, {  64.7954072,   13.2052653,  -29.2366572}, { 0.8495361,  0.3883545, -0.3570282, -0.4755033,  0.2706507, -0.8370453, -0.2284403,  0.8808683,  0.4145914} },\
  { { 0.8592855,  0.0293562, -0.0329881, -0.5095865}, {   4.8243587,   -1.5341803,  -61.4034921}, { 0.4784668,  0.8738238, -0.0866115, -0.8776974,  0.4789196, -0.0168302,  0.0267733,  0.0840713,  0.9961000} },\
  { { 0.0397235,  0.0285028, -0.0677396,  0.9965044}, {  -7.6429093,   -3.5654106,  175.6727058}, {-0.9952193, -0.0830308,  0.0514246,  0.0753077, -0.9876668, -0.1372702,  0.0621880, -0.1327413,  0.9891979} },\
  { { 0.9066822, -0.0863606, -0.0358289, -0.4113217}, {  -7.3725984,   -7.8173107,  -48.2989696}, { 0.6590615,  0.7520645,  0.0060730, -0.7396877,  0.6467127,  0.1860777,  0.1360149, -0.1271288,  0.9825163} },\
  { {-0.4597551, -0.6767789,  0.2576012, -0.5140401}, {  97.7698986,  -68.8517302,   20.1000893}, { 0.3388087, -0.8213433,  0.4589160,  0.1239870, -0.4445337, -0.8871398,  0.9326499,  0.3574703, -0.0487760} },\
  { {-0.3089801,  0.6903872, -0.1470305, -0.6374000}, { -89.1628670,   76.1593162,   52.9284482}, { 0.1442065, -0.5969038, -0.7892467,  0.1908720, -0.7658266,  0.6140663, -0.9709647, -0.2391974,  0.0034951} },\
  { { 0.8114432, -0.1285075, -0.0633894, -0.5665929}, {  -8.1143149,  -14.3885762,  -68.8236570}, { 0.3499085,  0.9358080,  0.0427491, -0.9032239,  0.3249166,  0.2803851,  0.2484968, -0.1367211,  0.9589352} },\
  { { 0.6204010,  0.2808673,  0.7211052, -0.1273715}, { 140.1934742,   75.0825445,  106.3420226}, {-0.0724324,  0.5631125,  0.8231998,  0.2470269,  0.8097802, -0.5321971, -0.9662977,  0.1648042, -0.1977583} },\
  { {-0.2013437, -0.0394523, -0.2219087, -0.9532370}, {  26.0397084,    0.8104810,  156.3338792}, {-0.9158085, -0.3663469,  0.1645745,  0.4013661, -0.8204345,  0.4071762, -0.0141451,  0.4389500,  0.8984001} },\
  { {-0.7428491,  0.4769536, -0.2243531,  0.4127422}, { -63.5654915,   -3.4625865,  -55.9690420}, { 0.5586191,  0.3991983,  0.7270388, -0.8272224,  0.2043183,  0.5234091,  0.0603968, -0.8938091,  0.4443619} },\
  { { 0.5654727,  0.3908600,  0.5681651, -0.4523909}, { -55.8618956,   85.0078944, -129.1489635}, {-0.0549381,  0.9557754,  0.2889207, -0.0674835,  0.2851419, -0.9561067, -0.9962067, -0.0720241,  0.0488339} },\
  { { 0.4837317, -0.2484231,  0.2853116, -0.7892319}, { -44.0588334,   -6.6669845, -114.2903619}, {-0.4085793,  0.6217969,  0.6681553, -0.9053088, -0.3692020, -0.2100137,  0.1160984, -0.6906942,  0.7137666} },\
  { {-0.1772178,  0.3240382,  0.4789889, -0.7963434}, { -69.3301998,   20.2624295,  140.8191620}, {-0.7271863,  0.0281689, -0.6858620,  0.5926738, -0.4783270, -0.6480286, -0.3463206, -0.8777299,  0.3311379} },\
  { { 0.8283748, -0.3521616,  0.2803232, -0.3334612}, { -52.3292669,   13.2712149,  -50.3966445}, { 0.6204451,  0.3550237,  0.6992897, -0.7498998,  0.5295717,  0.3964897, -0.2295608, -0.7703973,  0.5948024} },\
  { { 0.2229895, -0.0932211,  0.1124730,  0.9638130}, {  10.3727929,   13.2885919,  155.1578743}, {-0.8831711, -0.4508100, -0.1295348,  0.4088706, -0.8752511,  0.2583804, -0.2298560,  0.1752311,  0.9573193} },\
  { { 0.3278801,  0.8804693, -0.2854093, -0.1892352}, { 136.1456979,    8.3993326,  -39.3070327}, { 0.7654631, -0.3784954, -0.5203916, -0.6266812, -0.6220723, -0.4693577, -0.1460715,  0.6853957, -0.7133694} },\
  { {-0.3044821, -0.0309479, -0.2101174, -0.9285384}, {  24.2092451,    4.0416330,  144.5568999}, {-0.8126658, -0.5524412,  0.1854267,  0.5784520, -0.7262826,  0.3713580, -0.0704813,  0.4090504,  0.9097858} },\
  { {-0.8062775,  0.0925440,  0.0011918,  0.5842524}, {  -8.5541076,   -6.3187670,  -71.3833375}, { 0.3172956,  0.9423596,  0.1062163, -0.9419184,  0.3001696,  0.1506249,  0.1100599, -0.1478397,  0.9828684} },\
  { {-0.5470365, -0.4505381, -0.4695392, -0.5265922}, {  81.1867431,    2.2471691,   89.7438570}, { 0.0044671, -0.1530397,  0.9882099,  0.9992210,  0.0394320,  0.0015898, -0.0392104,  0.9874330,  0.1530967} },\
  { { 0.6608396, -0.2826067,  0.1783277, -0.6720295}, { -38.2920480,   -8.2879516,  -88.0801826}, { 0.0331510,  0.7874143,  0.6155321, -0.9890007, -0.0629806,  0.1338324,  0.1441481, -0.6131983,  0.7766654} },\
  { { 0.6791909,  0.4981241, -0.2244238, -0.4901082}, {  65.7972027,   10.5688088,  -64.7806922}, { 0.4188558,  0.4421722, -0.7931227, -0.8893359,  0.0233326, -0.4566588, -0.1834162,  0.8966266,  0.4030126} },\
  { { 0.3131737, -0.2438649, -0.3815067,  0.8348082}, { -53.2380049,    9.6834779,  134.0115797}, {-0.6849042, -0.3368078, -0.6461167,  0.7089523, -0.5127497, -0.4842257, -0.1682051, -0.7897141,  0.5899650} },\
  { { 0.7872362, -0.0643383, -0.0180854, -0.6130193}, {  -4.5647356,   -6.1629285,  -75.5697876}, { 0.2477604,  0.9675092,  0.0504063, -0.9628549,  0.2401357,  0.1234722,  0.1073561, -0.0791255,  0.9910670} },\
  { { 0.1035115,  0.1145294,  0.1192538,  0.9807889}, {  15.2449929,  -11.5352044,  166.4018449}, {-0.9523368, -0.1757298,  0.2493466,  0.2303621, -0.9501278,  0.2102155,  0.1999700,  0.2576359,  0.9453231} },\
  { { 0.6727555, -0.5963503,  0.1976128, -0.3907883}, { -77.5854437,  -11.5488414,  -51.0084305}, { 0.6164674,  0.2901170,  0.7319837, -0.7615029, -0.0166983,  0.6479464,  0.2002032, -0.9568455,  0.2106310} },\
  { { 0.8409452,  0.5333638,  0.0637304, -0.0653649}, {  64.5514190,   10.1900652,   -2.4430304}, { 0.9833317,  0.1779196,  0.0374610, -0.0419536,  0.4225009, -0.9053910, -0.1769141,  0.8887281,  0.4229229} },\
  { { 0.2692671,  0.0363301,  0.2997245,  0.9145166}, {  34.7745315,    5.4491799,  148.8947934}, {-0.8523507, -0.4707204,  0.2278609,  0.5142765, -0.6753210,  0.5286410, -0.0949628,  0.5677710,  0.8176907} },\
  { {-0.5975089, -0.1234062,  0.2360850,  0.7563186}, {  30.4574533,   -5.4776192, -104.8731631}, {-0.2555080,  0.8455455, -0.4687947, -0.9620830, -0.1744939,  0.2096383,  0.0954569,  0.5045837,  0.8580695} },\
  { {-0.9138308, -0.1557500,  0.0035664,  0.3750235}, {  16.8039195,    6.3327032,  -43.6888479}, { 0.7186894,  0.6843050, -0.1233380, -0.6865269,  0.6701987, -0.2819833, -0.1103016,  0.2873333,  0.9514584} },\
  { {-0.9169584, -0.0444481, -0.0859050,  0.3870814}, {   0.8763227,   11.0667603,  -45.6878515}, { 0.6855766,  0.7175118,  0.1231325, -0.7022385,  0.6963847, -0.1480186, -0.1919526,  0.0150097,  0.9812894} },\
  { {-0.9160072, -0.0439822, -0.0858326,  0.3893958}, {   0.8015502,   11.0403073,  -45.9833400}, { 0.6820074,  0.7209289,  0.1229936, -0.7058285,  0.6928730, -0.1474217, -0.1914995,  0.0137303,  0.9813967} },\
  { {-0.9151402, -0.0442469, -0.0856472,  0.3914401}, {   0.8133496,   11.0344010,  -46.2378913}, { 0.6788785,  0.7240242,  0.1221183, -0.7088658,  0.6896338, -0.1480357, -0.1913983,  0.0139327,  0.9814135} },\
  { {-0.9142975, -0.0444460, -0.0854944,  0.3934150}, {   0.8175075,   11.0290103,  -46.4847252}, { 0.6758308,  0.7269964,  0.1213631, -0.7117969,  0.6864984, -0.1485432, -0.1913060,  0.0140042,  0.9814305} },\
  { {-0.9149436, -0.0477110, -0.0803852,  0.3926067}, {   1.4101280,   10.6354341,  -46.3175407}, { 0.6787965,  0.7260965,  0.1096326, -0.7107554,  0.6871674, -0.1504253, -0.1845592,  0.0241861,  0.9825238} },\
  { {-0.9133491, -0.0569652, -0.0759415,  0.3959561}, {   2.5608704,   10.5931462,  -46.6380496}, { 0.6749033,  0.7319444,  0.0936108, -0.7146403,  0.6799474, -0.1641973, -0.1838338,  0.0439193,  0.9819757} },\
  { {-0.9131598, -0.0571130, -0.0757799,  0.3964022}, {   2.5788070,   10.5840481,  -46.6923749}, { 0.6742454,  0.7326132,  0.0931188, -0.7153011,  0.6792068, -0.1643852, -0.1836777,  0.0442280,  0.9819910} },\
  { {-0.9952012, -0.0379346,  0.0886059, -0.0168692}, {   4.2258209,  -10.2324002,    1.5636818}, { 0.9837288, -0.0402989, -0.1750816,  0.0268540,  0.9965528, -0.0784945,  0.1776413,  0.0725157,  0.9814199} },\
  { {-0.9906564, -0.0620462,  0.0943138, -0.0765182}, {   6.3529976,  -11.3242067,    8.2029914}, { 0.9704998, -0.1633101, -0.1773698,  0.1399029,  0.9805905, -0.1373663,  0.1963604,  0.1084994,  0.9745104} },\
  { {-0.9895707, -0.0673152,  0.1191948, -0.0448446}, {   7.2551281,  -14.0011127,    4.2973586}, { 0.9675631, -0.1048011, -0.2298658,  0.0727066,  0.9869152, -0.1439168,  0.2419407,  0.1225359,  0.9625225} },\
  { {-0.9887705, -0.0540189,  0.1307935, -0.0480394}, {   5.6079284,  -15.2980176,    4.8093222}, { 0.9611705, -0.1091305, -0.2534595,  0.0808692,  0.9895484, -0.1193910,  0.2638397,  0.0942580,  0.9599500} },\
  { {-0.9867428, -0.0569646,  0.1240475,  0.0877823}, {   7.9354225,  -13.5801343,  -11.1138262}, { 0.9538130,  0.1591044, -0.2548070, -0.1873697,  0.9780986, -0.0906405,  0.2348051,  0.1341972,  0.9627345} },\
  { {-0.9927820, -0.0580920, -0.0833457, -0.0637393}, {   7.3294437,    9.0956241,    7.9307957}, { 0.9779816, -0.1168751,  0.1728936,  0.1362420,  0.9851252, -0.1047206, -0.1580827,  0.1259702,  0.9793576} },\
  
  // All orthogonal axis configurations, tests gimbal lock
  { { 1.0000000,  0.0000000,  0.0000000,  0.0000000}, {   0,   -0,    0}, { 1,  0,  0,  0,  1,  0,  0,  0,  1} }, /* AC  1 */ \
  { { 0.7071068, -0.7071068, -0.0000000,  0.0000000}, { -90,   -0,    0}, { 1,  0,  0,  0,  0,  1,  0, -1,  0} }, /* AC  2 */ \
  { { 0.0000000,  1.0000000,  0.0000000,  0.0000000}, { 180,   -0,    0}, { 1,  0,  0,  0, -1,  0,  0,  0, -1} }, /* AC  3 */ \
  { { 0.7071068,  0.7071068,  0.0000000,  0.0000000}, {  90,   -0,    0}, { 1,  0,  0,  0,  0, -1,  0,  1,  0} }, /* AC  4 */ \
  { { 0.7071068,  0.0000000, -0.7071068,  0.0000000}, {   0,  -90,   -0}, { 0,  0, -1,  0,  1,  0,  1,  0,  0} }, /* AC  5 */ \
  { { 0.5000000,  0.5000000, -0.5000000,  0.5000000}, {   0,  -90,   90}, { 0, -1,  0,  0,  0, -1,  1,  0,  0} }, /* AC  6 */ \
  { { 0.0000000, -0.7071068, -0.0000000, -0.7071068}, {   0,  -90,  180}, { 0,  0,  1,  0, -1,  0,  1,  0,  0} }, /* AC  7 */ \
  { { 0.5000000, -0.5000000, -0.5000000, -0.5000000}, {   0,  -90,  -90}, { 0,  1,  0,  0,  0,  1,  1,  0,  0} }, /* AC  8 */ \
  { { 0.0000000,  0.0000000,  1.0000000,  0.0000000}, { 180,   -0,  180}, {-1,  0,  0,  0,  1,  0,  0,  0, -1} }, /* AC  9 */ \
  { { 0.0000000, -0.0000000, -0.7071068,  0.7071068}, { -90,   -0,  180}, {-1,  0,  0,  0,  0, -1,  0, -1,  0} }, /* AC 10 */ \
  { { 0.0000000,  0.0000000,  0.0000000,  1.0000000}, {   0,   -0,  180}, {-1,  0,  0,  0, -1,  0,  0,  0,  1} }, /* AC 11 */ \
  { { 0.0000000,  0.0000000,  0.7071068,  0.7071068}, {  90,   -0,  180}, {-1,  0,  0,  0,  0,  1,  0,  1,  0} }, /* AC 12 */ \
  { { 0.7071068,  0.0000000,  0.7071068,  0.0000000}, {   0,   90,    0}, { 0,  0,  1,  0,  1,  0, -1,  0,  0} }, /* AC 13 */ \
  { { 0.5000000, -0.5000000,  0.5000000,  0.5000000}, {   0,   90,   90}, { 0, -1,  0,  0,  0,  1, -1,  0,  0} }, /* AC 14 */ \
  { { 0.0000000, -0.7071068,  0.0000000,  0.7071068}, {   0,   90,  180}, { 0,  0, -1,  0, -1,  0, -1,  0,  0} }, /* AC 15 */ \
  { { 0.5000000,  0.5000000,  0.5000000, -0.5000000}, {   0,   90,  -90}, { 0,  1,  0,  0,  0, -1, -1,  0,  0} }, /* AC 16 */ \
  { { 0.5000000,  0.5000000, -0.5000000, -0.5000000}, {  90,   -0,  -90}, { 0,  0, -1, -1,  0,  0,  0,  1,  0} }, /* AC 17 */ \
  { { 0.7071068,  0.0000000, -0.0000000, -0.7071068}, {   0,   -0,  -90}, { 0,  1,  0, -1,  0,  0,  0,  0,  1} }, /* AC 18 */ \
  { { 0.5000000, -0.5000000,  0.5000000, -0.5000000}, { -90,   -0,  -90}, { 0,  0,  1, -1,  0,  0,  0, -1,  0} }, /* AC 19 */ \
  { { 0.0000000,  0.7071068, -0.7071068, -0.0000000}, { 180,   -0,  -90}, { 0, -1,  0, -1,  0,  0,  0,  0, -1} }, /* AC 20 */ \
  { { 0.7071068,  0.0000000,  0.0000000,  0.7071068}, {   0,   -0,   90}, { 0, -1,  0,  1,  0,  0,  0,  0,  1} }, /* AC 21 */ \
  { { 0.5000000,  0.5000000,  0.5000000,  0.5000000}, {  90,   -0,   90}, { 0,  0,  1,  1,  0,  0,  0,  1,  0} }, /* AC 22 */ \
  { { 0.0000000,  0.7071068,  0.7071068,  0.0000000}, { 180,   -0,   90}, { 0,  1,  0,  1,  0,  0,  0,  0, -1} }, /* AC 23 */ \
  { { 0.5000000, -0.5000000, -0.5000000,  0.5000000}, { -90,   -0,   90}, { 0,  0, -1,  1,  0,  0,  0, -1,  0} }  /* AC 24 */ 
};

bool compareQuat(const AttitudeTestData &trueData, SensorSample &sample)
{
  const double quat_tol = 1e-6;

  // For quaternions, -q == +q (encodes the same orientation)
  if ((std::abs(trueData.quat.w - sample.at(MT::Q_W).toDouble()) < quat_tol && \
       std::abs(trueData.quat.x - sample.at(MT::Q_X).toDouble()) < quat_tol && \
       std::abs(trueData.quat.y - sample.at(MT::Q_Y).toDouble()) < quat_tol && \
       std::abs(trueData.quat.z - sample.at(MT::Q_Z).toDouble()) < quat_tol) || \
      (std::abs(trueData.quat.w + sample.at(MT::Q_W).toDouble()) < quat_tol && \
       std::abs(trueData.quat.x + sample.at(MT::Q_X).toDouble()) < quat_tol && \
       std::abs(trueData.quat.y + sample.at(MT::Q_Y).toDouble()) < quat_tol && \
       std::abs(trueData.quat.z + sample.at(MT::Q_Z).toDouble()) < quat_tol))
  {
    return true;
  }
  BOOST_TEST_MESSAGE("QUATERNION ERROR: should be [" << trueData.quat.w << ", " << trueData.quat.x << ", " << trueData.quat.y << ", " << trueData.quat.z << "], is ["\
                      << sample.at(MT::Q_W).toDouble() << ", " << sample.at(MT::Q_X).toDouble() << ", "\
                      << sample.at(MT::Q_Y).toDouble() << ", " << sample.at(MT::Q_Z).toDouble() << "]");
  return false;
}

bool compareEuler(const AttitudeTestData &trueData, SensorSample &sample)
{
  const double euler_tol = 1e-3;

  if (std::abs(trueData.euler.roll - sample.at(MT::ROLL).toDouble()) < euler_tol && \
      std::abs(trueData.euler.pitch - sample.at(MT::PITCH).toDouble()) < euler_tol && \
      std::abs(trueData.euler.heading - sample.at(MT::HEADING).toDouble()) < euler_tol)
  {
    return true;
  }
  BOOST_TEST_MESSAGE("EULER ANGLE ERROR: should be [" << trueData.euler.roll << ", " << trueData.euler.pitch << ", " << trueData.euler.heading << "], is ["\
                      << sample.at(MT::ROLL).toDouble() << ", " << sample.at(MT::PITCH).toDouble() << ", " << sample.at(MT::HEADING).toDouble() << "]");
  return false;
}

bool compareRotmat(const AttitudeTestData &trueData, SensorSample &sample)
{
  const double rotmat_tol = 1e-5;

  if (std::abs(trueData.rotmat.r11 - sample.at(MT::ROTMAT_11).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r12 - sample.at(MT::ROTMAT_12).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r13 - sample.at(MT::ROTMAT_13).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r21 - sample.at(MT::ROTMAT_21).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r22 - sample.at(MT::ROTMAT_22).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r23 - sample.at(MT::ROTMAT_23).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r31 - sample.at(MT::ROTMAT_31).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r32 - sample.at(MT::ROTMAT_32).toDouble()) < rotmat_tol && \
      std::abs(trueData.rotmat.r33 - sample.at(MT::ROTMAT_33).toDouble()) < rotmat_tol)
  {
    return true;
  }
  BOOST_TEST_MESSAGE("ROTMAT ERROR: should be ["\
                     << trueData.rotmat.r11 << ", " << trueData.rotmat.r12 << ", " << trueData.rotmat.r13 << "; "\
                     << trueData.rotmat.r21 << ", " << trueData.rotmat.r22 << ", " << trueData.rotmat.r23 << "; "\
                     << trueData.rotmat.r31 << ", " << trueData.rotmat.r32 << ", " << trueData.rotmat.r33 << "], is [ "\
                     << sample.at(MT::ROTMAT_11).toDouble() << ", " << sample.at(MT::ROTMAT_12).toDouble() << ", " << sample.at(MT::ROTMAT_13).toDouble() << "; "\
                     << sample.at(MT::ROTMAT_21).toDouble() << ", " << sample.at(MT::ROTMAT_22).toDouble() << ", " << sample.at(MT::ROTMAT_23).toDouble() << "; "\
                     << sample.at(MT::ROTMAT_31).toDouble() << ", " << sample.at(MT::ROTMAT_32).toDouble() << ", " << sample.at(MT::ROTMAT_33).toDouble() << "]");
  return false;
}

// Testdata for converting between LLA <-> ECEF
struct PositionTestData
{
  struct
  {
    double lat,lon,alt; // deg,deg,m
  } lla;
  struct
  {
    double x,y,z; // m,m,m
  } ecef;
};

PositionTestData testdata_pos[] = 
{
  // Generated from MATLAB with:
  // [lat,lon] = meshgrid(-80:24:90, 180:-60:-170); alt = 100*((1:numel(lat)) - 1)';
  // for lla = [lat(:), lon(:), alt]'
  //     [x,y,z] = lla2ecef(deg2rad(lla(1)), deg2rad(lla(2)), lla(3)); fprintf("  {{%8.1f, %8.1f, %8.1f}, {%13.3f, %13.3f, %13.3f}},\\\n", lla(1),lla(2),lla(3),x,y,z); 
  // end
  {{   -80.0,    180.0,      0.0}, { -1111164.871,         0.000,  -6259542.961}},\
  {{   -80.0,    120.0,    100.0}, {  -555591.118,    962312.044,  -6259641.442}},\
  {{   -80.0,     60.0,    200.0}, {   555599.800,    962327.083,  -6259739.923}},\
  {{   -80.0,      0.0,    300.0}, {  1111216.965,         0.000,  -6259838.403}},\
  {{   -80.0,    -60.0,    400.0}, {   555617.165,   -962357.159,  -6259936.884}},\
  {{   -80.0,   -120.0,    500.0}, {  -555625.847,   -962372.198,  -6260035.365}},\
  {{   -56.0,    180.0,    600.0}, { -3575177.994,         0.000,  -5264939.659}},\
  {{   -56.0,    120.0,    700.0}, { -1787616.957,   3096243.394,  -5265022.562}},\
  {{   -56.0,     60.0,    800.0}, {  1787644.917,   3096291.821,  -5265105.466}},\
  {{   -56.0,      0.0,    900.0}, {  3575345.752,         0.000,  -5265188.370}},\
  {{   -56.0,    -60.0,   1000.0}, {  1787700.836,  -3096388.676,  -5265271.274}},\
  {{   -56.0,   -120.0,   1100.0}, { -1787728.795,  -3096437.104,  -5265354.178}},\
  {{   -32.0,    180.0,   1200.0}, { -5415075.877,         0.000,  -3361067.337}},\
  {{   -32.0,    120.0,   1300.0}, { -2707580.341,   4689666.716,  -3361120.329}},\
  {{   -32.0,     60.0,   1400.0}, {  2707622.743,   4689740.159,  -3361173.321}},\
  {{   -32.0,      0.0,   1500.0}, {  5415330.292,         0.000,  -3361226.313}},\
  {{   -32.0,    -60.0,   1600.0}, {  2707707.548,  -4689887.046,  -3361279.305}},\
  {{   -32.0,   -120.0,   1700.0}, { -2707749.951,  -4689960.489,  -3361332.297}},\
  {{    -8.0,    180.0,   1800.0}, { -6318257.416,         0.000,   -882030.418}},\
  {{    -8.0,    120.0,   1900.0}, { -3159178.221,   5471857.190,   -882044.336}},\
  {{    -8.0,     60.0,   2000.0}, {  3159227.735,   5471942.949,   -882058.253}},\
  {{    -8.0,      0.0,   2100.0}, {  6318554.497,         0.000,   -882072.170}},\
  {{    -8.0,    -60.0,   2200.0}, {  3159326.762,  -5472114.469,   -882086.088}},\
  {{    -8.0,   -120.0,   2300.0}, { -3159376.275,  -5472200.229,   -882100.005}},\
  {{    16.0,    180.0,   2400.0}, { -6134925.577,         0.000,   1747389.534}},\
  {{    16.0,    120.0,   2500.0}, { -3067510.852,   5313084.648,   1747417.097}},\
  {{    16.0,     60.0,   2600.0}, {  3067558.915,   5313167.896,   1747444.661}},\
  {{    16.0,      0.0,   2700.0}, {  6135213.956,         0.000,   1747472.225}},\
  {{    16.0,    -60.0,   2800.0}, {  3067655.041,  -5313334.391,   1747499.789}},\
  {{    16.0,   -120.0,   2900.0}, { -3067703.104,  -5313417.639,   1747527.352}},\
  {{    40.0,    180.0,   3000.0}, { -4895005.733,         0.000,   4079913.935}},\
  {{    40.0,    120.0,   3100.0}, { -2447541.169,   4239265.658,   4079978.214}},\
  {{    40.0,     60.0,   3200.0}, {  2447579.471,   4239332.000,   4080042.493}},\
  {{    40.0,      0.0,   3300.0}, {  4895235.547,         0.000,   4080106.771}},\
  {{    40.0,    -60.0,   3400.0}, {  2447656.076,  -4239464.682,   4080171.050}},\
  {{    40.0,   -120.0,   3500.0}, { -2447694.378,  -4239531.024,   4080235.329}},\
  {{    64.0,    180.0,   3600.0}, { -2805160.427,         0.000,   5712950.594}},\
  {{    64.0,    120.0,   3700.0}, { -1402602.132,   2429378.155,   5713040.473}},\
  {{    64.0,     60.0,   3800.0}, {  1402624.050,   2429416.119,   5713130.353}},\
  {{    64.0,      0.0,   3900.0}, {  2805291.938,         0.000,   5713220.232}},\
  {{    64.0,    -60.0,   4000.0}, {  1402667.888,  -2429492.047,   5713310.112}},\
  {{    64.0,   -120.0,   4100.0}, { -1402689.806,  -2429530.012,   5713399.991}},\
  {{    88.0,    180.0,   4200.0}, {  -223488.258,         0.000,   6357051.320}},\
  {{    88.0,    120.0,   4300.0}, {  -111745.874,    193549.531,   6357151.259}},\
  {{    88.0,     60.0,   4400.0}, {   111747.619,    193552.554,   6357251.198}},\
  {{    88.0,      0.0,   4500.0}, {   223498.728,         0.000,   6357351.137}},\
  {{    88.0,    -60.0,   4600.0}, {   111751.109,   -193558.598,   6357451.077}},\
  {{    88.0,   -120.0,   4700.0}, {  -111752.854,   -193561.621,   6357551.016}},\
  {{    90.0,      0.0,      0.0}, {        0.000,         0.000,   6356752.314}},\
  {{   -90.0,      0.0,      0.0}, {        0.000,         0.000,  -6356752.314}}
};

bool compareLLA(const PositionTestData &trueData, SensorSample &sample)
{
  const double ll_tol = 1e-6;
  const double alt_tol = 1e-2;

  if (std::abs(trueData.lla.lat - sample.at(MT::POS_LATITUDE).toDouble()) < ll_tol && \
      std::abs(trueData.lla.lon - sample.at(MT::POS_LONGITUDE).toDouble()) < ll_tol && \
      std::abs(trueData.lla.alt - sample.at(MT::POS_VERTICAL).toDouble()) < alt_tol)
  {
    return true;
  }
  BOOST_TEST_MESSAGE("LLA ERROR: should be [" << trueData.lla.lat << ", " << trueData.lla.lon << ", " << trueData.lla.alt << "], is ["\
                      << sample.at(MT::POS_LATITUDE).toDouble() << ", " << sample.at(MT::POS_LONGITUDE).toDouble() << ", " << sample.at(MT::POS_VERTICAL).toDouble() << "]");
  return false;
}

bool compareECEF(const PositionTestData &trueData, SensorSample &sample)
{
  const double tol = 1e-2;

  if (std::abs(trueData.ecef.x - sample.at(MT::ECEF_POS_X).toDouble()) < tol && \
      std::abs(trueData.ecef.y - sample.at(MT::ECEF_POS_Y).toDouble()) < tol && \
      std::abs(trueData.ecef.z - sample.at(MT::ECEF_POS_Z).toDouble()) < tol)
  {
    return true;
  }
  BOOST_TEST_MESSAGE("ECEF ERROR: should be [" << trueData.ecef.x << ", " << trueData.ecef.y << ", " << trueData.ecef.z << "], is ["\
                      << sample.at(MT::ECEF_POS_X).toDouble() << ", " << sample.at(MT::ECEF_POS_Y).toDouble() << ", " << sample.at(MT::ECEF_POS_Z).toDouble() << "]");
  return false;
}

} // namespace

BOOST_AUTO_TEST_CASE(TestInferredData)
{
  SensorDataBackend backend;
  DataUartParseInfo parseInfo;
  DataSelection selection;
  std::vector<uint8_t> input_vector;
  SensorSample sample;

  struct __attribute__ ((packed))
  {
    uint8_t header;
    int32_t qw, qx, qy, qz;
  } input_quat_data;

  struct __attribute__ ((packed))
  {
    uint8_t header;
    int32_t roll, pitch, heading;
  } input_eul_data;

  struct __attribute__ ((packed))
  {
    uint8_t header;
    int32_t r11, r12, r13, r21, r22, r23, r31, r32, r33;
  } input_rotmat_data;

  struct __attribute__((packed))
  {
    uint8_t header;
    int32_t lat, lon, alt;
  } input_lla_data;

  struct __attribute__((packed))
  {
    uint8_t header;
    int32_t x,y,z;
  } input_ecef_data;

  input_quat_data.header = 0xFA;
  input_eul_data.header = 0xFA;
  input_rotmat_data.header = 0xFA;
  input_lla_data.header = 0xFA;
  input_ecef_data.header = 0xFA;

  for (std::size_t i = 0; i < sizeof(testdata_att)/sizeof(testdata_att[0]); i++)
  {
    const auto testsample = testdata_att[i];

#if 1
    // Send in quaternion, should get correct inferred euler angles and rotmat +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    selection = backend.parseDataUartConfigString("o000Ar044CF4DF4EF4FF", parseInfo);

    input_quat_data.qw = static_cast<int32_t>(std::round(1e6*testsample.quat.w));
    input_quat_data.qx = static_cast<int32_t>(std::round(1e6*testsample.quat.x));
    input_quat_data.qy = static_cast<int32_t>(std::round(1e6*testsample.quat.y));
    input_quat_data.qz = static_cast<int32_t>(std::round(1e6*testsample.quat.z));

    input_vector.assign((uint8_t*)&input_quat_data, (uint8_t*)&input_quat_data + sizeof(input_quat_data));
    BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));
    sample = backend.getSensorSample(selection);
    //for (auto it = sample.begin(); it != sample.end(); ++it) BOOST_TEST_MESSAGE(it->first);

    BOOST_CHECK_MESSAGE(compareQuat(testsample, sample), "quat in, quat out @" << i);
    BOOST_CHECK(!sample.at(MT::Q_W).isInferred &&
                !sample.at(MT::Q_X).isInferred &&
                !sample.at(MT::Q_Y).isInferred &&
                !sample.at(MT::Q_Z).isInferred);

    BOOST_CHECK_MESSAGE(compareEuler(testsample, sample), "quat in, euler out @" << i);
    BOOST_CHECK(sample.at(MT::ROLL).isInferred &&
                sample.at(MT::PITCH).isInferred &&
                sample.at(MT::HEADING).isInferred);

    BOOST_CHECK_MESSAGE(compareRotmat(testsample, sample), "quat in, rotmat out @" << i);
    BOOST_CHECK(sample.at(MT::ROTMAT_11).isInferred && sample.at(MT::ROTMAT_12).isInferred && sample.at(MT::ROTMAT_13).isInferred && \
                sample.at(MT::ROTMAT_21).isInferred && sample.at(MT::ROTMAT_22).isInferred && sample.at(MT::ROTMAT_23).isInferred && \
                sample.at(MT::ROTMAT_31).isInferred && sample.at(MT::ROTMAT_32).isInferred && sample.at(MT::ROTMAT_33).isInferred);
#else
  (void)input_quat_data;
#endif

#if 1
    // Send in euler angles, should get correct inferred quaternion and rotmat +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    selection = backend.parseDataUartConfigString("o000Ar0337F38F39F", parseInfo);

    input_eul_data.roll = static_cast<int32_t>(std::round(1e6*testsample.euler.roll));
    input_eul_data.pitch = static_cast<int32_t>(std::round(1e6*testsample.euler.pitch));
    input_eul_data.heading = static_cast<int32_t>(std::round(1e6*testsample.euler.heading));

    input_vector.assign((uint8_t*)&input_eul_data, (uint8_t*)&input_eul_data + sizeof(input_eul_data));
    BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));
    sample = backend.getSensorSample(selection);

    BOOST_CHECK_MESSAGE(compareQuat(testsample, sample), "euler in, quat out @" << i);
    BOOST_CHECK(sample.at(MT::Q_W).isInferred &&
                sample.at(MT::Q_X).isInferred &&
                sample.at(MT::Q_Y).isInferred &&
                sample.at(MT::Q_Z).isInferred);

    BOOST_CHECK_MESSAGE(compareEuler(testsample, sample), "euler in, euler out @" << i);
    BOOST_CHECK(!sample.at(MT::ROLL).isInferred &&
                !sample.at(MT::PITCH).isInferred &&
                !sample.at(MT::HEADING).isInferred);

    BOOST_CHECK_MESSAGE(compareRotmat(testsample, sample), "euler in, rotmat out @" << i);
    BOOST_CHECK(sample.at(MT::ROTMAT_11).isInferred &&
                sample.at(MT::ROTMAT_12).isInferred &&
                sample.at(MT::ROTMAT_13).isInferred &&
                sample.at(MT::ROTMAT_21).isInferred &&
                sample.at(MT::ROTMAT_22).isInferred &&
                sample.at(MT::ROTMAT_23).isInferred &&
                sample.at(MT::ROTMAT_31).isInferred &&
                sample.at(MT::ROTMAT_32).isInferred &&
                sample.at(MT::ROTMAT_33).isInferred);
#else
  (void)input_eul_data;
#endif
    
#if 1
    // Send in rotmat, should get correct inferred quaternion and euler angles +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    selection = backend.parseDataUartConfigString("o000Ar0950F51F52F53F54F55F56F57F58F", parseInfo);

    input_rotmat_data.r11 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r11));
    input_rotmat_data.r12 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r12));
    input_rotmat_data.r13 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r13));
    input_rotmat_data.r21 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r21));
    input_rotmat_data.r22 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r22));
    input_rotmat_data.r23 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r23));
    input_rotmat_data.r31 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r31));
    input_rotmat_data.r32 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r32));
    input_rotmat_data.r33 = static_cast<int32_t>(std::round(1e6*testsample.rotmat.r33));

    input_vector.assign((uint8_t*)&input_rotmat_data, (uint8_t*)&input_rotmat_data + sizeof(input_rotmat_data));
    BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));
    sample = backend.getSensorSample(selection);

    BOOST_CHECK_MESSAGE(compareQuat(testsample, sample), "rotmat in, quat out @" << i);
    BOOST_CHECK(sample.at(MT::Q_W).isInferred &&
                sample.at(MT::Q_X).isInferred &&
                sample.at(MT::Q_Y).isInferred &&
                sample.at(MT::Q_Z).isInferred);

    BOOST_CHECK_MESSAGE(compareEuler(testsample, sample), "rotmat in, euler out @" << i);
    BOOST_CHECK(sample.at(MT::ROLL).isInferred &&
                sample.at(MT::PITCH).isInferred &&
                sample.at(MT::HEADING).isInferred);

    BOOST_CHECK_MESSAGE(compareRotmat(testsample, sample), "rotmat in, rotmat out @" << i);
    BOOST_CHECK(!sample.at(MT::ROTMAT_11).isInferred && !sample.at(MT::ROTMAT_12).isInferred && !sample.at(MT::ROTMAT_13).isInferred && \
                !sample.at(MT::ROTMAT_21).isInferred && !sample.at(MT::ROTMAT_22).isInferred && !sample.at(MT::ROTMAT_23).isInferred && \
                !sample.at(MT::ROTMAT_31).isInferred && !sample.at(MT::ROTMAT_32).isInferred && !sample.at(MT::ROTMAT_33).isInferred);
#else
  (void)input_rotmat_data;
#endif
  }

  for (std::size_t i = 0; i < sizeof(testdata_pos)/sizeof(testdata_pos[0]); i++)
  {
    const auto testsample = testdata_pos[i];
#if 1
    // Send in LLA, check ECEF +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    selection = backend.parseDataUartConfigString("o000Ar0331F32F35F", parseInfo);

    input_lla_data.lat = static_cast<int32_t>(std::round(1e7*testsample.lla.lat));
    input_lla_data.lon = static_cast<int32_t>(std::round(1e7*testsample.lla.lon));
    input_lla_data.alt = static_cast<int32_t>(std::round(1e3*testsample.lla.alt));

    input_vector.assign((uint8_t*)&input_lla_data, (uint8_t*)&input_lla_data + sizeof(input_lla_data));
    BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));
    sample = backend.getSensorSample(selection);

    BOOST_CHECK_MESSAGE(compareLLA(testsample, sample), "lla in, lla out @" << i);
    BOOST_CHECK(!sample.at(MT::POS_LATITUDE).isInferred &&
                !sample.at(MT::POS_LONGITUDE).isInferred &&
                !sample.at(MT::POS_VERTICAL).isInferred);

    BOOST_CHECK_MESSAGE(compareECEF(testsample, sample), "lla in, ecef out @" << i);
    BOOST_CHECK(sample.at(MT::ECEF_POS_X).isInferred &&
                sample.at(MT::ECEF_POS_Y).isInferred &&
                sample.at(MT::ECEF_POS_Z).isInferred);
#else
    (void)input_lla_data;
#endif

#if 1
    // Send in ECEF, check LLA +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    selection = backend.parseDataUartConfigString("o000Ar0359F5AF5BF", parseInfo);

    input_ecef_data.x = static_cast<int32_t>(std::round(1e2*testsample.ecef.x));
    input_ecef_data.y = static_cast<int32_t>(std::round(1e2*testsample.ecef.y));
    input_ecef_data.z = static_cast<int32_t>(std::round(1e2*testsample.ecef.z));

    input_vector.assign((uint8_t*)&input_ecef_data, (uint8_t*)&input_ecef_data + sizeof(input_ecef_data));
    BOOST_REQUIRE(backend.parseDataUartData(input_vector, parseInfo));
    sample = backend.getSensorSample(selection);

    BOOST_CHECK_MESSAGE(compareECEF(testsample, sample), "ecef in, ecef out @" << i);
    BOOST_CHECK(!sample.at(MT::ECEF_POS_X).isInferred &&
                !sample.at(MT::ECEF_POS_Y).isInferred &&
                !sample.at(MT::ECEF_POS_Z).isInferred);

    BOOST_CHECK_MESSAGE(compareLLA(testsample, sample), "ecef in, lla out @" << i);
    BOOST_CHECK(sample.at(MT::POS_LATITUDE).isInferred &&
                sample.at(MT::POS_LONGITUDE).isInferred &&
                sample.at(MT::POS_VERTICAL).isInferred);
#else
    (void)input_ecef_data;
#endif
  }

}

BOOST_AUTO_TEST_SUITE_END();
