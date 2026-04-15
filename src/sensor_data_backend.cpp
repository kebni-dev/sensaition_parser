#include <iomanip>
#include <sstream>
#include <iostream>
#include <cassert>
#include <cmath>
#include <algorithm>

#include "sensaition_parser/sensor_data_backend.hpp"

using namespace sepa;

constexpr std::size_t CAN_CFG_LEN = 512;       // Maximum length of the data uart config string (includes null-terminator)
constexpr std::size_t DATA_UART_CFG_LEN = 256; // Maximum length of the data uart config string (includes null-terminator)
constexpr std::size_t USER_UART_CFG_LEN = 32;  // Maximum length of the user uart config string (includes null-terminator)

SensorDataBackend::SensorDataBackend(void)
{
  populateSensorData();
}

SensorDataBackend::~SensorDataBackend(void)
{
  for (auto value : values)
  {
    if (value != nullptr)
    {
      delete value;
    }
  }
}

/// @brief Returns a sensor sample including all recently received measurements
SensorSample SensorDataBackend::getSensorSample(void)
{
  SensorSample sample;
  for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
  {
    auto value = values.at(idx);
    if (value != nullptr && value->parsed)
    {
      value->insertMeasurementValue(&sample);
    }
  }
  return sample;
}

/// @brief Returns a sensor sample including all measurements in selection
/// @param selection Measurements to include in sample
SensorSample SensorDataBackend::getSensorSample(const DataSelection selection)
{
  SensorSample sample;
  for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
  {
    auto value = values.at(idx);
    if (value != nullptr && (selection.test(idx) || value->inferred)) // always include potential inferred variables
    {
      value->insertMeasurementValue(&sample);
    }
  }
  return sample;
}

/// @brief Generates a user uart configuration string based on the selected user uart entries
/// @param selected Bitmask declaring which data elements are selected to be included in the config string
/// @param nmeaSelection Standard NMEA messages to include in the config string
/// @throws std::invalid_argument if config unable to be generated
/// @return The generated config string with the elements ordered in the same way as in the csv logs
std::string SensorDataBackend::getUserUartConfigString(const DataSelection& selected, const StandardNmeaSelection& nmeaSelection) const
{
  // String is a comma-delimited list of indices (in decimal), ex "0,1,2,3,4,5"

  std::ostringstream cfg; // output 
  cfg << std::dec;
  
  // Order selected data string with highest priority first
  for (auto idx : values_priority)
  {
    auto value = values[idx];
    if (idx < SensorDataValueId::SENSOR_DATA_NUM && value != nullptr && selected.test(idx))
    {
      if (cfg.tellp() > 0) cfg << ",";
      cfg << idx;
    }
  }

  if (!nmeaSelection.empty())
  {
    // Also include standard NMEA data in output...
    for (int msg_id = (int)StandardNmeaMessages::STANDARD_NMEA_GGA; msg_id < StandardNmeaMessages::STANDARD_NMEA_NUM; msg_id++)
    {
      for (int msg_src = (int)StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV; msg_src < StandardNmeaSources::STANDARD_NMEA_SOURCE_NUM; msg_src++)
      {
        if (nmeaSelection.get(static_cast<StandardNmeaMessages>(msg_id), static_cast<StandardNmeaSources>(msg_src)))
        {
          if (cfg.tellp() > 0) cfg << ",";
          cfg << nmea_identifiers.at(msg_id);
          if (msg_src != StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV) cfg << msg_src;
        }
      }
    }
  }

  std::streampos str_size = cfg.tellp();
  if (str_size >= static_cast<std::streampos>(USER_UART_CFG_LEN))
    throw std::invalid_argument(std::string("Config string too long (") + std::to_string(str_size) + "), max length = " + std::to_string(USER_UART_CFG_LEN));
  // Before fw version 543 only supports 8 indexes, has been upped to 32 since the 
  // but will hit the string length limit first, so don't bother checking for this limit

  return cfg.str();
}

/// @brief Generates a user uart configuration string based on the selected user uart entries
/// @param selected Bitmask declaring which data elements are selected to be included in the config string
/// @throws std::invalid_argument if config unable to be generated
/// @return The generated config string with the elements ordered in the same way as in the csv logs
std::string SensorDataBackend::getUserUartConfigString(const DataSelection& selected) const
{
  StandardNmeaSelection nmeaSelection;
  return getUserUartConfigString(selected, nmeaSelection);
}

/// @brief Generates a data uart configuration string for a single message with the provided data
/// @param message DataUartMessage describing the data to include
/// @throws std::invalid_argument if config unable to be generated
/// @return The generated config string with the elements ordered in the same way as in the csv logs
std::string SensorDataBackend::getDataUartConfigString(const DataUartMessage& message) const
{
  std::ostringstream cfg;
  // Print numbers in hex and uppercase letters and fill up with 0:s to get fixed width
  cfg << std::hex << std::uppercase << std::setfill('0');

  if (message.dataRate < 1 || message.dataRate > 65535)
    throw std::invalid_argument(std::string("Invalid data rate selection (")
                                  + std::to_string(message.dataRate) + std::string(")"));
  if (message.checksumBytes < 0 || message.checksumBytes > 2)
    throw std::invalid_argument(std::string("Invalid checksum length (")
                                  + std::to_string(message.checksumBytes) + std::string(")"));
  if (message.numIdentifier < 0 || message.numIdentifier > 4)
    throw std::invalid_argument(std::string("Invalid number of identifiers (")
                                  + std::to_string(message.numIdentifier) + std::string(")"));

  // Write data rate
  cfg << 'o' << std::setw(4) << message.dataRate;

  // Write identifier(s)
  for (int i = 0; i < message.numIdentifier; i++)
  {
    int offset = 8*(message.numIdentifier - 1) - i*8; // MSB first
    cfg << 'i' << std::setw(2) << ((message.identifier >> offset) & 0xFF);
  }

  // Calculate total number of bytes
  int n_bytes = 0;
  int n_entries = 0;
  for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
  {
    auto value = values.at(idx);
    if (value != nullptr && message.selection.test(idx))
    {
      n_bytes += value->getSize();
      n_entries++;
    }
  }

  // Write data header
  if (message.useCompact)
    cfg << (message.useBigEndian ? 'R' : 'r') << std::setw(2) << n_entries;
  else
    cfg << 's' << std::setw(2) << n_bytes;
  
  // Order selected data in string with highest priority first
  for (auto idx : values_priority)
  {
    auto value = values[idx];
    if (idx < SensorDataValueId::SENSOR_DATA_NUM && value != nullptr && message.selection.test(idx))
    {
      uint32_t byteMask = value->getByteMask();
      if (message.useCompact)
      {
        cfg << std::setw(2) << idx << std::setw(1) << byteMask;
      }
      else for (int i = 0; i < 4; i++)
      {
        int byteIndex = message.useBigEndian ? (3 - i) : i;
        if ((1 << byteIndex) & byteMask)
          cfg << std::setw(2) << idx << std::setw(1) << byteIndex;
      }
    }
  }

  // Customizable checksum
  if (message.checksumBytes == 1)
    cfg << "x";
  else if (message.checksumBytes == 2)
    cfg << "X";
  // Fallback to no checksum

  // Check that the string is below the required size
  std::streampos str_size = cfg.tellp();
  if (str_size < 0 || str_size >= static_cast<std::streampos>(DATA_UART_CFG_LEN))
    throw std::invalid_argument(std::string("Config string length too long (") + std::to_string(str_size)
                                  + std::string("), max length = ") + std::to_string(DATA_UART_CFG_LEN));

  return cfg.str();
}

/// @brief Generates a data uart configuration string with multiple input messages
/// @param message DataUartMessageSet with data for each message to include
/// @throws std::invalid_argument if config unable to be generated
/// @return The generated config string with the elements ordered in the same way as in the csv logs
std::string SensorDataBackend::getDataUartConfigString(const DataUartMessageSet& messages) const
{
  // Error check input
  if (messages.numMessages < 1 || messages.numMessages > SENSOR_DATA_NUM_MESSAGES)
    throw std::invalid_argument(std::string("Invalid number of messages (") + std::to_string(messages.numMessages) + std::string(")"));
  for (int i = 1; i < messages.numMessages; i++)
  {
    if (messages.message[i].numIdentifier == 0 || messages.message[i-1].numIdentifier == 0)
      throw std::invalid_argument("Require identifiers when using multiple messages");
    if (messages.message[i].numIdentifier != messages.message[i-1].numIdentifier)
      throw std::invalid_argument("For multiple messages, require all messages to have the same number of identifiers");
    for (int j = 0; j < i; j++)
      if (messages.message[i].identifier == messages.message[j].identifier)
        throw std::invalid_argument("For multiple messages, require all messages to have unique identifiers");
  }

  // Assemble string
  std::ostringstream cfg;
  for (int i = 0; i < messages.numMessages; i++)
    cfg << getDataUartConfigString(messages.message[i]);

  // Check that the string is below the required size
  std::streampos str_size = cfg.tellp();
  if (str_size < 0 || str_size >= static_cast<std::streampos>(DATA_UART_CFG_LEN))
    throw std::invalid_argument(std::string("Config string length too long (") + std::to_string(str_size)
                                  + std::string("), max length = ") + std::to_string(DATA_UART_CFG_LEN));
  
  return cfg.str();
}

/// @brief Generates a data uart configuration string based on the selected entries
/// @param selected Bitmask declaring which data elements are selected to be included in the config string
/// @param rateDivisor Output rate, 1000 Hz divisor
/// @param checksumLength Number of checksum bytes, can be 0 (no checksum), 1 (8-bit checksum) or 2 (16-bit checksum)
/// @param useBigEndian true if we should send bytes in big endian order, false if we should send in little endian
/// @param compact If true, uses the new compact config string format, if false uses the standard format
/// @throws std::invalid_argument if config unable to be generated
/// @return The generated config string with the elements ordered in the same way as in the csv logs
std::string SensorDataBackend::getDataUartConfigString(const DataSelection& selected, int rateDivisor, int checksumLength,
                                                       bool useBigEndian, bool compact) const
{
  // Legacy command interface, todo: remove?
  DataUartMessage message = {
    .selection = selected, 
    .dataRate = rateDivisor, 
    .checksumBytes = checksumLength,
    .numIdentifier = 0,
    .identifier = 0,
    .useBigEndian = useBigEndian,
    .useCompact = compact
  };
  return getDataUartConfigString(message);
}

/// @brief Generates a can configuration string containing all the messages
/// @param messages vector of messages
/// @throws std::invalid_argument if invalid parameters
/// @return The generated config string
std::string SensorDataBackend::getCanConfigString(const std::vector<CanMessage>& messages) const
{
  std::ostringstream cfg;
  // Print numbers in hex and uppercase letters and fill up with 0:s to get fixed width
  cfg << std::hex << std::uppercase << std::setfill('0');

  int dataRate = -1;
  int phase = 0;
  int n_bytes;
  
  // Go through all messages
  for (auto& message : messages)
  {
    if (message.dataRate != dataRate)
    {
      // Push new data rate
      if (message.dataRate < 1 || message.dataRate > 65535)
        throw std::invalid_argument(std::string("Invalid data rate selection (") + std::to_string(message.dataRate) + std::string(")"));

      cfg << 'o' << std::setw(4) << message.dataRate;
      dataRate = message.dataRate;
      phase = 0;
    }

    if (message.phase != phase)
    {
      // Push new phase
      if (message.phase < 0 || message.phase > 255)
        throw std::invalid_argument(std::string("Invalid phase selection (") + std::to_string(message.phase) + std::string(")"));
      
      cfg << 'p' << std::setw(2) << message.phase;
      phase = message.phase;
    }

    // Construct identifier
    if (message.idType == CanIdentifierTypes::CAN_STANDARD_ID)
    {
      if (message.identifier > 0x7FF)
        throw std::invalid_argument(std::string("Standard identifier > 11 bits (") + std::to_string(message.identifier) + std::string(")"));
      cfg << "t" << std::setw(3) << message.identifier;
    }
    else if (message.idType == CanIdentifierTypes::CAN_EXTENDED_ID)
    {
      if (message.identifier > 0x1FFFFFFF)
        throw std::invalid_argument(std::string("Extended identifier > 29 bits (") + std::to_string(message.identifier) + std::string(")"));
      cfg << "E" << std::setw(8) << message.identifier;
    }
    else if (message.idType == CanIdentifierTypes::CAN_EXTENDED_ID_WITH_CHIP_ID)
    {
      if (message.identifier > 0x1F)
        throw std::invalid_argument(std::string("Extended identifier with chip id > 5 bits (") + std::to_string(message.identifier) + std::string(")"));
      cfg << "T" << std::setw(2) << message.identifier;
    }
    else
    {
      throw std::invalid_argument(std::string("Unknown idType (") + std::to_string(message.idType) + std::string(")")); 
    }
    // todo: make sure unique id's?

    // Set message size
    n_bytes = 0;
    for (std::size_t idx = 0; idx < SensorDataValueId::SENSOR_DATA_NUM; idx++)
    {
      auto value = values.at(idx);
      if (value != nullptr && message.selection.test(idx))
        n_bytes += value->getSize();
    }
    if (n_bytes > 8)
      throw std::invalid_argument(\
        std::string("Too many data bytes in message with id ") + std::to_string(message.identifier) + \
        std::string(", can only fit 8 bytes, got ") + std::to_string(n_bytes) + std::string(" bytes"));
    cfg << std::setw(1) << n_bytes;

    // Set message content
    for (auto idx : values_priority) // Order selected data in string with highest priority first
    {
      auto value = values.at(idx);
      if (idx < SensorDataValueId::SENSOR_DATA_NUM && value != nullptr && message.selection.test(idx))
      {
        uint32_t byteMask = value->getByteMask();
        for (int i = 0; i < 4; i++)
        {
          int byteIndex = message.useBigEndian ? (3 - i) : i;
          if ((1 << byteIndex) & byteMask)
            cfg << std::setw(2) << idx << std::setw(1) << byteIndex;
        }
      }
    }
  }

  // Check that the string is below the required size
  std::streampos str_size = cfg.tellp();
  if (str_size < 0 || str_size >= static_cast<std::streampos>(CAN_CFG_LEN))
    throw std::invalid_argument(std::string("Config string length too long (") + std::to_string(str_size) + std::string("), max length = ") + std::to_string(CAN_CFG_LEN));
  // todo: max 16-message limit?
  
  return cfg.str();
}

/// @brief Parses a User Uart config string
/// @param str string containing the user uart config (contents of user reg 5)
/// @param[out] nmeaSelection optional nmea selection which stores standard nmea identifiers located in the string
/// @throws std::invalid_argument if string is invalid
/// @return Bitmask declaring which data elements were selected in the string
DataSelection SensorDataBackend::parseUserUartConfigString(const std::string& str, StandardNmeaSelection& nmeaSelection) const
{
  DataSelection selection;
  if (str.length() >= USER_UART_CFG_LEN)
    throw std::invalid_argument(std::string("User UART string too long, max string length is ") + std::to_string(USER_UART_CFG_LEN) + \
                                            std::string(", this string has length ")  + std::to_string(str.length()));

  selection.reset();
  nmeaSelection.clear_all();

  std::istringstream iss(str);
  for (std::string token; std::getline(iss, token, ',');)
  {
    // Try parse as integer consuming the whole token
    try 
    {
      std::size_t pos;
      int idx = std::stoi(token, &pos, 10);
      if (pos == token.size() && idx >= 0 && idx < SensorDataValueId::SENSOR_DATA_NUM)
      {
        // Entry is an integer sensor data value idx
        if (values[idx] != nullptr)
          selection.set(idx, true);
        continue; // go to next token
      }
    } 
    catch (const std::exception&)
    {
      // Couldn't parse it as an integer, go to next step
    }

    // Try parse standard NMEA entry, exactly 3 letters + optional selector token (0 - 2)
    // e.g. GGA, RMC0, GLL1, GST2 etc...
    if (token.size() == 3 || (token.size() == 4 && token[3] >= '0' && token[3] <= '2') )
    {
      StandardNmeaSources msg_src = (token.size() == 4) ? static_cast<StandardNmeaSources>(token[3] - '0') : StandardNmeaSources::STANDARD_NMEA_SOURCE_NAV;
      StandardNmeaMessages msg_id = StandardNmeaMessages::STANDARD_NMEA_NUM;
      if (token.compare(0, 3, "GGA") == 0) msg_id = StandardNmeaMessages::STANDARD_NMEA_GGA;
      else if (token.compare(0, 3, "RMC") == 0) msg_id = StandardNmeaMessages::STANDARD_NMEA_RMC;
      else if (token.compare(0, 3, "GLL") == 0) msg_id = StandardNmeaMessages::STANDARD_NMEA_GLL;
      else if (token.compare(0, 3, "GST") == 0) msg_id = StandardNmeaMessages::STANDARD_NMEA_GST;

      if (msg_id < StandardNmeaMessages::STANDARD_NMEA_NUM && msg_src < StandardNmeaSources::STANDARD_NMEA_SOURCE_NUM)
      {
        // VALID PARSE
        nmeaSelection.set(msg_id, msg_src);
        continue; // go to next token
      }
    }

    // Got here without a valid match, report error
    throw std::invalid_argument(std::string("Invalid data entry \"") + token + std::string("\""));
  }

  return selection;
}

/// @brief Parses a User Uart config string
/// @param str string containing the user uart config (contents of user reg 5)
/// @throws std::invalid_argument if string is invalid
/// @return Bitmask declaring which data elements were selected in the string
DataSelection SensorDataBackend::parseUserUartConfigString(const std::string &str) const
{
  StandardNmeaSelection nmeaSelection;
  return parseUserUartConfigString(str, nmeaSelection);
}

/// @brief Parses the data uart config string and populates the parseInfo with the result
/// @param str The config string
/// @param parseInfo Resulting parse info (will popoulate every relevant value)
/// @param selection data selection which can be used by getDataUartConfigString to regenerate this string 
///                  (only true if this string is already generated by getDataUartConfigString).
///                  NOT updated if this function returns early.
/// @throws std::invalid_argument if string is invalid
/// @return Bitmask of all data elements included in the string
DataSelection SensorDataBackend::parseDataUartConfigString(const std::string& str, DataUartParseInfo& parseInfo, DataUartMessageSet& selection) const
{
  // reset params
  parseInfo.numMessages = 0;
  uint16_t numMessages = 0; // local assignment, only assign to parseInfo before returning at the end
  
  // NOTE: This could be ambiguous, since it's possible to write the config string with
  // different byte order for different values. 
  // For standard data, we switch this to false if we find
  // AT LEAST one example of non-big endian byte order in the config string.
  // This will work correctly if the endianness is consistent in the whole string.
  // For compact data it will reflect the endianness of the last parsed message.
  parseInfo.usingBigEndian = true;

  bool usingCompactMode = false;

  int previousRow = -1; // -1 means not initialized
  int previousCol = 0;

  std::istringstream ss(str); // Stringstream for managing input

  int byte_length;
  char str_switch;

  DataUartParseInfo::DataUartParsedMessage* message = &parseInfo.messages[0]; // Pointer to currently handler message
  int data_idx = 0; // index into parseInfo.Data

  if (ss.peek() != 'o')
  {
    // Doesn't start with an 'o' switch, push a message with default data rate
    message = &parseInfo.messages[numMessages];
    numMessages++;
    message->checksumBytes = 0;
    message->startIndex = 0;
    message->numBytes = 0;
    message->numIdentifier = 0;
    message->identifier = 0;
    message->dataRate = 10; // default to 100 Hz
  }

  while (ss >> str_switch) // this ignores whitespace but w/e...
  {
    switch (str_switch)
    {
    case 'o': // data rate & new message
      if (numMessages >= SENSOR_DATA_NUM_MESSAGES) 
        throw std::invalid_argument(std::string("Only supports ") + std::to_string(SENSOR_DATA_NUM_MESSAGES) + std::string(" messages"));
      
      // Push new message
      message = &parseInfo.messages[numMessages];
      numMessages++;

      // Default values
      message->checksumBytes = 0;
      message->startIndex = (uint16_t)data_idx;
      message->numBytes = 0;
      message->numIdentifier = 0;
      message->identifier = 0;

      // Parse its data rate
      message->dataRate = char_to_hex(ss.get());
      message->dataRate = (message->dataRate << 4) | char_to_hex(ss.get());
      message->dataRate = (message->dataRate << 4) | char_to_hex(ss.get());
      message->dataRate = (message->dataRate << 4) | char_to_hex(ss.get());
      if (message->dataRate == 0) throw std::invalid_argument("Data-rate divisor cannot be 0");

      previousRow = -1; // reset endianess detection
      break;
    
    case 's': // standard data format
      if (message->checksumBytes > 0) throw std::invalid_argument("Cannot have data bytes defined after a checksum");

      byte_length = char_to_hex(ss.get());
      byte_length = (byte_length << 4) | char_to_hex(ss.get());

      while (byte_length--)
      {
        if (data_idx >= SENSOR_DATA_BYTES) throw std::invalid_argument("Maximum number of bytes reached");

        parseInfo.data[data_idx].row = char_to_hex(ss.get());
        parseInfo.data[data_idx].row = (parseInfo.data[data_idx].row << 4) | char_to_hex(ss.get());
        parseInfo.data[data_idx].col = char_to_hex(ss.get());
        if (parseInfo.data[data_idx].row >= SensorDataValueId::SENSOR_DATA_NUM)
          throw std::invalid_argument(std::string("Data value index \"") + std::to_string(parseInfo.data[data_idx].row) + std::string("\" out of bounds"));
        if (parseInfo.data[data_idx].col >= 4)
          throw std::invalid_argument(std::string("Byte index \"") + std::to_string(parseInfo.data[data_idx].col) + std::string("\" out of bounds"));

        // Here we have valid values for row and col, so we can estimate endianness
        bool sameMeasurementValue = static_cast<int>(parseInfo.data[data_idx].row) == previousRow;
        bool increasingByteIndex = static_cast<int>(parseInfo.data[data_idx].col) > previousCol;
        if (sameMeasurementValue && increasingByteIndex)
          parseInfo.usingBigEndian = false;
        previousRow = static_cast<int>(parseInfo.data[data_idx].row);
        previousCol = static_cast<int>(parseInfo.data[data_idx].col);

        data_idx++;
        message->numBytes++;
      }
      break;
    
    case 'r': // Compact format little endian
    case 'R': // Compact format big endian
      parseInfo.usingBigEndian = (str_switch == 'R'); // will only reflect endianess of last r/R switch

      if (message->checksumBytes > 0) throw std::invalid_argument("Cannot have data bytes defined after a checksum");

      byte_length = char_to_hex(ss.get()); // Technically token length
      byte_length = (byte_length << 4) | char_to_hex(ss.get());

      while (byte_length--)
      {
        uint8_t row, mask, col;
        row = char_to_hex(ss.get());
        row = (row << 4) | char_to_hex(ss.get());
        if (row >= SensorDataValueId::SENSOR_DATA_NUM)
          throw std::invalid_argument(std::string("Data value index \"") + std::to_string(row) + std::string("\" out of bounds"));

        mask = char_to_hex(ss.get());

        col = parseInfo.usingBigEndian ? 3 : 0;
        while (col < 4)
        {
          if (mask & (1 << col))
          {
            if (data_idx >= SENSOR_DATA_BYTES) throw std::invalid_argument("Maximum number of bytes reached");
            parseInfo.data[data_idx].row = row;
            parseInfo.data[data_idx].col = col;
            data_idx++;
            message->numBytes++;
          }
          col = parseInfo.usingBigEndian ? (col - 1) : (col + 1);
        }
      }
      usingCompactMode = true;
      break;
    
    case 'i': // identifier
      if (message->numBytes > 0) throw std::invalid_argument("Doesn't support identifers after data");
      if (message->numIdentifier >= 4) throw std::invalid_argument("Only supports up to 4 identifier bytes");
      message->identifier = (message->identifier << 4) | (uint32_t)char_to_hex(ss.get());
      message->identifier = (message->identifier << 4) | (uint32_t)char_to_hex(ss.get());
      message->numIdentifier++;
      // todo: trailing identifiers for custom end bytes?
      break;

    case 'x': // 8-bit checksum
      if (message->checksumBytes > 0) throw std::invalid_argument("Cannot have multiple checksums");
      message->checksumBytes = 1;
      break;
    
    case 'X': // 16-bit crc
      if (message->checksumBytes > 0) throw std::invalid_argument("Cannot have multiple checksums");
      message->checksumBytes = 2;
      break;
    
    case 'n': // NAV02S format switch (todo: support?)
    default:
      throw std::invalid_argument(std::string("Unknown switch \"" + std::string(&str_switch,1) + std::string("\"")));
    }
  }

  if (numMessages > 1)
  {
    // When using multiple messages, check identifier validity.
    // Needs to exist, have the same size and no duplicates.
    for (int i = 1; i < numMessages; i++)
    {
      for (int j = 0; j < i; j++)
      {
        if (parseInfo.messages[i].numIdentifier == 0 || parseInfo.messages[j].numIdentifier == 0)
          throw std::invalid_argument("Requires identifiers to be able to properly parse interleaved messages");
        if (parseInfo.messages[i].numIdentifier != parseInfo.messages[j].numIdentifier)
          throw std::invalid_argument("Requires identifiers of different messages to be the same size");
        if (parseInfo.messages[i].identifier == parseInfo.messages[j].identifier)
          throw std::invalid_argument("Different messages share the same identifier");
      }
    }
  }

  // Set the numMessages entry after all error checks are done so that any early
  // return will invalidate parseInfo by keeping parseInfo.numMessages = 0
  parseInfo.numMessages = numMessages;

  // Populate which sensor data values are included in this string
  // Note that this isn't a perfect replica of the indata, we can technically parse any incoming indata
  // but when generating strings we require some consistency across the different messages...
  selection.numMessages = numMessages;
  for (int i = 0; i < numMessages; i++)
  {
    selection.message[i].dataRate = parseInfo.messages[i].dataRate;
    selection.message[i].numIdentifier = parseInfo.messages[i].numIdentifier;
    selection.message[i].identifier = parseInfo.messages[i].identifier;
    selection.message[i].checksumBytes = parseInfo.messages[i].checksumBytes;
    selection.message[i].useBigEndian = parseInfo.usingBigEndian;
    selection.message[i].useCompact = usingCompactMode;

    selection.message[i].selection.reset();
    for (int j = 0; j < parseInfo.messages[i].numBytes; j++)
    {
      const int data_idx = parseInfo.data[parseInfo.messages[i].startIndex + j].row;
      if (data_idx < SensorDataValueId::SENSOR_DATA_NUM && values[data_idx] != nullptr)
        selection.message[i].selection.set(data_idx, true);
    }
  }

  // return a OR of all values included in the message sets
  DataSelection retval;
  for (int i = 0; i < selection.numMessages; i++)
    retval |= selection.message[i].selection;
  return retval;
}

/// @brief Parses the data uart config string and populates the parseInfo with the result
/// @param str The config string
/// @param parseInfo Resulting parse info (will popoulate every relevant value)
/// @throws std::invalid_argument if string is invalid
/// @return Bitmask of all data elements included in the string
DataSelection SensorDataBackend::parseDataUartConfigString(const std::string& str, DataUartParseInfo& parseInfo) const
{
  DataUartMessageSet selection;
  return parseDataUartConfigString(str, parseInfo, selection);
}

/// @brief Parses a can config string and populates the selection
/// @param str the config string to parse
/// @param selection data selection which can be used by getCanConfigString to regenerate this string 
///                  (only true if this string is already generated by getCanConfigString).
///                  NOT updated if this function returns early.
/// @throws std::invalid_argument if string is invalid
/// @return Bitmask of all data elements included in the string
DataSelection SensorDataBackend::parseCanConfigString(const std::string& str, std::vector<CanParseInfo>& parseInfo, std::vector<CanMessage>& selection) const
{
  // Should group related messages together (i.e. same data-rate and phase), 
  // so that we only inform about an update sample when all messages of that group is received.
  parseInfo.clear();

  // datarate and phase detection
  int dataRate = 10; // default 100Hz
  int phase = 0;
  CanParseInfo msg;
  DataSelection retval; // return-value (all included data)

  char str_switch;
  std::istringstream ss(str); // Stringstream for managing input
  while (ss >> str_switch) // this ignores whitespace but w/e...
  {
    switch (str_switch)
    {
    case 'o': // data rate
      // Parse its data rate
      dataRate = char_to_hex(ss.get());
      dataRate = (dataRate << 4) | char_to_hex(ss.get());
      dataRate = (dataRate << 4) | char_to_hex(ss.get());
      dataRate = (dataRate << 4) | char_to_hex(ss.get());
      if (dataRate == 0) throw std::invalid_argument("Data-rate divisor cannot be 0");
      phase = 0;
      break;

    case 'p': // phase
      phase = char_to_hex(ss.get());
      phase = (phase << 4) | char_to_hex(ss.get());
      break;

    case 'e':
    case 't': // standard id
      msg.idType = CanIdentifierTypes::CAN_STANDARD_ID;
      msg.dataRate = dataRate;
      msg.phase = phase;

      msg.identifier = char_to_hex(ss.get());
      msg.identifier = (msg.identifier << 4) | char_to_hex(ss.get());
      msg.identifier = (msg.identifier << 4) | char_to_hex(ss.get());
      if (msg.identifier > 0x7FF)
        throw std::invalid_argument(std::string("Invalid standard identifier (") + std::to_string(msg.identifier) + std::string(")"));
      
      msg.dataLength = char_to_hex(ss.get());
      for (uint8_t i = 0; i < msg.dataLength; i++)
      {
        msg.data.at(i).row = char_to_hex(ss.get());
        msg.data.at(i).row = (msg.data.at(i).row << 4) | char_to_hex(ss.get());
        msg.data.at(i).col = char_to_hex(ss.get());
      }

      parseInfo.push_back(msg);
      break;

    case 'T': // extended id + chip id
      msg.idType = CanIdentifierTypes::CAN_EXTENDED_ID_WITH_CHIP_ID;
      msg.dataRate = dataRate;
      msg.phase = phase;

      msg.identifier = char_to_hex(ss.get());
      msg.identifier = (msg.identifier << 4) | char_to_hex(ss.get());
      if (msg.identifier > 0x1F)
        throw std::invalid_argument(std::string("Invalid extended identifier with chip id (") + std::to_string(msg.identifier) + std::string(")"));
      
      msg.dataLength = char_to_hex(ss.get());
      for (uint8_t i = 0; i < msg.dataLength; i++)
      {
        msg.data.at(i).row = char_to_hex(ss.get());
        msg.data.at(i).row = (msg.data.at(i).row << 4) | char_to_hex(ss.get());
        msg.data.at(i).col = char_to_hex(ss.get());
      }

      parseInfo.push_back(msg);
      break;
    
    case 'C': // calibration, just place here for convenience...
      msg.idType = CanIdentifierTypes::CAN_EXTENDED_ID_WITH_CHIP_ID;
      msg.dataRate = 1000; // always 1 Hz
      msg.phase = phase;

      msg.identifier = char_to_hex(ss.get());
      msg.identifier = (msg.identifier << 4) | char_to_hex(ss.get());
      if (msg.identifier > 0x1F)
        throw std::invalid_argument(std::string("Invalid extended identifier with chip id (") + std::to_string(msg.identifier) + std::string(")"));
      
      msg.dataLength = char_to_hex(ss.get());
      for (uint8_t i = 0; i < msg.dataLength; i++)
      {
        msg.data.at(i).row = char_to_hex(ss.get());
        msg.data.at(i).row = (msg.data.at(i).row << 4) | char_to_hex(ss.get());
        msg.data.at(i).col = char_to_hex(ss.get());
      }

      parseInfo.push_back(msg);
      break;

    case 'E': // extended id with full address
      msg.idType = CanIdentifierTypes::CAN_EXTENDED_ID;
      msg.dataRate = dataRate;
      msg.phase = phase;

      msg.identifier = 0;
      for (int i = 0; i < 8; i++)
        msg.identifier = (msg.identifier << 4) | char_to_hex(ss.get());
      if (msg.identifier > 0x1FFFFFFF)
        throw std::invalid_argument(std::string("Invalid extended identifier (") + std::to_string(msg.identifier) + std::string(")"));
      
      msg.dataLength = char_to_hex(ss.get());
      for (uint8_t i = 0; i < msg.dataLength; i++)
      {
        msg.data.at(i).row = char_to_hex(ss.get());
        msg.data.at(i).row = (msg.data.at(i).row << 4) | char_to_hex(ss.get());
        msg.data.at(i).col = char_to_hex(ss.get());
      }

      parseInfo.push_back(msg);
      break;

    case 'n': // GX format switch
      // GX format, discard all input/output and return early (can't parse this from here)
      parseInfo.clear();
      selection.clear();
      return retval;

    default:
      throw std::invalid_argument(std::string("Unknown switch \"" + std::string(&str_switch,1) + std::string("\"")));
    }
  }

  // Extra input validation + endianness detection
  for (auto& msg : parseInfo)
  {
    msg.usingBigEndian = true;
    uint8_t previousRow = 0xFF;
    uint8_t previousCol = 0;

    for (uint8_t i = 0; i < msg.dataLength; i++)
    {
      if (msg.data.at(i).row >= SensorDataValueId::SENSOR_DATA_NUM)
        throw std::invalid_argument(std::string("Data value index \"") + std::to_string(msg.data.at(i).row) + std::string("\" out of bounds"));
      if (msg.data.at(i).col >= 4)
        throw std::invalid_argument(std::string("Byte index \"") + std::to_string(msg.data.at(i).col) + std::string("\" out of bounds"));
      // todo: make sure unqiue id's?

      // Estimate endianness (one case of little endian -> set little endian)
      if (msg.data.at(i).row == previousRow && msg.data.at(i).col > previousCol)
        msg.usingBigEndian = false;
      previousRow = msg.data.at(i).row;
      previousCol = msg.data.at(i).col; 
    }
  }

  // Populate which sensor data values are included in this string
  // Note that this isn't a perfect replica of the indata, we can technically parse any incoming indata
  // but when generating strings we require some consistency across the different messages...
  selection.clear();
  for (auto& msg : parseInfo)
  {
    CanMessage selMsg;
    selMsg.dataRate = msg.dataRate;
    selMsg.phase = msg.phase;
    selMsg.idType = msg.idType;
    selMsg.identifier = msg.identifier;
    selMsg.useBigEndian = msg.usingBigEndian;

    selMsg.selection.reset();
    for (int i = 0; i < msg.dataLength; i++)
      selMsg.selection.set(msg.data.at(i).row);
    retval |= selMsg.selection;

    selection.push_back(selMsg);
  }

  return retval;
}

DataSelection SensorDataBackend::parseCanConfigString(const std::string &str) const
{
  std::vector<CanMessage> selection;
  std::vector<CanParseInfo> parseInfo;
  return parseCanConfigString(str, parseInfo, selection);
}

/// @brief Converst char representation of a hexadecimal number to an int 
/// @throws std::invalid_argument in case of non-hexadecimal number
uint8_t SensorDataBackend::char_to_hex(const char c) const
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c == 0 || c == -1) throw std::invalid_argument("Reached end of string");
  throw std::invalid_argument(std::string("Argument \"") +  std::string(&c,1) + std::string("\" is not hex "));
}

/// @brief Parses a $PKEBD data message received on the USER UART and stores the resulting data
/// @param str the NMEA string
bool SensorDataBackend::parseUserUartData(const std::string &str)
{
  // First half of the message is the data index, second half is their values
  // ex: $PKEBD,66,10,11,12,55,56,57,9,1768075,-1075,-630,645,-605513,-884676,157743087,4183*6C

  for (auto value : values)
  {
    if (value != nullptr)
    {
      value->parsed = false;
      value->inferred = false;
    }
  }

  // Sanity checks and maximum length
  if (str.find('$') != 0) return false; // Not nmea
  std::size_t iend = str.find('*');
  if (iend == str.npos) return false; // Not nmea

  // Count nbr of commas to determine nbr of elements
  int nentries = 0;
  std::size_t idx = 0;
  while (idx < iend)
  {
    idx = str.find(',', idx);
    if (idx == str.npos) break;
    nentries++;
    idx++;
  }
  if (nentries % 2 != 0) return false; // invalid
  nentries /= 2;

  // Find start points
  std::size_t iidx = str.find(','); // string index of data index
  std::size_t didx = iidx;          // string index of data value
  for (int i = 0; i < nentries; i++) didx = str.find(',', ++didx);
  iidx++; 
  didx++;
  
  // Parse data and update
  try
  {
    for (int i = 0; i < nentries; i++)
    {
      idx = str.find(',', iidx);
      int idata = std::stoi(str.substr(iidx, idx-iidx));
      iidx = idx+1;

      idx = str.find(',', didx);
      if (idx == str.npos) idx = str.find('*');
      int32_t ddata = std::stoi(str.substr(didx, idx-didx));
      didx = idx+1;

      if (idata >= SensorDataValueId::SENSOR_DATA_NUM) throw std::logic_error("invalid index");

      auto value = values[idata];
      if (value != nullptr)
      {
        if (!value->isRecent(true)) // Don't update data if recent data exist on the data stream
        {
          value->convertFromRaw(ddata);
          value->parsed = true;
        }
        value->timestampUser();
      }
    }
    // TODO: calculateInferredData(); ???
  }
  catch (const std::logic_error &e)
  {
    // badly formatted input string...
    return false;
  }
  return true;
}

/// @brief Parses a message received on the data uart and updates the local data entries
/// @param data Message, must begin with 0xFA and end with checksum
/// @param parseInfo The active parser info, as returned from parseDataUartConfigString()
/// @return true if parsing successful, false otherwise
bool SensorDataBackend::parseDataUartData(const std::vector<uint8_t>& data, const DataUartParseInfo& parseInfo)
{
  // Reset parsing status
  for (auto value : values)
  {
    if (value != nullptr)
    {
      value->parsed = false;
      value->inferred = false;
      value->parseData.fill(0);
    }
  }

  // Sanity check data stream
  if (parseInfo.numMessages == 0 || parseInfo.numMessages > SENSOR_DATA_NUM_MESSAGES) return false; // no valid parsed...
  if (data[0] != 0xFA) return false; // correct header

  bool parse_success = false;
  for (int msg_id = 0; msg_id < parseInfo.numMessages; msg_id++)
  {
    const auto message = &parseInfo.messages[msg_id];

    // Check the identifier for this message and match it to one of the parsed messages
    if (found_message_identifier(data, message, parseInfo.numMessages))
    {
      if (data.size() < static_cast<std::size_t>(1 + message->numIdentifier + message->numBytes + message->checksumBytes)) return false;

      // Parse data
      for (int i = message->startIndex, j = 0; j < message->numBytes; i++, j++)
      {
        values[parseInfo.data[i].row]->parseData[parseInfo.data[i].col] = data[1 + message->numIdentifier + j];
        values[parseInfo.data[i].row]->parsed = true;
      }
      parse_success = true;
      break;
    }
  }
  if (!parse_success) return false; // no valid parser found for this message

  // Parsing successful! Convert
  for (auto value : values)
  {
    if (value != nullptr && value->parsed)
    {
      int32_t raw = static_cast<int32_t>(value->parseData[3]) << 24 |
                    static_cast<int32_t>(value->parseData[2]) << 16 |
                    static_cast<int32_t>(value->parseData[1]) << 8  |
                    static_cast<int32_t>(value->parseData[0]);
      value->convertFromRaw(raw);
      value->timestampData();
    }
  }
  calculateInferredData();

  // TODO: some sanity checking of some variables ?

  return true;
}

/// @brief Checks if the data packet is valid
/// @param data Data packet to validate
/// @param parseInfo The active parser info, as returned from parseDataUartConfigString()
/// @return true if valid data packet, false otherwise
bool SensorDataBackend::validateDataUartData(const std::vector<uint8_t>& data, const DataUartParseInfo& parseInfo) const
{
  if (parseInfo.numMessages == 0 || parseInfo.numMessages > SENSOR_DATA_NUM_MESSAGES) return false; // no valid parsed...
  if (data[0] != 0xFA) return false; // correct header

  for (int msg_id = 0; msg_id < parseInfo.numMessages; msg_id++)
  {
    auto message = &parseInfo.messages[msg_id];

    if (found_message_identifier(data, message, parseInfo.numMessages))
    {
      const int message_size = (message->numIdentifier + message->numBytes + message->checksumBytes);
      if (data.size() < static_cast<std::size_t>(1 + message_size)) return false;

      if (message->checksumBytes == 1)
      {
        // crc-8
        uint8_t chk = 0;
        for (int i = 0; i < message_size; i++)
        {
          chk ^= data[i+1];
        }
        return chk == 0;
      }
      else if (message->checksumBytes == 2)
      {
        return crc16(data.data() + 1, message_size) == 0;
      }
      return true; // no checksum
    }
  }

  return false; // didn't find a matching message
}

/// @brief Checks if a received data uart message has the identifier described in the parsed message
/// @return true if identifier matches the one in the message, false otherwise
bool SensorDataBackend::found_message_identifier(const std::vector<uint8_t>& data, const DataUartParseInfo::DataUartParsedMessage* message, int numMessages) const
{
  // If we only have 1 message without an identifier, always pass true
  if (numMessages == 1 && message->numIdentifier == 0) return true;

  // No identifer available to distinguish different messages
  if (numMessages > 1 && message->numIdentifier == 0) return false;

  // Require data space to check for identifiers
  if (data.size() < static_cast<std::size_t>(1 + message->numIdentifier)) return false; 

  // Check if identifier matches what's in data
  uint32_t data_identifier = 0; // identifier from data
  for (int i = 0; i < message->numIdentifier; i++)
    data_identifier = (data_identifier << 8) | (uint32_t)data[1 + i]; // first byte is header
  return data_identifier == message->identifier;
}

// Sensor data value -------------------------------------------------------------------------------------------------

SensorDataBackend::SensorDataValue::SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                                                    SensaitionHardwareModel hwModel, std::string tooltip, uint16_t minVersion)
{
  this->scaleFactor = 1.0;
  this->convert = nullptr;

  init(name, unit, type, measurementType, hwModel, tooltip, minVersion);
}

SensorDataBackend::SensorDataValue::SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                                                    double scaleFactor, SensaitionHardwareModel hwModel, std::string tooltip, uint16_t minVersion)
{
  this->scaleFactor = scaleFactor;
  this->convert = nullptr;

  init(name, unit, type, measurementType, hwModel, tooltip, minVersion);
}

SensorDataBackend::SensorDataValue::SensorDataValue(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                                                    std::function<double(int)> convert, SensaitionHardwareModel hwModel, std::string tooltip, uint16_t minVersion)
{
  this->convert = convert;
  try {
    this->scaleFactor = this->convert(1) - this->convert(0); // rough estimation, used to decide number of significant decimals
  } catch(const std::runtime_error& e) {
    this->scaleFactor = 1e-6; // fallback...
  }
  
  if (this->scaleFactor < 0) this->scaleFactor *= -1;
  
  init(name, unit, type, measurementType, hwModel, tooltip, minVersion);

  // init timestamps
  last_data = std::chrono::time_point<std::chrono::steady_clock>::min();
  last_user = std::chrono::time_point<std::chrono::steady_clock>::min();
}

SensorDataBackend::SensorDataValue::~SensorDataValue()
{
}

void SensorDataBackend::SensorDataValue::init(std::string name, std::string unit, DataType type, SensorSample::MeasurementType measurementType,
                                              SensaitionHardwareModel hwModel, std::string tooltip, uint16_t minVersion)
{
  // Common constructor (probably a nicer, more c++, way to do this but...)
  this->name = name;
  this->unit = unit;
  this->type = type;
  this->measurementType = measurementType;
  this->hwModel = hwModel;
  this->tooltip = tooltip;
  this->minVersion = minVersion;

  this->data.dfloat = 0; // Default to 0

  // Calculate number of relevant decimal points
  nDec = scaleFactor < 1 && scaleFactor > 0 ? std::lround(-1*std::log10(scaleFactor)) : 0;
}


/// @brief Converts raw value into their final data representation and stores it
/// @param raw raw value as received from the unit
void SensorDataBackend::SensorDataValue::convertFromRaw(int32_t raw)
{
  // Takes the raw value and converts it to the desired data type
  switch (type)
  {
  case DataType::FLOAT16:
    raw = static_cast<int32_t>(static_cast<int16_t>(raw)); // Force sign-extend from 16->32 bits
    [[fallthrough]];
  case DataType::FLOAT:
    data.dfloat = convert == nullptr ? scaleFactor*static_cast<double>(raw) : convert(raw);
    break;
  
  case DataType::INT8:
    data.dint = static_cast<int32_t>(static_cast<int8_t>(raw)); // sign-extend
    break;

  case DataType::INT16:
    data.dint = static_cast<int32_t>(static_cast<int16_t>(raw)); // sign-extend
    break;

  case DataType::INT32:
    data.dint = static_cast<int32_t>(raw);
    break;

  case DataType::UINT8:
  case DataType::UINT16:
  case DataType::UINT32:
  case DataType::FLAGS8:
  case DataType::FLAGS16:
  case DataType::FLAGS32:
    data.duint = static_cast<uint32_t>(raw);
    break;
  
  case DataType::UINT16x2:
  case DataType::FLAGS16x2:
    // Contains two sequential uint16 values
    data.duint16x2[0] = static_cast<uint16_t>(static_cast<uint32_t>(raw) >> 16);
    data.duint16x2[1] = static_cast<uint16_t>(static_cast<uint32_t>(raw) & 0xFFFF);
    break;
  
  case DataType::UINT8x2:
    // same format as uint16x2 but with only one significant byte per entry
    data.duint16x2[0] = static_cast<uint16_t>((static_cast<uint32_t>(raw) >> 16) & 0xFF);
    data.duint16x2[1] = static_cast<uint16_t>(static_cast<uint32_t>(raw) & 0xFF);
    break;
  
  case DataType::UINT8x4:
    // Contains 4 sequential uint8 values
    data.duint8x4[0] = static_cast<uint8_t>((static_cast<uint32_t>(raw) >> 24) & 0xFF);
    data.duint8x4[1] = static_cast<uint8_t>((static_cast<uint32_t>(raw) >> 16) & 0xFF);
    data.duint8x4[2] = static_cast<uint8_t>((static_cast<uint32_t>(raw) >> 8) & 0xFF);
    data.duint8x4[3] = static_cast<uint8_t>(static_cast<uint32_t>(raw) & 0xFF);
    break;
  } 
}

/// @brief Gets the number of relevant bytes present in the output data
int SensorDataBackend::SensorDataValue::getSize(void) const
{
  switch (type)
  {
  case DataType::INT8:
  case DataType::UINT8:
  case DataType::FLAGS8:
    return 1;

  case DataType::FLOAT16:
  case DataType::INT16:
  case DataType::UINT16:
  case DataType::FLAGS16:
  case DataType::UINT8x2:
    return 2;

  default: 
    return 4;
  }
}

/// @brief Gets the mask of the relevant bytes included in the 4-byte array
uint32_t SensorDataBackend::SensorDataValue::getByteMask(void) const
{
  switch (type)
  {
  case DataType::INT8:
  case DataType::UINT8:
  case DataType::FLAGS8:
    return 0b0001;

  case DataType::FLOAT16:
  case DataType::INT16:
  case DataType::UINT16:
  case DataType::FLAGS16:
    return 0b0011;
  
  case DataType::UINT8x2:
    return 0b0101;

  default:
    return 0b1111;
  }
}

/// @brief  Gets current value of the data as a double
/// @return Current data as a double 
double SensorDataBackend::SensorDataValue::getData(void) const
{
  switch (type)
  {
  case DataType::FLOAT:
  case DataType::FLOAT16:
    return static_cast<double>(data.dfloat);

  case DataType::UINT8:
  case DataType::UINT16:
  case DataType::UINT32:
  case DataType::FLAGS8:
  case DataType::FLAGS16:
  case DataType::FLAGS32:
    return static_cast<double>(data.duint);
  
  case DataType::INT8:
  case DataType::INT16:
  case DataType::INT32:
    return static_cast<double>(data.dint);
  
  case DataType::UINT8x2:
  case DataType::UINT16x2:
  case DataType::FLAGS16x2:
    return static_cast<double>(data.duint16x2[0]); // only first element for now...
  
  case DataType::UINT8x4:
    return static_cast<double>(data.duint8x4[0]); // only first element for now...
  }

  return 0.0;
}

/// @brief Gets the current data as a formatted string, with correct precision and unit 
/// @return Formatted string
std::string SensorDataBackend::SensorDataValue::getFormattedData(void) const
{
  std::ostringstream ss;

  switch (type)
  {
  case DataType::FLOAT:
  case DataType::FLOAT16:
    ss << std::fixed << std::setprecision(nDec) << data.dfloat;
    break;

  case DataType::INT8:
  case DataType::INT16:
  case DataType::INT32:
    ss << std::dec << data.dint;
    break;
  
  case DataType::UINT8:
  case DataType::UINT16:
  case DataType::UINT32:
    ss << std::dec << data.duint;
    break;

  case DataType::FLAGS8:
  case DataType::FLAGS16:
  case DataType::FLAGS32:
    ss << "0x" << std::hex << std::uppercase << std::setfill('0') \
       << std::setw(type == DataType::FLAGS32 ? 8 : (type == DataType::FLAGS16 ? 4 : 2)) \
       << data.duint;
    break;

  case DataType::UINT8x2:
  case DataType::UINT16x2:
    ss << std::dec << data.duint16x2[0] << " " << data.duint16x2[1];
    break;
  
  case DataType::FLAGS16x2:
    ss << "0x"  << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << data.duint16x2[0] \
       << " 0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << data.duint16x2[1];
    break;
  
  case DataType::UINT8x4:
    ss << std::dec << static_cast<int>(data.duint8x4[0]) << " " << static_cast<int>(data.duint8x4[1]) << " " \
                   << static_cast<int>(data.duint8x4[2]) << " " << static_cast<int>(data.duint8x4[3]);
    break;
  }

  // append unit
  if (!unit.empty()) ss << " " << unit;

  return ss.str();
}

void SensorDataBackend::SensorDataValue::insertMeasurementValue(SensorSample* sample) const
{
  using MT = SensorSample::MeasurementType;
  using MV = SensorSample::MeasurementValue;

  // INVALID signifies types that have no mapping to MeasurementType
  if (measurementType == MT::INVALID)
  {
    return;
  }

  switch (type)
  {
  case DataType::FLOAT16:
  case DataType::FLOAT:
    sample->set(measurementType, MV(static_cast<double>(data.dfloat), name, unit, nDec, inferred, parsed));
    break;

  case DataType::INT8:
  case DataType::INT16:
  case DataType::INT32:
    sample->set(measurementType, MV(static_cast<int32_t>(data.dint), name, unit, inferred, parsed));
    break;

  case DataType::UINT8:
  case DataType::UINT16:
  case DataType::UINT32:
    sample->set(measurementType, MV(static_cast<uint32_t>(data.duint), name, unit, 0, inferred, parsed));
    break;

  case DataType::FLAGS8:
    sample->set(measurementType, MV(static_cast<uint32_t>(data.duint), name, unit, 1, inferred, parsed));
    break;
  case DataType::FLAGS16:
    sample->set(measurementType, MV(static_cast<uint32_t>(data.duint), name, unit, 2, inferred, parsed));
    break;

  case DataType::FLAGS32:
    sample->set(measurementType, MV(static_cast<uint32_t>(data.duint), name, unit, 4, inferred, parsed));
    break;

  case DataType::UINT8x2:
  case DataType::UINT16x2:
    // Each array of multiple values is handled individually and is mapped to multiple MeasurementType enums
    if (measurementType == MT::UTC_YEAR)
    {
      sample->set(MT::UTC_YEAR, MV(static_cast<uint32_t>(data.duint16x2[0]), "utc_year", "", 0, inferred, parsed));
      sample->set(MT::UTC_MONTH, MV(static_cast<uint32_t>(data.duint16x2[1]), "utc_month", "", 0, inferred, parsed));
    }
    else if (measurementType == MT::GNSS1_FIXTYPE)
    {
      sample->set(MT::GNSS1_FIXTYPE, MV(static_cast<uint32_t>(data.duint16x2[1]), "gnss1_fixtype", "", 0, inferred, parsed));
      sample->set(MT::GNSS2_FIXTYPE, MV(static_cast<uint32_t>(data.duint16x2[0]), "gnss2_fixtype", "", 0, inferred, parsed));
    }
    else if (measurementType == MT::GNSS1_NUM_SAT)
    {
      sample->set(MT::GNSS1_NUM_SAT, MV(static_cast<uint32_t>(data.duint16x2[1]), "gnss1_num_sat", "", 0, inferred, parsed));
      sample->set(MT::GNSS2_NUM_SAT, MV(static_cast<uint32_t>(data.duint16x2[0]), "gnss2_num_sat", "", 0, inferred, parsed));
    }
    else if (measurementType == MT::GNSS_UTC_YEAR)
    {
      sample->set(MT::GNSS_UTC_YEAR, MV(static_cast<uint32_t>(data.duint16x2[0]), "gnss_utc_year", "", 0, inferred, parsed));
      sample->set(MT::GNSS_UTC_MONTH, MV(static_cast<uint32_t>(data.duint16x2[1]), "gnss_utc_month", "", 0, inferred, parsed));
    }
    else
      throw std::invalid_argument("Unhandled UINT8x2 or UINT16x2 name: " + name);
    break;

  case DataType::FLAGS16x2:
    // Each flag array is handled individually and is mapped to multiple MeasurementType enums
    if (measurementType == MT::GNSS1_FLAGS)
    {
      sample->set(MT::GNSS1_FLAGS, MV(static_cast<uint32_t>(data.duint16x2[1]), "gnss1_flags", "", 2, inferred, parsed));
      sample->set(MT::GNSS2_FLAGS, MV(static_cast<uint32_t>(data.duint16x2[0]), "gnss2_flags", "", 2, inferred, parsed));
    }
    else if (measurementType == MT::FRTK_FLAGS)
    {
      sample->set(MT::FRTK_FLAGS, MV(static_cast<uint32_t>(data.duint16x2[1]), "frtk_flags", "", 2, inferred, parsed));
      sample->set(MT::FRTK_FLAGS, MV(static_cast<uint32_t>(data.duint16x2[0]), "mrtk_flags", "", 2, inferred, parsed));
    }
    else
        throw std::invalid_argument("Unhandled FLAGS16x2 name: " + name);
    break;

  case DataType::UINT8x4:
    // Each uint8_t array is handled individually and is mapped to multiple MeasurementType enums
    if(measurementType == MT::UTC_DAY)
    {
      sample->set(MT::UTC_DAY, MV(static_cast<uint32_t>(data.duint8x4[0]), "utc_day", "", 0, inferred, parsed));
      sample->set(MT::UTC_HOUR, MV(static_cast<uint32_t>(data.duint8x4[1]), "utc_hour", "", 0, inferred, parsed));
      sample->set(MT::UTC_MINUTE, MV(static_cast<uint32_t>(data.duint8x4[2]), "utc_minute", "", 0, inferred, parsed));
      sample->set(MT::UTC_SECOND, MV(static_cast<uint32_t>(data.duint8x4[3]), "utc_second", "", 0, inferred, parsed));
    }
    else if(measurementType == MT::GNSS_UTC_DAY)
    {
      sample->set(MT::GNSS_UTC_DAY, MV(static_cast<uint32_t>(data.duint8x4[0]), "gnss_utc_day", "", 0, inferred, parsed));
      sample->set(MT::GNSS_UTC_HOUR, MV(static_cast<uint32_t>(data.duint8x4[1]), "gnss_utc_hour", "", 0, inferred, parsed));
      sample->set(MT::GNSS_UTC_MINUTE, MV(static_cast<uint32_t>(data.duint8x4[2]), "gnss_utc_minute", "", 0, inferred, parsed));
      sample->set(MT::GNSS_UTC_SECOND, MV(static_cast<uint32_t>(data.duint8x4[3]), "gnss_utc_second", "", 0, inferred, parsed));
    }
    else
      throw std::invalid_argument("Unhandled UINT8x4 name: " + name);
    break;
  }
}

/// @brief Timestamp reception of a new data from a DATA STREAM 
void SensorDataBackend::SensorDataValue::timestampData(void)
{
  last_data = std::chrono::steady_clock::now();
}

/// @brief Timestamp reception of new data from the USER UART STREAM
void SensorDataBackend::SensorDataValue::timestampUser(void)
{
  last_user = std::chrono::steady_clock::now();
}

/// @brief Checks if this data has recently been updated
/// @param data_only if true, only checks the data channels for recent values, if false also includes the user uart channel
/// @return true if this data has recently been updated, false otherwise
bool SensorDataBackend::SensorDataValue::isRecent(bool data_channels_only) const
{
  const auto now = std::chrono::steady_clock::now();
  auto diff_user = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_user).count();
  auto diff_data = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_data).count();

  return (diff_data >= 0 && diff_data < RECENT_TIMEOUT) || (!data_channels_only && diff_user >= 0 && diff_user < RECENT_TIMEOUT);
}

// Big functions -----------------------------------------------------------------------------------------------------

/// @brief Populates all the sensor data fields
void SensorDataBackend::populateSensorData(void)
{
  assert(values.size() == SensorDataValueId::SENSOR_DATA_NUM);
  assert(values_priority.size() == SensorDataValueId::SENSOR_DATA_NUM);

  using DataType = SensorDataValue::DataType;
  using HwModel = SensaitionHardwareModel;
  using MT = SensorSample::MeasurementType;

  values.fill(nullptr); // Make sure unfilled entries are reported as inactive

  auto temp_convert = [](int raw){return (static_cast<double>(raw) / 10000.0)*80.0 + 20.0;}; // Conversion function for temperature readings
  
  values.at(SensorDataValueId::ACC_X) = \
    new SensorDataValue("acc_x", "g", DataType::FLOAT, MT::ACCELERATION_X, 1e-6, HwModel::IMU, "Accelerometer X raw value");
  values.at(SensorDataValueId::ACC_Y) = \
    new SensorDataValue("acc_y", "g", DataType::FLOAT, MT::ACCELERATION_Y, 1e-6, HwModel::IMU, "Accelerometer Y raw value");
  values.at(SensorDataValueId::ACC_Z) = \
    new SensorDataValue("acc_z", "g", DataType::FLOAT, MT::ACCELERATION_Z, 1e-6, HwModel::IMU, "Accelerometer Z raw value");
  values.at(SensorDataValueId::GYRO_X) = \
    new SensorDataValue("gyro_x", "°/s", DataType::FLOAT, MT::GYRO_RATE_X, 1e-6, HwModel::IMU, "Gyro X raw value");
  values.at(SensorDataValueId::GYRO_Y) = \
    new SensorDataValue("gyro_y", "°/s", DataType::FLOAT, MT::GYRO_RATE_Y, 1e-6, HwModel::IMU, "Gyro Y raw value");
  values.at(SensorDataValueId::GYRO_Z) = \
    new SensorDataValue("gyro_z", "°/s", DataType::FLOAT, MT::GYRO_RATE_Z, 1e-6, HwModel::IMU, "Gyro Z raw value");
  values.at(SensorDataValueId::INCL_X) = \
    new SensorDataValue("incl_x", "g", DataType::FLOAT, MT::INCLINOMETER_X, 1e-6, HwModel::IMU, "Inclinometer X raw value");
  values.at(SensorDataValueId::INCL_Y) = \
    new SensorDataValue("incl_y", "g", DataType::FLOAT, MT::INCLINOMETER_Y, 1e-6, HwModel::IMU, "Inclinometer Y raw value");
  values.at(SensorDataValueId::INCL_Z) = \
    new SensorDataValue("incl_z", "g", DataType::FLOAT, MT::INCLINOMETER_Z, 1e-6, HwModel::IMU, "Inclinometer Z raw value");

  values.at(SensorDataValueId::TEMP) = \
    new SensorDataValue("temp", "°C", DataType::FLOAT16, MT::INTERNAL_TEMP, temp_convert, HwModel::IMU, "IMU temperature (internal)");

  values.at(SensorDataValueId::MAG_X) = \
    new SensorDataValue("mag_x", "gauss", DataType::FLOAT16, MT::MAGNETIC_FIELD_X, 1e-3, HwModel::IMU, "Magnetometer X raw value");
  values.at(SensorDataValueId::MAG_Y) = \
    new SensorDataValue("mag_y", "gauss", DataType::FLOAT16, MT::MAGNETIC_FIELD_Y, 1e-3, HwModel::IMU, "Magnetometer Y raw value");
  values.at(SensorDataValueId::MAG_Z) = \
    new SensorDataValue("mag_z", "gauss", DataType::FLOAT16, MT::MAGNETIC_FIELD_Z, 1e-3, HwModel::IMU, "Magnetometer Z raw value");

  values.at(SensorDataValueId::BAROMETER) = \
    new SensorDataValue("barometer", "hPa", DataType::FLOAT, MT::AIR_PRESSURE, 1e-3, HwModel::IMU, "Barometer raw value");
  values.at(SensorDataValueId::ODOMETER) = \
    new SensorDataValue("odometer", "m/s", DataType::FLOAT, MT::ODOMETER_SPEED, 1e-3, HwModel::IMU, "Odometer speed raw value");

  values.at(SensorDataValueId::TEMP_CALIB) = \
    new SensorDataValue("temp_calib", "°C", DataType::FLOAT16, MT::EXTERNAL_TEMP, temp_convert, HwModel::IMU, "Calibrated IMU temperature (estimated external)");

#ifndef USE_SENSOR_CALIB_DATA
  // RAW GNSS1 data in place of calibration data when not configured for calibration
  // Since this should always be the case for user operation, assume this area contains GNSS data
  // Can change this during compile time if we ever want to test calibration data.

  values.at(SensorDataValueId::GNSS_LATITUDE) = \
    new SensorDataValue("gnss_latitude", "°", DataType::FLOAT, MT::GNSS_LATITUDE, 1e-7, HwModel::INS, "Raw GNSS1 latitude", 583);
  values.at(SensorDataValueId::GNSS_LONGITUDE) = \
    new SensorDataValue("gnss_longitude", "°", DataType::FLOAT, MT::GNSS_LONGITUDE, 1e-7, HwModel::INS, "Raw GNSS1 longitude", 583);
  values.at(SensorDataValueId::GNSS_ALTITUDE) = \
    new SensorDataValue("gnss_altitude", "m", DataType::FLOAT, MT::GNSS_ALTITUDE, 1e-3, HwModel::INS, "Raw GNSS1 altitude", 583);
  
  values.at(SensorDataValueId::GNSS_VEL_NORTH) = \
    new SensorDataValue("gnss_vel_north", "m/s", DataType::FLOAT, MT::GNSS_VEL_NORTH, 1e-3, HwModel::INS, "Raw GNSS1 velocity north", 583);
  values.at(SensorDataValueId::GNSS_VEL_EAST) = \
    new SensorDataValue("gnss_vel_east", "m/s", DataType::FLOAT, MT::GNSS_VEL_EAST, 1e-3, HwModel::INS, "Raw GNSS1 velocity east", 583);
  values.at(SensorDataValueId::GNSS_VEL_DOWN) = \
    new SensorDataValue("gnss_vel_down", "m/s", DataType::FLOAT, MT::GNSS_VEL_DOWN, 1e-3, HwModel::INS, "Raw GNSS1 velocity down", 583);
  
  values.at(SensorDataValueId::GNSS_SPEED) = \
    new SensorDataValue("gnss_speed", "m/s", DataType::FLOAT, MT::GNSS_SPEED, 1e-3, HwModel::INS, "Raw GNSS1 absolute speed", 583);
  values.at(SensorDataValueId::GNSS_COURSE) = \
    new SensorDataValue("gnss_course", "°", DataType::FLOAT, MT::GNSS_COURSE, 1e-6, HwModel::INS, "Raw GNSS1 course over ground", 583);
  
  values.at(SensorDataValueId::GNSS_STD_HORZ) = \
    new SensorDataValue("gnss_std_horizontal", "m", DataType::FLOAT, MT::GNSS_STD_HORZIONTAL, 1e-3, HwModel::INS, "Raw GNSS1 horizontal quality", 583);
  values.at(SensorDataValueId::GNSS_STD_VERT) = \
    new SensorDataValue("gnss_std_vertical", "m", DataType::FLOAT, MT::GNSS_STD_VERTICAL, 1e-3, HwModel::INS, "Raw GNSS1 vertical quality", 583);
  values.at(SensorDataValueId::GNSS_STD_SPEED) = \
    new SensorDataValue("gnss_std_speed", "m/s", DataType::FLOAT, MT::GNSS_STD_SPEED, 1e-3, HwModel::INS, "Raw GNSS1 speed quality", 583);
  values.at(SensorDataValueId::GNSS_STD_COURSE) = \
    new SensorDataValue("gnss_std_course", "°", DataType::FLOAT, MT::GNSS_STD_COURSE, 1e-6, HwModel::INS, "Raw GNSS1 course over ground quality", 583);
  
  values.at(SensorDataValueId::GNSS_PDOP) = \
    new SensorDataValue("gnss_pdop", "", DataType::FLOAT, MT::GNSS_PDOP, 1e-2, HwModel::INS, "Raw GNSS1 position dilution of precision", 583);
  values.at(SensorDataValueId::GNSS_GEOID_SEP) = \
    new SensorDataValue("gnss_geoid_sep", "m", DataType::FLOAT, MT::GNSS_GEOID_SEP, 1e-3, HwModel::INS, "Raw GNSS1 geoid separation", 583);
  values.at(SensorDataValueId::GNSS_UTC_YEAR_MONTH) = \
    new SensorDataValue("gnss_utc_month", "year,month", DataType::UINT16x2, MT::GNSS_UTC_YEAR, HwModel::INS, "Raw GNSS1 UTC year and month timestamp", 583);
  values.at(SensorDataValueId::GNSS_UTC_DAY_H_MIN_S) = \
    new SensorDataValue("gnss_utc_day", "day,h,min,s", DataType::UINT8x4, MT::GNSS_UTC_DAY, HwModel::INS, "Raw GNSS1 UTC day, hours, minutes and seconds timestamp", 583);

  values.at(SensorDataValueId::GNSS_FLAGS) = \
    new SensorDataValue("gnss_flags", "gnss2,gnss1", DataType::FLAGS16x2, MT::GNSS1_FLAGS, HwModel::INS, "Raw status flags from GNSS receiver", 583);
  values.at(SensorDataValueId::GNSS_RELPOS_FLAGS) = \
    new SensorDataValue("gnss_rtk_flags", "MRTK,FRTK", DataType::FLAGS16x2, MT::FRTK_FLAGS, HwModel::INS, "Raw RTK status flags from GNSS receiver", 583);
  
#else
  // Calibration specific data, can be removed from public releases
  values.at(SensorDataValueId::MEAN_ACC_X) = new SensorDataValue("mean_acc_x", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_ACC_Y) = new SensorDataValue("mean_acc_y", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_ACC_Z) = new SensorDataValue("mean_acc_z", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_GYRO_X) = new SensorDataValue("mean_gyro_x", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_GYRO_Y) = new SensorDataValue("mean_gyro_y", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_GYRO_Z) = new SensorDataValue("mean_gyro_z", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_MAG_X) = new SensorDataValue("mean_mag_x", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_MAG_Y) = new SensorDataValue("mean_mag_y", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV);
  values.at(SensorDataValueId::MEAN_MAG_Z) = new SensorDataValue("mean_mag_z", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV);
  values.at(SensorDataValueId::STD_ACC_X) = new SensorDataValue("std_acc_x", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_ACC_Y) = new SensorDataValue("std_acc_y", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_ACC_Z) = new SensorDataValue("std_acc_z", "g", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_GYRO_X) = new SensorDataValue("std_gyro_x", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_GYRO_Y) = new SensorDataValue("std_gyro_y", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_GYRO_Z) = new SensorDataValue("std_gyro_z", "°/s", DataType::FLOAT, MT::INVALID, 1e-6, HwModel::DEV);
  values.at(SensorDataValueId::STD_MAG_X) = new SensorDataValue("std_mag_x", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV, "", 581);
  values.at(SensorDataValueId::STD_MAG_Y) = new SensorDataValue("std_mag_y", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV, "", 581);
  values.at(SensorDataValueId::STD_MAG_Z) = new SensorDataValue("std_mag_z", "gauss", DataType::FLOAT16, MT::INVALID, 1e-3, HwModel::DEV, "", 581);
  values.at(SensorDataValueId::COUNTER) = new SensorDataValue("counter", "ms", DataType::UINT32, MT::INVALID, HwModel::DEV);
#endif

  values.at(SensorDataValueId::FRTK_ITOW) = \
    new SensorDataValue("frtk_itow", "ms", DataType::UINT32, MT::FRTK_ITOW, HwModel::INS, "GNSS fixed base relpos time-of-week timestamp");
  values.at(SensorDataValueId::FRTK_NORTH) = \
    new SensorDataValue("frtk_north", "m", DataType::FLOAT, MT::FRTK_NORTH, 1e-3, HwModel::INS, "GNSS fixed base relpos north offset");
  values.at(SensorDataValueId::FRTK_EAST) = \
    new SensorDataValue("frtk_east", "m", DataType::FLOAT, MT::FRTK_EAST, 1e-3, HwModel::INS, "GNSS fixed base relpos east offset");
  values.at(SensorDataValueId::FRTK_DOWN) = \
    new SensorDataValue("frtk_down", "m", DataType::FLOAT, MT::FRTK_DOWN, 1e-3, HwModel::INS, "GNSS fixed base relpos vertical offset");

  values.at(SensorDataValueId::MRTK_ITOW) = \
    new SensorDataValue("mrtk_itow", "ms", DataType::UINT32, MT::MRTK_ITOW, HwModel::INS, "GNSS moving base relpos time-of-week timestamp");
  values.at(SensorDataValueId::MRTK_NORTH) = \
    new SensorDataValue("mrtk_north", "m", DataType::FLOAT, MT::MRTK_NORTH, 1e-3, HwModel::INS, "GNSS moving base relpos north offset");
  values.at(SensorDataValueId::MRTK_EAST) = \
    new SensorDataValue("mrtk_east", "m", DataType::FLOAT, MT::MRTK_EAST, 1e-3, HwModel::INS, "GNSS moving base relpos east offset");
  values.at(SensorDataValueId::MRTK_DOWN) = \
    new SensorDataValue("mrtk_down", "m", DataType::FLOAT, MT::MRTK_DOWN, 1e-3, HwModel::INS, "GNSS moving base relpos vertical offset");

  values.at(SensorDataValueId::NUM_SAT) = \
    new SensorDataValue("gnss_num_sat", "gnss2,gnss1", DataType::UINT8x2, MT::GNSS1_NUM_SAT, HwModel::INS, "Current number of satellites used in navigation solution", 564);

  values.at(SensorDataValueId::SENSORS_USED) = \
    new SensorDataValue("sensors_used", "", DataType::FLAGS16, MT::SENSORS_USED, HwModel::AHRS, "Sensors used in nav update", 584);
  values.at(SensorDataValueId::ERROR_FLAGS) = \
    new SensorDataValue("error_flags", "", DataType::FLAGS32, MT::ERROR_FLAGS, HwModel::IMU, "Currently flagged errors");
  values.at(SensorDataValueId::SENSOR_VALID) = \
    new SensorDataValue("sensor_valid", "", DataType::FLAGS8, MT::SENSOR_VALID, HwModel::IMU, "Available sensor signals");

  values.at(SensorDataValueId::POS_LATITUDE) = \
    new SensorDataValue("pos_latitude", "°", DataType::FLOAT, MT::POS_LATITUDE, 1e-7, HwModel::INS, "Estimated horizontal position latitude");
  values.at(SensorDataValueId::POS_LONGITUDE) = \
    new SensorDataValue("pos_longitude", "°", DataType::FLOAT, MT::POS_LONGITUDE, 1e-7, HwModel::INS, "Estimated horizontal position longitude");
  values.at(SensorDataValueId::VEL_NORTH) = \
    new SensorDataValue("vel_north", "m/s", DataType::FLOAT, MT::VEL_NORTH, 1e-3, HwModel::INS, "Estimated horizontal velocity north");
  values.at(SensorDataValueId::VEL_EAST) = \
    new SensorDataValue("vel_east", "m/s", DataType::FLOAT, MT::VEL_EAST, 1e-3, HwModel::INS, "Estimated horizontal velocity east");
  values.at(SensorDataValueId::POS_VERTICAL) = \
    new SensorDataValue("pos_vertical", "m", DataType::FLOAT, MT::POS_VERTICAL, 1e-3, HwModel::INS, "Estimated vertical position/altitude");
  values.at(SensorDataValueId::VEL_VERTICAL) = \
    new SensorDataValue("vel_vertical", "m/s", DataType::FLOAT, MT::VEL_DOWN, 1e-3, HwModel::INS, "Estimated vertical velocity");

  values.at(SensorDataValueId::ROLL) = \
    new SensorDataValue("roll", "°", DataType::FLOAT, MT::ROLL, 1e-6, HwModel::AHRS, "Estimated roll");
  values.at(SensorDataValueId::PITCH) = \
    new SensorDataValue("pitch", "°", DataType::FLOAT, MT::PITCH, 1e-6, HwModel::AHRS, "Estimated pitch");
  values.at(SensorDataValueId::HEADING) = \
    new SensorDataValue("heading", "°", DataType::FLOAT, MT::HEADING, 1e-6, HwModel::AHRS, "Estimated heading/yaw");

  values.at(SensorDataValueId::CORR_ACC_X) = \
    new SensorDataValue("corr_acc_x", "g", DataType::FLOAT, MT::CORR_ACCELERATION_X, 1e-6, HwModel::AHRS, "Estimated corrected accelerometer X values");
  values.at(SensorDataValueId::CORR_ACC_Y) = \
    new SensorDataValue("corr_acc_y", "g", DataType::FLOAT, MT::CORR_ACCELERATION_Y, 1e-6, HwModel::AHRS, "Estimated corrected accelerometer Y values");
  values.at(SensorDataValueId::CORR_ACC_Z) = \
    new SensorDataValue("corr_acc_z", "g", DataType::FLOAT, MT::CORR_ACCELERATION_Z, 1e-6, HwModel::AHRS, "Estimated corrected accelerometer Z values");
  values.at(SensorDataValueId::CORR_GYRO_X) = \
    new SensorDataValue("corr_gyro_x", "°/s", DataType::FLOAT, MT::CORR_GYRO_RATE_X, 1e-6, HwModel::AHRS, "Estimated corrected gyro X values");
  values.at(SensorDataValueId::CORR_GYRO_Y) = \
    new SensorDataValue("corr_gyro_y", "°/s", DataType::FLOAT, MT::CORR_GYRO_RATE_Y, 1e-6, HwModel::AHRS, "Estimated corrected gyro Y values");
  values.at(SensorDataValueId::CORR_GYRO_Z) = \
    new SensorDataValue("corr_gyro_z", "°/s", DataType::FLOAT, MT::CORR_GYRO_RATE_Z, 1e-6, HwModel::AHRS, "Estimated corrected gyro Z values");

  values.at(SensorDataValueId::ALIGNMENT_INS) = \
    new SensorDataValue("alignment_ins", "", DataType::UINT8, MT::ALIGNMENT_INS, HwModel::INS, "Alignment status, set if unit is in INS mode");
  values.at(SensorDataValueId::ATTITUDE_STATUS) = \
    new SensorDataValue("attitude_status", "", DataType::UINT8, MT::ATTITUDE_STATUS, HwModel::AHRS, "Attitude status, set if unit has valid roll, pitch and heading");

  values.at(SensorDataValueId::TICK) = \
    new SensorDataValue("tick", "ms", DataType::UINT32, MT::TICK, HwModel::IMU, "System time since startup");

  values.at(SensorDataValueId::GNSS1_ITOW) = \
    new SensorDataValue("gnss1_itow", "ms", DataType::UINT32, MT::GNSS1_ITOW, HwModel::INS, "GNSS1 time-of-week timestamp");
  values.at(SensorDataValueId::GNSS2_ITOW) = \
    new SensorDataValue("gnss2_itow", "ms", DataType::UINT32, MT::GNSS2_ITOW, HwModel::INS, "GNSS2 time-of-week timestamp");

  values.at(SensorDataValueId::UTC_YEAR_MONTH) = \
    new SensorDataValue("utc_month", "year,month", DataType::UINT16x2, MT::UTC_YEAR, HwModel::INS, "UTC year and month timestamp");
  values.at(SensorDataValueId::UTC_DAY_H_MIN_S) = \
    new SensorDataValue("utc_day", "day,h,min,s", DataType::UINT8x4, MT::UTC_DAY, HwModel::INS, "UTC day, hours, minutes and seconds timestamp");

  values.at(SensorDataValueId::GNSS_FIXTYPE) = \
    new SensorDataValue("gnss_fixtype", "gnss2,gnss1", DataType::UINT8x2, MT::GNSS1_FIXTYPE, HwModel::INS, "Current fix type by GNSS1 & GNSS2 (none,2D,3D fix)");
  
  values.at(SensorDataValueId::SYNC_IN_COUNT) = \
    new SensorDataValue("sync_in_count", "", DataType::UINT32, MT::SYNC_IN_COUNT, HwModel::IMU, "Number of sync-in pulses detected");
  values.at(SensorDataValueId::SYNC_IN_TIME) = \
    new SensorDataValue("sync_in_time", "µs", DataType::UINT32, MT::SYNC_IN_TIME, HwModel::IMU, "Time since last read sync-in pulse");

  values.at(SensorDataValueId::UTC_US) = \
    new SensorDataValue("utc_us", "µs", DataType::UINT32, MT::UTC_US, HwModel::INS, "UTC subsecond timestamp", 564);
  values.at(SensorDataValueId::UTC_STATUS) = \
    new SensorDataValue("utc_status", "", DataType::UINT8, MT::UTC_STATUS, HwModel::INS, "UTC timestamp status", 564);

  values.at(SensorDataValueId::Q_W) = \
    new SensorDataValue("q_w", "", DataType::FLOAT, MT::Q_W, 1e-6, HwModel::AHRS, "Estimated attitude quaternion scalar component");
  values.at(SensorDataValueId::Q_X) = \
    new SensorDataValue("q_x", "", DataType::FLOAT, MT::Q_X, 1e-6, HwModel::AHRS, "Estimated attitude quaternion X component");
  values.at(SensorDataValueId::Q_Y) = \
    new SensorDataValue("q_y", "", DataType::FLOAT, MT::Q_Y, 1e-6, HwModel::AHRS, "Estimated attitude quaternion Y component");
  values.at(SensorDataValueId::Q_Z) = \
    new SensorDataValue("q_z", "", DataType::FLOAT, MT::Q_Z, 1e-6, HwModel::AHRS, "Estimated attitude quaternion Z component");

  values.at(SensorDataValueId::ROTMAT_11) = \
    new SensorDataValue("rotmat_11", "", DataType::FLOAT, MT::ROTMAT_11, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (1,1) element");
  values.at(SensorDataValueId::ROTMAT_12) = \
    new SensorDataValue("rotmat_12", "", DataType::FLOAT, MT::ROTMAT_12, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (1,2) element");
  values.at(SensorDataValueId::ROTMAT_13) = \
    new SensorDataValue("rotmat_13", "", DataType::FLOAT, MT::ROTMAT_13, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (1,3) element");
  values.at(SensorDataValueId::ROTMAT_21) = \
    new SensorDataValue("rotmat_21", "", DataType::FLOAT, MT::ROTMAT_21, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (2,1) element");
  values.at(SensorDataValueId::ROTMAT_22) = \
    new SensorDataValue("rotmat_22", "", DataType::FLOAT, MT::ROTMAT_22, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (2,2) element");
  values.at(SensorDataValueId::ROTMAT_23) = \
    new SensorDataValue("rotmat_23", "", DataType::FLOAT, MT::ROTMAT_23, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (2,3) element");
  values.at(SensorDataValueId::ROTMAT_31) = \
    new SensorDataValue("rotmat_31", "", DataType::FLOAT, MT::ROTMAT_31, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (3,1) element");
  values.at(SensorDataValueId::ROTMAT_32) = \
    new SensorDataValue("rotmat_32", "", DataType::FLOAT, MT::ROTMAT_32, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (3,2) element");
  values.at(SensorDataValueId::ROTMAT_33) = \
    new SensorDataValue("rotmat_33", "", DataType::FLOAT, MT::ROTMAT_33, 1e-6, HwModel::AHRS, "Estimated attitude rotation matrix (3,3) element");

  values.at(SensorDataValueId::ECEF_POS_X) = \
    new SensorDataValue("ecef_pos_x", "m", DataType::FLOAT, MT::ECEF_POS_X, 1e-2, HwModel::INS, "ECEF position X", 553);
  values.at(SensorDataValueId::ECEF_POS_Y) = \
    new SensorDataValue("ecef_pos_y", "m", DataType::FLOAT, MT::ECEF_POS_Y, 1e-2, HwModel::INS, "ECEF position Y", 553);
  values.at(SensorDataValueId::ECEF_POS_Z) = \
    new SensorDataValue("ecef_pos_z", "m", DataType::FLOAT, MT::ECEF_POS_Z, 1e-2, HwModel::INS, "ECEF position Z", 553);

  values.at(SensorDataValueId::PERF_TICK) = \
    new SensorDataValue("perf_tick", "µs", DataType::UINT32, MT::PERF_TICK, HwModel::IMU, "High performance time tick");

  values.at(SensorDataValueId::STD_LATITUDE) = \
    new SensorDataValue("std_latitude", "m", DataType::FLOAT, MT::STD_LATITUDE, 1e-3, HwModel::INS, "Estimated quality horizontal position latitude");
  values.at(SensorDataValueId::STD_LONGITUDE) = \
    new SensorDataValue("std_longitude", "m", DataType::FLOAT, MT::STD_LONGITUDE, 1e-3, HwModel::INS, "Estimated quality horizontal position longitude");
  values.at(SensorDataValueId::STD_VEL_NORTH) = \
    new SensorDataValue("std_vel_north", "m/s", DataType::FLOAT, MT::STD_VEL_NORTH, 1e-3, HwModel::INS, "Estimated quality horizontal speed north");
  values.at(SensorDataValueId::STD_VEL_EAST) = \
    new SensorDataValue("std_vel_east", "m/s", DataType::FLOAT, MT::STD_VEL_EAST, 1e-3, HwModel::INS, "Estimated quality horizontal speed east");
  values.at(SensorDataValueId::STD_POS_VERTICAL) = \
    new SensorDataValue("std_pos_vertical", "m", DataType::FLOAT, MT::STD_POS_VERTICAL, 1e-3, HwModel::INS, "Estimated quality vertical position/altitude");
  values.at(SensorDataValueId::STD_VEL_VERTICAL) = \
    new SensorDataValue("std_vel_vertical", "m/s", DataType::FLOAT, MT::STD_VEL_DOWN, 1e-3, HwModel::INS, "Estimated quality vertical speed");
  values.at(SensorDataValueId::STD_ROLL) = \
    new SensorDataValue("std_roll", "°", DataType::FLOAT, MT::STD_ROLL, 1e-6, HwModel::AHRS, "Estimated quality roll");
  values.at(SensorDataValueId::STD_PITCH) = \
    new SensorDataValue("std_pitch", "°", DataType::FLOAT, MT::STD_PITCH, 1e-6, HwModel::AHRS, "Estimated quality pitch");
  values.at(SensorDataValueId::STD_HEADING) = \
    new SensorDataValue("std_heading", "°", DataType::FLOAT, MT::STD_HEADING, 1e-6, HwModel::AHRS, "Estimated quality heading/yaw");

  // Developer debug data
  values.at(SensorDataValueId::UTC_SEC_PERIOD) = new SensorDataValue("utc_sec_period", "", DataType::UINT32, MT::DEBUG6, HwModel::DEV, "UTC sec estimate timer period", 564);
  values.at(SensorDataValueId::TEMP_MAG) = new SensorDataValue("temp_mag", "°C", DataType::FLOAT16, MT::DEBUG5, temp_convert, HwModel::DEV, "Magnetometer temperature reading", 564);
  values.at(SensorDataValueId::WDO_PIN) = new SensorDataValue("wdo_pin", "", DataType::UINT8, MT::DEBUG4, HwModel::DEV, "Watchdog fault wiggle pin state", 564);
  values.at(SensorDataValueId::GNSS_DELAY) = new SensorDataValue("gnss_delay", "µs", DataType::UINT32, MT::DEBUG3, HwModel::DEV, "Delay between GNSS PPS and received message", 564);
  values.at(SensorDataValueId::KALMAN_TIME) = new SensorDataValue("kalman_time", "µs", DataType::UINT32, MT::DEBUG2, HwModel::DEV, "Kalman filter update execution time", 564);
  values.at(SensorDataValueId::NAV_TIME) = new SensorDataValue("nav_time", "µs", DataType::UINT32, MT::DEBUG1, HwModel::DEV, "Kalman filter navigation execution time", 564);

  // Generate a list of all SensorDataValueId's sorted based on SensorSample::MeasurementType
  // Used to sort and group related values together in the generated strings, produces the same byte order as the generated logs
  for (std::size_t i = 0; i < values_priority.size(); i++)
    values_priority.at(i) = i;
  
  std::sort(values_priority.begin(), values_priority.end(), 
    [this](std::size_t a, std::size_t b)
    {
      // Sort based on measurementType, if they're the same, fallback to raw indices
      MT ma = values.at(a) == nullptr ? MT::INVALID : values.at(a)->measurementType;
      MT mb = values.at(b) == nullptr ? MT::INVALID : values.at(b)->measurementType;
      return (ma == mb) ? a < b : ma < mb;
    });
}


/// @brief Calculates sensor data that can be inferred from other data fields, 
///        ex: roll,pitch,yaw <-> quat <-> rotmat.
///        To be called after any data parser when the "parsed" field of each sensor data value has been populated
void SensorDataBackend::calculateInferredData(void)
{
  // Attitude, convert between quat, euler, rotmat
  bool has_euler = values.at(SensorDataValueId::ROLL)->parsed && values.at(SensorDataValueId::PITCH)->parsed && values.at(SensorDataValueId::HEADING)->parsed;
  bool has_quat = values.at(SensorDataValueId::Q_W)->parsed && values.at(SensorDataValueId::Q_X)->parsed && \
                  values.at(SensorDataValueId::Q_Y)->parsed && values.at(SensorDataValueId::Q_Z)->parsed;
  bool has_rotmat = values.at(SensorDataValueId::ROTMAT_11)->parsed && values.at(SensorDataValueId::ROTMAT_12)->parsed && values.at(SensorDataValueId::ROTMAT_13)->parsed && \
                    values.at(SensorDataValueId::ROTMAT_21)->parsed && values.at(SensorDataValueId::ROTMAT_22)->parsed && values.at(SensorDataValueId::ROTMAT_23)->parsed && \
                    values.at(SensorDataValueId::ROTMAT_31)->parsed && values.at(SensorDataValueId::ROTMAT_32)->parsed && values.at(SensorDataValueId::ROTMAT_33)->parsed;
  
  if (has_euler && !has_rotmat)
  {
    const double roll = (M_PI/180)*values.at(SensorDataValueId::ROLL)->getData();
    const double pitch = (M_PI/180)*values.at(SensorDataValueId::PITCH)->getData();
    const double yaw = (M_PI/180)*values.at(SensorDataValueId::HEADING)->getData();

    // eul -> rotmat
    const double cx = cos(roll), sx = sin(roll);
    const double cy = cos(pitch), sy = sin(pitch);
    const double cz = cos(yaw), sz = sin(yaw);
    values.at(SensorDataValueId::ROTMAT_11)->data.dfloat = cy*cz;
    values.at(SensorDataValueId::ROTMAT_12)->data.dfloat = cz*sx*sy - cx*sz;
    values.at(SensorDataValueId::ROTMAT_13)->data.dfloat = cx*cz*sy + sx*sz;
    values.at(SensorDataValueId::ROTMAT_21)->data.dfloat = cy*sz;
    values.at(SensorDataValueId::ROTMAT_22)->data.dfloat = cx*cz + sx*sy*sz;
    values.at(SensorDataValueId::ROTMAT_23)->data.dfloat = cx*sy*sz - cz*sx;
    values.at(SensorDataValueId::ROTMAT_31)->data.dfloat = -sy;
    values.at(SensorDataValueId::ROTMAT_32)->data.dfloat = cy*sx;
    values.at(SensorDataValueId::ROTMAT_33)->data.dfloat = cx*cy;

    for (std::size_t i = SensorDataValueId::ROTMAT_11; i <= SensorDataValueId::ROTMAT_33; i++)
    {
      values.at(i)->parsed = true;
      values.at(i)->inferred = true;
      values.at(i)->timestampData();
    }
    has_rotmat = true;
  }

  if (has_rotmat && !has_quat)
  {
    const double r00 = values.at(SensorDataValueId::ROTMAT_11)->getData();
    const double r01 = values.at(SensorDataValueId::ROTMAT_12)->getData();
    const double r02 = values.at(SensorDataValueId::ROTMAT_13)->getData();
    const double r10 = values.at(SensorDataValueId::ROTMAT_21)->getData();
    const double r11 = values.at(SensorDataValueId::ROTMAT_22)->getData();
    const double r12 = values.at(SensorDataValueId::ROTMAT_23)->getData();
    const double r20 = values.at(SensorDataValueId::ROTMAT_31)->getData();
    const double r21 = values.at(SensorDataValueId::ROTMAT_32)->getData();
    const double r22 = values.at(SensorDataValueId::ROTMAT_33)->getData();

    // Convert rotmat -> quat
    double qw, qx, qy, qz;
    const double tr = r00 + r11 + r22;
    if (tr > 0) { 
      double S = sqrt(tr + 1.0) * 2; // S=4*qw 
      qw = 0.25 * S;
      qx = (r21 - r12) / S;
      qy = (r02 - r20) / S; 
      qz = (r10 - r01) / S; 
    } else if ((r00 > r11) && (r00 > r22)) { 
      double S = sqrt(1.0 + r00 - r11 - r22) * 2; // S=4*qx 
      qw = (r21 - r12) / S;
      qx = 0.25 * S;
      qy = (r01 + r10) / S; 
      qz = (r02 + r20) / S; 
    } else if (r11 > r22) { 
      float S = sqrt(1.0 + r11 - r00 - r22) * 2; // S=4*qy
      qw = (r02 - r20) / S;
      qx = (r01 + r10) / S; 
      qy = 0.25 * S;
      qz = (r12 + r21) / S; 
    } else { 
      float S = sqrt(1.0 + r22 - r00 - r11) * 2; // S=4*qz
      qw = (r10 - r01) / S;
      qx = (r02 + r20) / S;
      qy = (r12 + r21) / S;
      qz = 0.25 * S;
    }

    values.at(SensorDataValueId::Q_W)->data.dfloat = qw;
    values.at(SensorDataValueId::Q_X)->data.dfloat = qx;
    values.at(SensorDataValueId::Q_Y)->data.dfloat = qy;
    values.at(SensorDataValueId::Q_Z)->data.dfloat = qz;

    for (std::size_t i = SensorDataValueId::Q_W; i <= SensorDataValueId::Q_Z; i++)
    {
      values.at(i)->parsed = true;
      values.at(i)->inferred = true;
      values.at(i)->timestampData();
    }
    has_quat = true;
  }

  if (has_quat)
  {
    const double q0 = values.at(SensorDataValueId::Q_W)->getData();
    const double q1 = values.at(SensorDataValueId::Q_X)->getData();
    const double q2 = values.at(SensorDataValueId::Q_Y)->getData();
    const double q3 = values.at(SensorDataValueId::Q_Z)->getData();

    if (!has_euler)
    {
      // quat -> euler
      const double sinp = 2 * (q0 * q2 - q3 * q1);
      if (abs(sinp) > 0.999999)
      {
        // Looking straight up or down (gimbal lock)
        values.at(SensorDataValueId::ROLL)->data.dfloat = 0;
        values.at(SensorDataValueId::PITCH)->data.dfloat = copysign(90,sinp);
        values.at(SensorDataValueId::HEADING)->data.dfloat = (180/M_PI)*atan2(2*(q0*q3 - q1*q2), 1 - 2*(q1*q1 + q3*q3));
      }
      else
      {
        values.at(SensorDataValueId::ROLL)->data.dfloat = (180/M_PI)*atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
        values.at(SensorDataValueId::PITCH)->data.dfloat = (180/M_PI)*asin(sinp);
        values.at(SensorDataValueId::HEADING)->data.dfloat = (180/M_PI)*atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
      }

      for (std::size_t i = SensorDataValueId::ROLL; i <= SensorDataValueId::HEADING; i++)
      {
        values.at(i)->parsed = true;
        values.at(i)->inferred = true;
        values.at(i)->timestampData();
      }
      has_euler = true;
    }

    if (!has_rotmat)
    {
      // quat -> rotmat
      values.at(SensorDataValueId::ROTMAT_11)->data.dfloat = 2*(q0*q0 + q1*q1) - 1;
      values.at(SensorDataValueId::ROTMAT_12)->data.dfloat = 2*(q1*q2 - q0*q3);
      values.at(SensorDataValueId::ROTMAT_13)->data.dfloat = 2*(q1*q3 + q0*q2);
      values.at(SensorDataValueId::ROTMAT_21)->data.dfloat = 2*(q1*q2 + q0*q3);
      values.at(SensorDataValueId::ROTMAT_22)->data.dfloat = 2*(q0*q0 + q2*q2) - 1;
      values.at(SensorDataValueId::ROTMAT_23)->data.dfloat = 2*(q2*q3 - q0*q1);
      values.at(SensorDataValueId::ROTMAT_31)->data.dfloat = 2*(q1*q3 - q0*q2);
      values.at(SensorDataValueId::ROTMAT_32)->data.dfloat = 2*(q2*q3 + q0*q1);
      values.at(SensorDataValueId::ROTMAT_33)->data.dfloat = 2*(q0*q0 + q3*q3) - 1;

      for (int i = SensorDataValueId::ROTMAT_11; i <= SensorDataValueId::ROTMAT_33; i++)
      {
        values.at(i)->parsed = true;
        values.at(i)->inferred = true;
        values.at(i)->timestampData();
      }
      has_rotmat = true; 
    }
  }

  const bool has_lla = values.at(SensorDataValueId::POS_LATITUDE)->parsed && values.at(SensorDataValueId::POS_LONGITUDE)->parsed && values.at(SensorDataValueId::POS_VERTICAL)->parsed;
  const bool has_ecef = values.at(SensorDataValueId::ECEF_POS_X)->parsed && values.at(SensorDataValueId::ECEF_POS_Y)->parsed && values.at(SensorDataValueId::ECEF_POS_Z)->parsed;

  if (has_lla != has_ecef)
  {
    const double WGS84_A = 6378137;
    //const double WGS84_E = 8.1819190842622e-2;
    const double WGS84_E2 = 6.69437999014e-3;

    if (has_lla)
    {
      // Convert LLA -> ECEF
      const double lat = (M_PI/180)*values.at(SensorDataValueId::POS_LATITUDE)->getData();
      const double lon = (M_PI/180)*values.at(SensorDataValueId::POS_LONGITUDE)->getData();
      const double alt = values.at(SensorDataValueId::POS_VERTICAL)->getData();
      const double clat = cos(lat);
      const double slat = sin(lat);
      const double clon = cos(lon);
      const double slon = sin(lon);

      double N = WGS84_A / sqrt(1.0 - WGS84_E2 * slat * slat);
      values.at(SensorDataValueId::ECEF_POS_X)->data.dfloat = (N + alt) * clat * clon;
      values.at(SensorDataValueId::ECEF_POS_Y)->data.dfloat = (N + alt) * clat * slon;
      values.at(SensorDataValueId::ECEF_POS_Z)->data.dfloat = (N * (1.0 - WGS84_E2) + alt) * slat;

      for (std::size_t i = SensorDataValueId::ECEF_POS_X; i <= SensorDataValueId::ECEF_POS_Z; i++)
      {
        values.at(i)->parsed = true;
        values.at(i)->inferred = true;
        values.at(i)->timestampData();
      }
    }
    else
    {
      // Convert ECEF -> LLA
      const double x = values.at(SensorDataValueId::ECEF_POS_X)->getData();
      const double y = values.at(SensorDataValueId::ECEF_POS_Y)->getData();
      const double z = values.at(SensorDataValueId::ECEF_POS_Z)->getData();
      double b   = sqrt(WGS84_A*WGS84_A*(1 - WGS84_E2));
      double ep  = sqrt((WGS84_A*WGS84_A - b*b) / (b*b));
      double p   = sqrt(x*x + y*y);
      double th  = atan2(WGS84_A*z, b*p);
      double lon = atan2(y, x);
      double lat = atan2((z + ep*ep*b*pow(sin(th),3)), (p - WGS84_E2*WGS84_A*pow(cos(th),3)));
      double N   = WGS84_A/sqrt(1 - WGS84_E2*pow(sin(lat),2));
      double alt = p/cos(lat) - N;

      if (abs(x) < 1 && abs(y) < 1)
        alt = abs(z) - b; // correct for numerical instability in altitude near exact poles

      values.at(SensorDataValueId::POS_LATITUDE)->data.dfloat  = (180/M_PI) * lat;
      values.at(SensorDataValueId::POS_LONGITUDE)->data.dfloat = (180/M_PI) * lon;
      values.at(SensorDataValueId::POS_VERTICAL)->data.dfloat = alt;

      values.at(SensorDataValueId::POS_LATITUDE)->parsed = values.at(SensorDataValueId::POS_LATITUDE)->inferred = true;
      values.at(SensorDataValueId::POS_LATITUDE)->timestampData();
      values.at(SensorDataValueId::POS_LONGITUDE)->parsed = values.at(SensorDataValueId::POS_LONGITUDE)->inferred = true;
      values.at(SensorDataValueId::POS_LONGITUDE)->timestampData();
      values.at(SensorDataValueId::POS_VERTICAL)->parsed = values.at(SensorDataValueId::POS_VERTICAL)->inferred = true;
      values.at(SensorDataValueId::POS_VERTICAL)->timestampData();
    }
  }
}
