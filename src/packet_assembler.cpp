#include "sensaition_parser/packet_assembler.hpp"

#include <stdexcept>

namespace sepa
{
    constexpr uint8_t packageSync = 0xFA;

    // parseInfo.messages[msg_idx].numIdentifier + parseInfo.messages[msg_idx].numBytes + parseInfo.messages[msg_idx].checksumBytes
    void PacketAssembler::resetBuffer(const DataUartParseInfo& parseInfo)
    {
        std::size_t maxMessageSize = 0;
        for (int i = 0; i < parseInfo.numMessages; i++) {
            const auto& msg = parseInfo.messages[i];
            maxMessageSize = std::max<size_t>(maxMessageSize,
                static_cast<std::size_t>(msg.numIdentifier + msg.numBytes + msg.checksumBytes));
        }
        inputBuffer_.resize(maxMessageSize + 1); // +1 for packageSync
        parseState_ = PARSE_HEADER;
        parseIndex_ = 0;
    }

    int PacketAssembler::checkIdentifierInInputBuffer(const DataUartParseInfo& parseInfo) const
    {
        int i;
        uint32_t identifier = 0;
        for (i = 0; i < parseInfo.messages[0].numIdentifier; i++)
            identifier = (identifier << 8) | inputBuffer_.at(1 + i);
        for (i = 0; i < parseInfo.numMessages; i++)
            if (parseInfo.messages[i].identifier == identifier)
                return i;
        return -1; // no found...
    }

    size_t PacketAssembler::processBytes(const DataUartParseInfo& parseInfo, std::size_t bytesTransferred)
    {
        std::size_t parseIndex;
        std::size_t parseSize;
        int msg_idx;
        bool valid = false;

        switch (parseState_)
        {
        case PARSE_HEADER:
            if (parseInfo.numMessages > 0 && bytesTransferred == 1 && inputBuffer_.at(0) == packageSync)
            {
                /* Found header */
                if (parseInfo.messages[0].numIdentifier > 0)
                {
                    /* Expect identifiers, read it */
                    parseIndex = 1;
                    parseSize = parseInfo.messages[0].numIdentifier;
                    parseState_ = PARSE_IDENTIFER;
                }
                else
                {
                    /* No identifier, assume only 1 message and go directly to payload */
                    parseIndex = 1;
                    parseSize = parseInfo.messages[0].numBytes + parseInfo.messages[0].checksumBytes;
                    parseState_ = PARSE_PAYLOAD_CHECKSUM;
                }
                /* Assumes either none or all messages contains an identifier of the same size.
                * This should be guaranteed by the SensorDataBackend parser. */
            }
            else
            {
                /* Continue looking for header */
                parseIndex = 0;
                parseSize = 1;
            }
            break;

        case PARSE_IDENTIFER:
            valid = false;
            if (bytesTransferred == parseInfo.messages[0].numIdentifier)
            {
                msg_idx = checkIdentifierInInputBuffer(parseInfo);
                if (msg_idx >= 0)
                {
                    /* Found a matching identifier, parse payload + checksum */
                    parseIndex = 1 + parseInfo.messages[msg_idx].numIdentifier;
                    parseSize = parseInfo.messages[msg_idx].numBytes + parseInfo.messages[msg_idx].checksumBytes;
                    parseState_ = PARSE_PAYLOAD_CHECKSUM;
                    valid = true;
                }
            }
            
            if (!valid)
            {
                /* Sketchy indata or no matching identifier, go back to search for header... */
                parseIndex = 0;
                parseSize = 1;
                parseState_ = PARSE_HEADER;
            }
            break;
        
        case PARSE_PAYLOAD_CHECKSUM:
            msg_idx = (parseInfo.messages[0].numIdentifier > 0) ? checkIdentifierInInputBuffer(parseInfo) : 0;
            valid = false;
            if (msg_idx >= 0 && bytesTransferred == static_cast<std::size_t>(parseInfo.messages[msg_idx].numBytes + parseInfo.messages[msg_idx].checksumBytes))
            {
                const std::size_t msg_size = static_cast<std::size_t>(\
                    parseInfo.messages[msg_idx].numIdentifier + parseInfo.messages[msg_idx].numBytes + parseInfo.messages[msg_idx].checksumBytes);

                if (parseInfo.messages[msg_idx].checksumBytes == 1)
                {
                    // calculate & validate 8-bit checksum
                    uint8_t chk = 0;
                    for (std::size_t i = 1; i <= msg_size; i++)
                        chk ^= inputBuffer_.at(i); 
                    valid = (chk == 0);
                }
                else if (parseInfo.messages[msg_idx].checksumBytes == 2)
                {
                    // calculate & validate crc16
                    valid = (crc16(inputBuffer_.data() + 1, msg_size) == 0);
                }
                else
                {
                    valid = true; // no checksum, assume always valid
                }
            }
            
            if (valid)
            {
                // On valid message, send up...
                if (packetProcessor_ != nullptr)
                    packetProcessor_->processPacket(inputBuffer_);

                // ...and start searching for the next header
                parseIndex = 0;
                parseSize = 1;
                parseState_ = PARSE_HEADER;
            }
            else
            {
                // Not a valid message, enter resync to try to sync back to header
                // random read size from: 1 <= size <= buffer_size
                parseIndex = 0;
                parseSize = (static_cast<std::size_t>(std::rand()) % inputBuffer_.size()) + 1;
                parseState_ = PARSE_RESYNC;
            }
            break;
        
        case PARSE_RESYNC:
            // Resync complete, start searching for new header
            parseIndex = 0;
            parseSize = 1;
            parseState_ = PARSE_HEADER;
            break;

        default:
            throw std::runtime_error("Unknown data uart state");
        }

        if (parseSize == 0 || (parseIndex + parseSize) > inputBuffer_.size())
        {
            // safeguard
            parseIndex = 0;
            parseSize = 1;
            parseState_ = PARSE_HEADER;
        }
        parseIndex_ = parseIndex;
        return parseSize; // number of bytes wanted in next call
    } // processBytes

} // namespace
