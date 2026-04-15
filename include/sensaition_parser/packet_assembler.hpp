#ifndef PACKET_ASSEMBLER_HPP
#define PACKET_ASSEMBLER_HPP

#include <array>
#include <cstdint>
#include <vector>

#include "sensaition_parser/sensor_types.hpp" // DataUartParseInfo

namespace sepa
{

    /// @brief PacketAssembler listener interface
    class PacketProcessor
    {
    public:
        PacketProcessor() = default;
        virtual ~PacketProcessor() = default;
        PacketProcessor(const PacketProcessor&) = delete;
        PacketProcessor& operator=(const PacketProcessor&) = delete;
        PacketProcessor(PacketProcessor&&) = default;
        PacketProcessor& operator=(PacketProcessor&&) = default;

        virtual void processPacket(std::vector<std::uint8_t>& data) = 0;
    };

    /// @brief Packs data-uart bytes into messages
    class PacketAssembler
    {
    public:
        PacketAssembler() : packetProcessor_(nullptr), parseState_(PARSE_HEADER) {} 
        virtual ~PacketAssembler() = default;

        /// @brief Sets object implementing the callback
        /// @param listener Receiver of complete packages
        void setListener(PacketProcessor* listener) { packetProcessor_ = listener; }

        /// @brief Resets state for new message and updates input buffer size
        /// @param parseInfo Parse information as received from parsing the config string
        void resetBuffer(const DataUartParseInfo& parseInfo);

        /// @brief Parse the input buffer
        /// @param parseInfo Parse information as received from parsing the config string
        /// @param bytesTransferred Number of new bytes in input buffer to process
        /// @return Number of bytes wanted in next call to this method
        size_t processBytes(const DataUartParseInfo& parseInfo, std::size_t bytesTransferred);

        /// @brief Indices the input buffer, returning pointer for incoming data
        /// @return Pointer to current buffer position, where to write incoming data
        uint8_t* inputBuffer() { return inputBuffer_.data() + parseIndex_; }

    private:
        /// @brief Checks the identifier field in the input buffer and returns the index of the matched message
        /// @param parseInfo Parse information as received from parsing the config string
        /// @return index of message with matching identifier, or -1 if no match found
        int checkIdentifierInInputBuffer(const DataUartParseInfo& parseInfo) const;

        PacketProcessor* packetProcessor_;
        std::vector<uint8_t> inputBuffer_;
        enum ParserState {PARSE_HEADER, PARSE_IDENTIFER, PARSE_PAYLOAD_CHECKSUM, PARSE_RESYNC} parseState_;
        std::size_t parseIndex_ = 0;
    };

} // namespace
#endif // PACKET_ASSEMBLER_HPP
