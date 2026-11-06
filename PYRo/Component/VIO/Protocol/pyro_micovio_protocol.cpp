#include "pyro_micovio_protocol.h"
#include "pyro_crc.h"

#include <algorithm>
#include <cstring>

namespace pyro
{
namespace
{
constexpr size_t HEADER_LENGTH = sizeof(micovio_header_t);

int crc_extra(uint32_t message_id)
{
    switch (message_id)
    {
        case MICOVIO_ODOMETRY_ID: return 91;
        case MICOVIO_DISTANCE_ID: return 85;
        case 0: return 50;   // HEARTBEAT, optional on other firmware.
        case 77: return 143; // COMMAND_ACK.
        default: return -1;
    }
}

} // namespace

micovio_parser_t::micovio_parser_t(uint8_t sysid, uint8_t compid)
    : _sysid(sysid), _compid(compid)
{
}

void micovio_parser_t::set_callback(frame_callback_t callback, void *user)
{
    _callback = callback;
    _user = user;
}

void micovio_parser_t::reset()
{
    _unpack.step = unpack_step::HEADER_SOF;
    _unpack.index = 0;
}

void micovio_parser_t::drop(size_t length)
{
    _unpack.index -= length;
    std::memmove(_unpack.packet, _unpack.packet + length, _unpack.index);
}

void micovio_parser_t::feed(const uint8_t *data, size_t length)
{
    if (!data)
        return;
    for (size_t i = 0; i < length; ++i)
    {
        _unpack.packet[_unpack.index++] = data[i];
        unpack_data();
    }
}

void micovio_parser_t::unpack_data()
{
    auto &step = _unpack.step;
    auto &index = _unpack.index;
    auto &header = _unpack.header;
    auto &packet = _unpack.packet;

    while (index > 0)
    {
        switch (step)
        {
            case unpack_step::HEADER_SOF:
                if (packet[0] == 0xFD)
                    step = unpack_step::HEADER;
                else
                    drop(1);
                break;

            case unpack_step::HEADER:
                if (index < HEADER_LENGTH)
                    return;
                std::memcpy(&header, packet, sizeof(header));
                // The only supported incompatibility bit is signing.
                if ((header.incompat_flags & 0xFE) != 0)
                {
                    drop(1);
                    step = unpack_step::HEADER_SOF;
                    break;
                }
                _unpack.frame_length =
                    HEADER_LENGTH + header.payload_length + 2;
                if (header.incompat_flags & 1)
                    _unpack.frame_length += 13;
                step = unpack_step::DATA_CRC16;
                break;

            case unpack_step::DATA_CRC16:
            {
                if (index < _unpack.frame_length)
                    return;
                uint32_t message_id{};
                std::memcpy(&message_id, header.message_id,
                            sizeof(header.message_id));
                if (verify_frame_crc(message_id))
                {
                    solve_data(message_id);
                    drop(_unpack.frame_length);
                }
                else
                {
                    // Search retained bytes for the next valid frame.
                    drop(1);
                }
                step = unpack_step::HEADER_SOF;
                break;
            }
        }
    }
}

bool micovio_parser_t::verify_frame_crc(uint32_t message_id) const
{
    const int extra = crc_extra(message_id);
    if (extra < 0)
        return false;
    const size_t crc_offset = HEADER_LENGTH + _unpack.header.payload_length;
    uint16_t crc =
        get_crc16_check_sum(_unpack.packet + 1, crc_offset - 1, 0xFFFF);
    const uint8_t extra_byte = static_cast<uint8_t>(extra);
    crc = get_crc16_check_sum(&extra_byte, 1, crc);
    uint16_t received_crc;
    std::memcpy(&received_crc, _unpack.packet + crc_offset,
                sizeof(received_crc));
    return crc == received_crc;
}

void micovio_parser_t::solve_data(uint32_t message_id)
{
    const auto &header = _unpack.header;
    if ((_sysid != 0 && header.sysid != _sysid) ||
        (_compid != 0 && header.compid != _compid))
        return;
    _frame.message_id = message_id;
    _frame.payload_length = header.payload_length;
    _frame.sysid = header.sysid;
    _frame.compid = header.compid;
    _frame.seq = header.seq;
    const uint8_t *payload = _unpack.packet + HEADER_LENGTH;
    const size_t payload_length = header.payload_length;

    // Clear omitted trailing bytes before copying a possibly truncated payload.
    switch (message_id)
    {
        case MICOVIO_ODOMETRY_ID:
            _frame.odometry = {};
            std::memcpy(&_frame.odometry, payload,
                        std::min(payload_length, sizeof(_frame.odometry)));
            break;
        case MICOVIO_DISTANCE_ID:
            _frame.distance = {};
            std::memcpy(&_frame.distance, payload,
                        std::min(payload_length, sizeof(_frame.distance)));
            break;
        default:
            break;
    }
    // Signature bytes are framed, but cryptographic authentication is external.
    if (_callback)
        _callback(_frame, _user);
}

} // namespace pyro
