#include "pyro_micovio_drv.h"
#include "pyro_crc.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

using namespace pyro;
static uint32_t now_ms = 0;
extern "C" uint32_t HAL_GetTick(void) { return now_ms; }
static size_t checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); std::exit(1); } } while (false)

using bytes_t = std::vector<uint8_t>;

// Independent bit-wise X.25 implementation for generated frame CRCs.
static uint16_t crc_byte(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
        crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : crc >> 1;
    return crc;
}

static void put_u32(bytes_t &payload, size_t offset, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i)
        payload[offset + i] = static_cast<uint8_t>(value >> (8 * i));
}

static void put_float(bytes_t &payload, size_t offset, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, 4);
    put_u32(payload, offset, bits);
}

static bytes_t frame(uint32_t id, bytes_t payload, uint8_t seq = 0,
                     uint8_t sysid = 1, bool v1 = false, uint8_t flags = 0)
{
    bytes_t result = v1 ? bytes_t{0xFE, static_cast<uint8_t>(payload.size()), seq, sysid, 197,
                                  static_cast<uint8_t>(id)}
                        : bytes_t{0xFD, static_cast<uint8_t>(payload.size()), flags, 0, seq, sysid, 197,
                                  static_cast<uint8_t>(id), static_cast<uint8_t>(id >> 8),
                                  static_cast<uint8_t>(id >> 16)};
    result.insert(result.end(), payload.begin(), payload.end());
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < result.size(); ++i)
        crc = crc_byte(crc, result[i]);
    const uint8_t extra = id == 331 ? 91 : id == 132 ? 85 : id == 0 ? 50 : 143;
    crc = crc_byte(crc, extra);
    result.push_back(static_cast<uint8_t>(crc));
    result.push_back(static_cast<uint8_t>(crc >> 8));
    if (flags & 1)
        result.resize(result.size() + 13);
    return result;
}

static bytes_t odometry_payload(uint8_t reset = 0)
{
    bytes_t payload(233);
    put_u32(payload, 0, 0x12345678);
    put_u32(payload, 4, 2);
    put_float(payload, 8, 1.25f);
    put_float(payload, 12, -2.5f);
    put_float(payload, 20, 1.0f);
    put_float(payload, 36, 0.5f);
    put_float(payload, 60, 0.01f);
    put_float(payload, 60 + 6 * 4, 0.02f);
    put_float(payload, 144 + 15 * 4, std::numeric_limits<float>::quiet_NaN());
    payload[228] = MICOVIO_LOCAL_FRD;
    payload[229] = MICOVIO_BODY_FRD;
    payload[230] = reset;
    payload[231] = 3;
    payload[232] = 75;
    return payload;
}

static bytes_t distance_payload()
{
    bytes_t payload(39);
    payload[4] = 2;
    payload[6] = 0xB0;
    payload[7] = 4;
    payload[8] = 150;
    payload[12] = 25;
    payload[13] = 255;
    payload[38] = 100;
    return payload;
}

static void collect(const micovio_frame_t &packet, void *user)
{
    static_cast<std::vector<micovio_frame_t> *>(user)->push_back(packet);
}

static void test_crc_reuse()
{
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(get_crc16_check_sum(data, sizeof(data), 0xFFFF) == 0x6F91);
    const uint16_t first = get_crc16_check_sum(data, 4, 0xFFFF);
    CHECK(get_crc16_check_sum(data + 4, 5, first) == 0x6F91);
    CHECK(get_crc16_check_sum(data, 0, first) == first);
    uint8_t packet[] = {0xA5, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00};
    append_crc16_check_sum(packet, sizeof(packet));
    CHECK(verify_crc16_check_sum(packet, sizeof(packet)) == 1);
    packet[3] ^= 1;
    CHECK(verify_crc16_check_sum(packet, sizeof(packet)) == 0);
}

static void test_protocol()
{
    CHECK(crc_byte(0xFFFF, '1') == 0x2F8D);
    const auto odom = frame(331, odometry_payload(), 255);
    const auto dist = frame(132, distance_payload(), 0);
    CHECK(sizeof(micovio_header_t) == 10);
    micovio_header_t header{};
    std::memcpy(&header, odom.data(), sizeof(header));
    CHECK(header.magic == 0xFD && header.payload_length == 233);
    CHECK(header.seq == 255 && header.sysid == 1 && header.compid == 197);
    CHECK(header.message_id[0] == 0x4B && header.message_id[1] == 1 &&
          header.message_id[2] == 0);
    bytes_t stream = odom;
    stream.insert(stream.end(), dist.begin(), dist.end());
    for (size_t split = 0; split <= stream.size(); ++split)
    {
        micovio_parser_t parser;
        std::vector<micovio_frame_t> packets;
        parser.set_callback(collect, &packets);
        parser.feed(stream.data(), split);
        parser.feed(stream.data() + split, stream.size() - split);
        CHECK(packets.size() == 2);
        CHECK(packets[0].odometry.time_usec == UINT64_C(0x212345678));
        CHECK(packets[0].odometry.x == 1.25f && packets[0].odometry.y == -2.5f);
        CHECK(packets[0].odometry.vx == 0.5f);
        CHECK(packets[0].odometry.pose_covariance[6] == 0.02f);
        CHECK(std::isnan(packets[0].odometry.velocity_covariance[15]));
        CHECK(packets[1].distance.current_distance_cm == 150);
        CHECK(packets[0].seq == 255 && packets[1].seq == 0);
    }
    micovio_parser_t parser;
    std::vector<micovio_frame_t> packets;
    parser.set_callback(collect, &packets);
    for (uint8_t byte : stream)
        parser.feed(&byte, 1);
    CHECK(packets.size() == 2);
    auto corrupt = odom;
    corrupt[25] ^= 1;
    parser.feed(corrupt.data(), corrupt.size());
    CHECK(packets.size() == 2);
    parser.feed(stream.data(), stream.size());
    CHECK(packets.size() == 4);

    // MAVLink 2 truncation may cut through a float's trailing zero bytes.
    bytes_t truncated(17);
    truncated[16] = 0x80; // 0x00800000, smallest normal float.
    const auto short_frame = frame(132, truncated);
    parser.feed(short_frame.data(), short_frame.size());
    CHECK(packets.back().distance.horizontal_fov == std::numeric_limits<float>::min());
    const auto v1 = frame(132, distance_payload(), 1, 1, true);
    const size_t before_v1 = packets.size();
    parser.feed(v1.data(), v1.size());
    CHECK(packets.size() == before_v1);
    const auto signed_frame = frame(331, odometry_payload(), 2, 1, false, 1);
    parser.feed(signed_frame.data(), signed_frame.size());
    CHECK(packets.back().seq == 2);
    const auto maximum = frame(331, bytes_t(255), 3, 1, false, 1);
    CHECK(maximum.size() == micovio_parser_t::MAX_FRAME_SIZE);
    parser.feed(maximum.data(), maximum.size());
    CHECK(packets.back().seq == 3);
    const auto filtered = frame(331, odometry_payload(), 4, 2);
    const size_t before = packets.size();
    parser.feed(filtered.data(), filtered.size());
    CHECK(packets.size() == before);
    const auto unsupported = frame(132, distance_payload(), 5, 1, false, 2);
    parser.feed(unsupported.data(), unsupported.size());
    CHECK(packets.size() == before);
    parser.feed(stream.data(), stream.size());
    CHECK(packets.size() == before + 2);
    CHECK(packets.back().message_id == 132);
    parser.feed(odom.data(), 100);
    parser.reset();
    parser.feed(dist.data(), dist.size());
    CHECK(packets.back().message_id == 132);
}

static void test_resynchronization()
{
    const auto inner = frame(132, distance_payload(), 41);
    const auto odom = frame(331, odometry_payload(), 42);
    bytes_t payload(233);
    std::copy(inner.begin(), inner.end(), payload.begin() + 20);
    auto corrupt = frame(331, payload, 40);
    corrupt.back() ^= 1;
    bytes_t stream = corrupt;
    stream.insert(stream.end(), odom.begin(), odom.end());
    for (size_t split = 0; split <= stream.size(); ++split)
    {
        micovio_parser_t parser;
        std::vector<micovio_frame_t> packets;
        parser.set_callback(collect, &packets);
        parser.feed(stream.data(), split);
        parser.feed(stream.data() + split, stream.size() - split);
        CHECK(packets.size() == 2);
        CHECK(packets[0].message_id == 132 && packets[0].seq == 41);
        CHECK(packets[0].distance.current_distance_cm == 150);
        CHECK(packets[1].message_id == 331 && packets[1].odometry.x == 1.25f);
    }
    // A valid frame from another source is consumed in full.
    micovio_parser_t parser;
    std::vector<micovio_frame_t> packets;
    parser.set_callback(collect, &packets);
    const auto filtered = frame(331, payload, 40, 2);
    parser.feed(filtered.data(), filtered.size());
    CHECK(packets.empty());
    parser.feed(odom.data(), odom.size());
    CHECK(packets.size() == 1 && packets[0].message_id == 331);
}

static void poll()
{
    CHECK(task_base_t::last_started != nullptr);
    task_base_t::last_started->run_once();
}

static void test_component()
{
    now_ms = 0;
    uart_drv_t uart;
    micovio_drv_t vio(uart);
    const micovio_odometry_t &odometry = vio.get_odometry();
    const micovio_distance_t &distance = vio.get_distance();
    CHECK(odometry.time_usec == 0 && distance.current_distance_cm == 0);
    CHECK(&odometry == &vio.get_odometry());
    CHECK(&distance == &vio.get_distance());
    vio.init();
    CHECK(uart.callback && !uart.receiving);
    const auto *initial_task = task_base_t::last_started;
    vio.init();
    CHECK(task_base_t::last_started == initial_task);
    CHECK(uart.enable_rx_dma() == PYRO_OK);
    const auto odom = frame(331, odometry_payload(), 255);
    uart.inject(bytes_t(odom.begin(), odom.begin() + 100));
    poll();
    CHECK(odometry.time_usec == 0);
    now_ms = 10;
    uart.inject(bytes_t(odom.begin() + 100, odom.end()));
    now_ms = 20;
    poll();
    CHECK(odometry.time_usec == UINT64_C(0x212345678));
    CHECK(odometry.x == 1.25f);
    now_ms = 121;
    uart.inject(frame(132, distance_payload(), 0));
    poll();
    CHECK(distance.current_distance_cm == 150);
    CHECK(odometry.x == 1.25f);
    uart.inject(frame(331, odometry_payload(), 1));
    poll();
    auto duplicate = odometry_payload();
    put_float(duplicate, 8, 9.0f);
    now_ms = 122;
    uart.inject(frame(331, duplicate, 1));
    poll();
    CHECK(odometry.x == 1.25f);
    uart.inject(frame(331, odometry_payload(1), 1));
    poll();
    CHECK(odometry.reset_counter == 1);
    auto degraded = odometry_payload(1);
    put_float(degraded, 60, 12.0f);
    put_float(degraded, 60 + 6 * 4, 16.0f);
    degraded[232] = 49;
    uart.inject(frame(331, degraded, 2));
    poll();
    CHECK(odometry.pose_covariance[0] == 12.0f);
    CHECK(odometry.pose_covariance[6] == 16.0f && odometry.quality == 49);
    auto invalid_distance = distance_payload();
    invalid_distance[8] = invalid_distance[9] = 255;
    uart.inject(frame(132, invalid_distance, 3));
    poll();
    CHECK(distance.current_distance_cm == 0xFFFF);
    for (size_t i = 0; i < 20; ++i)
        uart.inject(bytes_t(512, 0));
    poll();
    CHECK(odometry.quality == 49);
    uart.inject(frame(331, odometry_payload(1), 4));
    poll();
    CHECK(odometry.quality == 75);
    auto stale = odometry_payload(1);
    put_float(stale, 8, 9.0f);
    uart.inject(frame(331, stale, 5));
    now_ms += 101;
    poll();
    CHECK(odometry.x == 1.25f);
    now_ms = UINT32_MAX - 5;
    uart.inject(frame(331, stale, 6));
    poll();
    now_ms = 3;
    CHECK(odometry.x == 9.0f);
    CHECK(&odometry == &vio.get_odometry());
    CHECK(&distance == &vio.get_distance());
}

static void test_init_failures()
{
    uart_drv_t uart;
    micovio_drv_t vio(uart);
    micovio_config_t config;
    config.timeout_ms = 0;
    vio.init(config);
    CHECK(!uart.callback);
    task_base_t::fail_creation = true;
    vio.init();
    CHECK(!uart.callback);
    task_base_t::fail_creation = false;
    vio.init();
    CHECK(uart.callback && !uart.receiving);
}

static void test_external_uart_lifecycle()
{
    uart_drv_t uart;
    CHECK(uart.reset(921600, UART_WORDLENGTH_8B,
                     UART_STOPBITS_1, UART_PARITY_NONE) == PYRO_OK);
    CHECK(uart.enable_rx_dma() == PYRO_OK);
    uart.reset_result = PYRO_ERROR;
    uart.rx_result = PYRO_ERROR;
    {
        micovio_drv_t vio(uart);
        vio.init();
        CHECK(uart.receiving && uart.baud == 921600);
        uart.inject(frame(331, odometry_payload()));
        poll();
        CHECK(vio.get_odometry().x == 1.25f);
    }
    CHECK(uart.receiving && !uart.callback);
}

static void test_independent_streams()
{
    now_ms = 0;
    uart_drv_t uart;
    micovio_drv_t vio(uart);
    vio.init();
    CHECK(uart.enable_rx_dma() == PYRO_OK);
    const auto &odometry = vio.get_odometry();
    const auto &distance = vio.get_distance();
    uart.inject(frame(331, odometry_payload(), 0));
    poll();
    CHECK(odometry.x == 1.25f && distance.current_distance_cm == 0);
    uart.inject(frame(132, distance_payload(), 1));
    poll();
    CHECK(distance.current_distance_cm == 150 && odometry.x == 1.25f);
    uint8_t seq = 2;
    for (size_t offset : {size_t{8}, size_t{36}, size_t{20}, size_t{60}})
    {
        auto payload = odometry_payload();
        put_float(payload, offset, std::numeric_limits<float>::quiet_NaN());
        uart.inject(frame(331, payload, seq++));
        poll();
        CHECK(std::memcmp(&odometry, payload.data(), sizeof(odometry)) == 0);
        CHECK(distance.current_distance_cm == 150);
    }
}

static void replay(const char *path)
{
    std::ifstream file(path, std::ios::binary);
    CHECK(file.is_open());
    bytes_t bytes((std::istreambuf_iterator<char>(file)), {});
    std::vector<micovio_frame_t> baseline;
    for (size_t chunk : {size_t{1}, size_t{7}, size_t{51}, size_t{245}, size_t{512}, size_t{4096}})
    {
        micovio_parser_t parser;
        std::vector<micovio_frame_t> packets;
        parser.set_callback(collect, &packets);
        for (size_t offset = 0; offset < bytes.size(); offset += chunk)
            parser.feed(bytes.data() + offset, std::min(chunk, bytes.size() - offset));
        if (chunk == 1)
            baseline = packets;
        CHECK(packets.size() == baseline.size());
        for (size_t i = 0; i < packets.size(); ++i)
        {
            CHECK(packets[i].message_id == baseline[i].message_id);
            CHECK(packets[i].seq == baseline[i].seq);
            CHECK(std::memcmp(&packets[i].odometry, &baseline[i].odometry,
                              sizeof(packets[i].odometry)) == 0);
            CHECK(std::memcmp(&packets[i].distance, &baseline[i].distance,
                              sizeof(packets[i].distance)) == 0);
        }
    }
    now_ms = 0;
    uart_drv_t uart;
    micovio_drv_t vio(uart);
    vio.init();
    CHECK(uart.enable_rx_dma() == PYRO_OK);
    for (size_t offset = 0; offset < bytes.size(); offset += 512)
    {
        now_ms = static_cast<uint32_t>(offset / 24); // Pace callbacks near 24 KB/s.
        const size_t end = std::min(offset + 512, bytes.size());
        uart.inject(bytes_t(bytes.begin() + offset, bytes.begin() + end));
        poll();
    }
    const auto &odometry = vio.get_odometry();
    const auto &distance = vio.get_distance();
    const micovio_frame_t latest = baseline.empty() ? micovio_frame_t{} : baseline.back();
    CHECK(std::memcmp(&odometry, &latest.odometry, sizeof(odometry)) == 0);
    CHECK(std::memcmp(&distance, &latest.distance, sizeof(distance)) == 0);
    const auto odometry_count = std::count_if(baseline.begin(), baseline.end(),
        [](const micovio_frame_t &packet) { return packet.message_id == MICOVIO_ODOMETRY_ID; });
    const auto distance_count = std::count_if(baseline.begin(), baseline.end(),
        [](const micovio_frame_t &packet) { return packet.message_id == MICOVIO_DISTANCE_ID; });
    std::printf("Replay %s: odometry=%zu distance=%zu\n", path,
                static_cast<size_t>(odometry_count), static_cast<size_t>(distance_count));
}

int main(int argc, char **argv)
{
    test_crc_reuse();
    test_protocol();
    test_resynchronization();
    test_component();
    test_init_failures();
    test_external_uart_lifecycle();
    test_independent_streams();
    for (int i = 1; i < argc; ++i)
        replay(argv[i]);
    std::printf("Passed %zu checks\n", checks);
}
