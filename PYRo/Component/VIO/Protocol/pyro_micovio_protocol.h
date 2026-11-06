#ifndef PYRO_MICOVIO_PROTOCOL_H
#define PYRO_MICOVIO_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace pyro
{

constexpr uint32_t MICOVIO_ODOMETRY_ID = 331;
constexpr uint32_t MICOVIO_DISTANCE_ID = 132;
constexpr uint8_t MICOVIO_LOCAL_FRD = 20;
constexpr uint8_t MICOVIO_BODY_FRD = 12;

// Wire payloads use byte packing and little-endian IEEE-754 values.
#pragma pack(push, 1)
struct micovio_header_t
{
    uint8_t magic{};
    uint8_t payload_length{};
    uint8_t incompat_flags{};
    uint8_t compat_flags{};
    uint8_t seq{};
    uint8_t sysid{};
    uint8_t compid{};
    uint8_t message_id[3]{}; // MAVLink 2 uses a 24-bit little-endian ID.
};

struct micovio_odometry_t
{
    uint64_t time_usec{}; // Module clock, independent of MCU reception time.
    float x{}, y{}, z{}; // m, in frame_id.
    float q[4]{};       // w,x,y,z, body -> reference frame.
    float vx{}, vy{}, vz{}; // m/s, in child_frame_id.
    float rollspeed{}, pitchspeed{}, yawspeed{}; // rad/s, body frame.
    float pose_covariance[21]{}; // Upper triangle of a 6x6 matrix.
    float velocity_covariance[21]{};
    uint8_t frame_id{};
    uint8_t child_frame_id{};
    uint8_t reset_counter{};
    uint8_t estimator_type{};
    int8_t quality{};
};

struct micovio_distance_t
{
    uint32_t time_boot_ms{};
    uint16_t min_distance_cm{};
    uint16_t max_distance_cm{};
    uint16_t current_distance_cm{}; // 0xFFFF means unavailable.
    uint8_t type{};
    uint8_t id{};
    uint8_t orientation{};
    uint8_t covariance{}; // 255 means unknown.
    float horizontal_fov{};
    float vertical_fov{};
    float quaternion[4]{};
    uint8_t signal_quality{}; // 0 unknown, 1 invalid, 2..100 valid.
};
#pragma pack(pop)

struct micovio_frame_t
{
    uint32_t message_id{};
    uint8_t payload_length{};
    uint8_t sysid{}, compid{}, seq{};
    // Only the member selected by message_id is updated.
    micovio_odometry_t odometry{};
    micovio_distance_t distance{};
};

// Allocation-free MAVLink 2 parser for little-endian MicoVIO payloads.
// Feed and callback execute synchronously in a single caller's context.
class micovio_parser_t
{
  public:
    using frame_callback_t = void (*)(const micovio_frame_t &, void *);
    static constexpr size_t MAX_FRAME_SIZE = 280;

    explicit micovio_parser_t(uint8_t sysid = 1, uint8_t compid = 197);
    void set_callback(frame_callback_t callback, void *user);
    void feed(const uint8_t *data, size_t length);
    void reset(); // Discard partial input, retain configuration and callback.

  private:
    enum class unpack_step
    {
        HEADER_SOF,
        HEADER,
        DATA_CRC16
    };

    struct unpack_context
    {
        unpack_step step{unpack_step::HEADER_SOF};
        size_t index{};
        size_t frame_length{};
        micovio_header_t header{};
        uint8_t packet[MAX_FRAME_SIZE]{};
    };

    void unpack_data();
    void drop(size_t length);
    bool verify_frame_crc(uint32_t message_id) const;
    void solve_data(uint32_t message_id);

    unpack_context _unpack{};
    uint8_t _sysid;
    uint8_t _compid;
    frame_callback_t _callback{};
    void *_user{};
    micovio_frame_t _frame{};
};

} // namespace pyro

#endif
