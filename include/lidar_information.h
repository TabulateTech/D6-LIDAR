#ifndef LIDAR_INFORMATION_H
#define LIDAR_INFORMATION_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

constexpr double PI = 3.14159265358979323846;
constexpr std::uint8_t START_LIDAR_CMD[4] = {0xAA, 0x55, 0xF0, 0x0F};
constexpr std::uint8_t STOP_LIDAR_CMD[4]  = {0xAA, 0x55, 0xF5, 0x0A};
constexpr std::uint8_t START_ACK_OK[12]    = {0xA5, 0x5A, 0x50, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA8};
constexpr std::uint8_t START_ACK_ERROR[12] = {0xA5, 0x5A, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9};
constexpr std::uint8_t STOP_ACK_OK[12]     = {0xA5, 0x5A, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAD};
constexpr std::uint8_t STOP_ACK_ERROR[12]  = {0xA5, 0x5A, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9};

constexpr std::uint16_t DATA_PACKET_HEADER = 0x55AA;
constexpr std::size_t MAX_SCAN_NODES = 800;

using result_t = std::int32_t;
constexpr result_t RESULT_OK = 0;
constexpr result_t RESULT_TIMEOUT = -1;
constexpr result_t RESULT_FAIL = -2;

inline bool is_ok(result_t value) { return value == RESULT_OK; }
inline bool is_timeout(result_t value) { return value == RESULT_TIMEOUT; }
inline bool is_fail(result_t value) { return value == RESULT_FAIL; }

struct DebugOptions {
    bool enabled = false;
    bool verbose = false;
    bool hex_dump_on_error = true;
    std::uint32_t print_every_ms = 2000;
    std::string dump_serial_path;
    std::size_t dump_serial_limit_bytes = 262144;
};

struct LidarDiagnosticsSnapshot {
    std::uint64_t bytes_ingested = 0;
    std::uint64_t single_byte_reads = 0;
    std::uint64_t bulk_reads = 0;
    std::uint64_t read_timeouts = 0;
    std::uint64_t available_polls = 0;

    std::uint64_t outer_frames_ok = 0;
    std::uint64_t outer_info_frames = 0;
    std::uint64_t outer_data_frames = 0;
    std::uint64_t outer_other_frames = 0;
    std::uint64_t outer_checksum_fail = 0;
    std::uint64_t outer_invalid_length = 0;

    std::uint64_t inner_packets_ok = 0;
    std::uint64_t inner_start_packets = 0;
    std::uint64_t inner_data_packets = 0;
    std::uint64_t inner_checksum_fail = 0;
    std::uint64_t inner_invalid_length = 0;
    std::uint64_t zero_lsn_packets = 0;
    std::uint64_t ring_starts = 0;
    std::uint64_t angle_rollover_starts = 0;
    std::uint64_t checksum_bypass_packets = 0;

    std::uint64_t resync_discards = 0;
    std::uint64_t speed_bytes_discarded = 0;
    std::uint64_t start_commands_sent = 0;
    std::uint64_t stop_commands_sent = 0;
    std::uint64_t start_ack_ok = 0;
    std::uint64_t start_ack_error = 0;
    std::uint64_t stop_ack_ok = 0;
    std::uint64_t stop_ack_error = 0;
    std::uint64_t ack_timeouts = 0;
    std::size_t stream_buffer_size = 0;
    std::size_t raw_buffer_size = 0;

    std::string last_error;
    std::string last_ack;
    std::string recent_hex;
};

struct LidarCoverAngleStr {
    float f_begin = 0.0f;
    float f_end = 0.0f;
};

struct LaserConfig {
    float min_angle = 0.0f;
    float max_angle = static_cast<float>(2.0 * PI);
    float angle_increment = 0.0f;
    float time_increment = 0.0f;
    float scan_time = 0.0f;
    float min_range = 0.10f;
    float max_range = 10.0f;
};

struct LaserPoint {
    float angle = 0.0f;          // grados
    float range = 0.0f;          // metros
    std::uint16_t intensity = 0;
    short range_check = 0;
    float x = 0.0f;              // metros
    float y = 0.0f;              // metros
    bool high_reflection = false;
};

struct LaserScan {
    std::uint64_t stamp_ns = 0;
    std::vector<LaserPoint> points;
    LaserConfig config;
    float scan_frequency_hz = 0.0f;
};

struct RawNode {
    bool sync_flag = false;
    float angle_deg = 0.0f;
    float distance_m = 0.0f;
    std::uint16_t intensity = 0;
    bool high_reflection = false;
    std::uint64_t stamp_ns = 0;
    float scan_frequency_hz = 0.0f;
};

struct LidarGeneralInfo {
    std::string port = "/dev/ttyUSB0";
    std::uint32_t baudrate = 230400;
    float range_min_m = 0.10f;
    float range_max_m = 10.0f;
    bool filter_enable = true;
};

struct LidarRobotInfo {
    float install_to_zero = 0.0f;
    int robot_diameter_mm = 0;
    int lidar_robot_center_distance_mm = 0;
    bool lidar_cover_enable = false;
    std::vector<LidarCoverAngleStr> cover_angles;
};

struct LidarStatus {
    bool connected = false;
    bool running = false;
    bool filter_enable = true;
    bool simulate = false;
};

struct DeviceInfo {
    std::string model;
    std::uint8_t revision = 0;
    bool clockwise = true;
    bool needs_angle_correction = true;
};

#endif
