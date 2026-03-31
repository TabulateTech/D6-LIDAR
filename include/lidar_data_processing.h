#ifndef LIDAR_DATA_PROCESSING_H
#define LIDAR_DATA_PROCESSING_H

#include "lidar_information.h"
#include "serial_port.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class LidarDataProcessing {
public:
    explicit LidarDataProcessing(SerialPort& serial_port);
    ~LidarDataProcessing();

    bool sendStartCommand();
    bool sendStopCommand();
    bool tryReadDeviceInfo(DeviceInfo& info, std::uint32_t timeout_ms = 500);

    bool readPacket(std::vector<RawNode>& packet_nodes,
                    bool& ring_start,
                    float& scan_frequency_hz,
                    std::uint32_t timeout_ms = 1000);

    void setDebugOptions(const DebugOptions& options);
    LidarDiagnosticsSnapshot diagnostics() const;

private:
    SerialPort& serial_port_;
    std::vector<std::uint8_t> stream_buffer_;
    std::vector<std::uint8_t> raw_buffer_;
    DebugOptions debug_options_;
    LidarDiagnosticsSnapshot diagnostics_;
    std::vector<std::uint8_t> recent_serial_bytes_;
    std::ofstream serial_dump_;
    std::size_t serial_dump_written_ = 0;

    bool ingestSerial(std::uint32_t timeout_ms);
    bool extractOuterFrame(std::uint8_t& type, std::vector<std::uint8_t>& payload);
    bool tryParseRawPacket(std::vector<RawNode>& packet_nodes,
                           bool& ring_start,
                           float& scan_frequency_hz);
    bool readExact(std::uint8_t* buffer, std::size_t size, std::uint32_t timeout_ms);
    static std::uint16_t computeChecksum(const std::vector<std::uint8_t>& packet);
    static float normalizeAngleDeg(float angle_deg);

    void appendRecentBytes(const std::uint8_t* data, std::size_t size);
    void dumpSerialBytes(const std::uint8_t* data, std::size_t size);
    void setLastError(std::string message);
    static std::string bytesToHex(const std::vector<std::uint8_t>& bytes, std::size_t max_count);
};

#endif
