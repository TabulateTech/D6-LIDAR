#include "lidar_data_processing.h"

#include "mtime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

std::uint16_t read_u16_le(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0] | (static_cast<std::uint16_t>(data[1]) << 8));
}

constexpr std::size_t MAX_OUTER_PAYLOAD = 4096;
constexpr std::size_t MAX_INNER_PACKET_SIZE = 2048;
constexpr std::size_t RECENT_HEX_BYTES = 96;
constexpr float ROLLOVER_LOW_DEG = 45.0f;
constexpr float ROLLOVER_HIGH_DEG = 315.0f;

} // namespace

LidarDataProcessing::LidarDataProcessing(SerialPort& serial_port)
    : serial_port_(serial_port) {}

LidarDataProcessing::~LidarDataProcessing() {
    if (serial_dump_.is_open()) {
        serial_dump_.flush();
        serial_dump_.close();
    }
}

bool LidarDataProcessing::sendStartCommand() {
    diagnostics_.start_commands_sent++;
    if (serial_port_.writeData(START_LIDAR_CMD, sizeof(START_LIDAR_CMD)) != RESULT_OK) {
        setLastError("No se pudo transmitir el comando de inicio");
        return false;
    }
    return waitForAck(START_ACK_OK, sizeof(START_ACK_OK), START_ACK_ERROR, sizeof(START_ACK_ERROR),
                      800, "inicio", true);
}

bool LidarDataProcessing::sendStopCommand() {
    diagnostics_.stop_commands_sent++;
    if (serial_port_.writeData(STOP_LIDAR_CMD, sizeof(STOP_LIDAR_CMD)) != RESULT_OK) {
        setLastError("No se pudo transmitir el comando de parada");
        return false;
    }
    return waitForAck(STOP_ACK_OK, sizeof(STOP_ACK_OK), STOP_ACK_ERROR, sizeof(STOP_ACK_ERROR),
                      800, "parada", false);
}

void LidarDataProcessing::setDebugOptions(const DebugOptions& options) {
    debug_options_ = options;
    if (serial_dump_.is_open()) {
        serial_dump_.flush();
        serial_dump_.close();
    }
    serial_dump_written_ = 0;
    if (!debug_options_.dump_serial_path.empty()) {
        serial_dump_.open(debug_options_.dump_serial_path, std::ios::binary | std::ios::trunc);
        if (!serial_dump_) {
            setLastError("No se pudo abrir el archivo de volcado serial: " + debug_options_.dump_serial_path);
        }
    }
}

LidarDiagnosticsSnapshot LidarDataProcessing::diagnostics() const {
    LidarDiagnosticsSnapshot snapshot = diagnostics_;
    snapshot.stream_buffer_size = stream_buffer_.size();
    snapshot.raw_buffer_size = raw_buffer_.size();
    snapshot.recent_hex = bytesToHex(recent_serial_bytes_, RECENT_HEX_BYTES);
    return snapshot;
}

bool LidarDataProcessing::tryReadDeviceInfo(DeviceInfo& info, std::uint32_t timeout_ms) {
    const auto deadline = current_time_ms() + timeout_ms;

    while (current_time_ms() < deadline) {
        std::uint8_t type = 0;
        std::vector<std::uint8_t> payload;
        if (extractOuterFrame(type, payload)) {
            if (type != 0x01u) {
                continue;
            }

            std::string model;
            for (std::size_t i = 0; i < std::min<std::size_t>(12, payload.size()); ++i) {
                if (payload[i] == 0) {
                    break;
                }
                model.push_back(static_cast<char>(payload[i]));
            }

            info.model = model.empty() ? "COIN-D6" : model;
            if (payload.size() >= 20) {
                info.clockwise = payload[14] == 0x00;
                info.needs_angle_correction = payload[15] == 0x01;
                info.revision = payload[19];
            }
            return true;
        }

        if (!ingestSerial(std::min<std::uint32_t>(20, static_cast<std::uint32_t>(deadline - current_time_ms())))) {
            continue;
        }
    }

    return false;
}

bool LidarDataProcessing::readPacket(std::vector<RawNode>& packet_nodes,
                                     bool& ring_start,
                                     float& scan_frequency_hz,
                                     std::uint32_t timeout_ms) {
    packet_nodes.clear();
    ring_start = false;
    scan_frequency_hz = 0.0f;

    const auto deadline = current_time_ms() + timeout_ms;
    while (current_time_ms() < deadline) {
        if (tryParseRawPacket(packet_nodes, ring_start, scan_frequency_hz)) {
            return true;
        }

        std::uint8_t outer_type = 0;
        std::vector<std::uint8_t> payload;
        while (extractOuterFrame(outer_type, payload)) {
            if ((outer_type == 0x81u || outer_type == 0x01u) && !payload.empty()) {
                raw_buffer_.insert(raw_buffer_.end(), payload.begin(), payload.end());
                if (tryParseRawPacket(packet_nodes, ring_start, scan_frequency_hz)) {
                    return true;
                }
            }
        }

        if (!ingestSerial(std::min<std::uint32_t>(20, static_cast<std::uint32_t>(deadline - current_time_ms())))) {
            continue;
        }
    }

    return false;
}

bool LidarDataProcessing::ingestSerial(std::uint32_t timeout_ms) {
    diagnostics_.available_polls++;

    if (timeout_ms == 0) {
        timeout_ms = 1;
    }

    std::size_t n = serial_port_.available();
    if (n == 0) {
        std::uint8_t one = 0;
        if (serial_port_.readByte(one, timeout_ms) != RESULT_OK) {
            diagnostics_.read_timeouts++;
            return false;
        }
        diagnostics_.single_byte_reads++;
        diagnostics_.bytes_ingested++;
        stream_buffer_.push_back(one);
        appendRecentBytes(&one, 1);
        dumpSerialBytes(&one, 1);
        return true;
    }

    n = std::min<std::size_t>(n, 4096);
    std::vector<std::uint8_t> temp(n);
    std::size_t received = 0;
    const result_t rc = serial_port_.readData(temp.data(), temp.size(), received, timeout_ms);
    if (rc == RESULT_FAIL || received == 0) {
        if (rc == RESULT_TIMEOUT) {
            diagnostics_.read_timeouts++;
        }
        return false;
    }

    diagnostics_.bulk_reads++;
    diagnostics_.bytes_ingested += received;
    stream_buffer_.insert(stream_buffer_.end(), temp.begin(), temp.begin() + static_cast<std::ptrdiff_t>(received));
    appendRecentBytes(temp.data(), received);
    dumpSerialBytes(temp.data(), received);
    return true;
}

bool LidarDataProcessing::extractOuterFrame(std::uint8_t& type, std::vector<std::uint8_t>& payload) {
    payload.clear();

    while (stream_buffer_.size() >= 2) {
        if (stream_buffer_[0] == 0xA5u && stream_buffer_[1] == 0x5Au) {
            break;
        }

        const std::uint8_t b = stream_buffer_.front();
        raw_buffer_.push_back(b);
        diagnostics_.resync_discards++;
        stream_buffer_.erase(stream_buffer_.begin());
    }

    if (stream_buffer_.size() < 7) {
        return false;
    }

    const std::uint16_t length = read_u16_le(&stream_buffer_[2]);
    const std::uint16_t checksum = read_u16_le(&stream_buffer_[4]);
    const std::uint8_t packet_type = stream_buffer_[6];

    if (length > MAX_OUTER_PAYLOAD) {
        diagnostics_.outer_invalid_length++;
        setLastError("Longitud inválida de frame externo: " + std::to_string(length));
        stream_buffer_.erase(stream_buffer_.begin());
        return false;
    }

    const std::size_t total = 7u + static_cast<std::size_t>(length);
    if (stream_buffer_.size() < total) {
        return false;
    }

    std::uint16_t sum = 0;
    sum = static_cast<std::uint16_t>(sum + 0xA5u + 0x5Au + stream_buffer_[2] + stream_buffer_[3] + packet_type);
    for (std::size_t i = 0; i < length; ++i) {
        sum = static_cast<std::uint16_t>(sum + stream_buffer_[7 + i]);
    }

    if (sum != checksum) {
        diagnostics_.outer_checksum_fail++;
        setLastError("Checksum externo inválido. esperado=" + std::to_string(checksum) +
                     " calculado=" + std::to_string(sum));
        raw_buffer_.push_back(stream_buffer_.front());
        diagnostics_.resync_discards++;
        stream_buffer_.erase(stream_buffer_.begin());
        return false;
    }

    diagnostics_.outer_frames_ok++;
    if (packet_type == 0x01u) {
        diagnostics_.outer_info_frames++;
    } else if (packet_type == 0x81u) {
        diagnostics_.outer_data_frames++;
    } else {
        diagnostics_.outer_other_frames++;
    }

    payload.assign(stream_buffer_.begin() + 7, stream_buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    stream_buffer_.erase(stream_buffer_.begin(), stream_buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    type = packet_type;
    return true;
}

bool LidarDataProcessing::tryParseRawPacket(std::vector<RawNode>& packet_nodes,
                                            bool& ring_start,
                                            float& scan_frequency_hz) {
    packet_nodes.clear();
    ring_start = false;
    scan_frequency_hz = 0.0f;

    // Resync to the inner packet header AA 55 (0x55AA in little-endian).
    std::size_t start = 0;
    while (start + 1 < raw_buffer_.size()) {
        if (raw_buffer_[start] == 0xAAu && raw_buffer_[start + 1] == 0x55u) {
            break;
        }

        const std::uint8_t b = raw_buffer_[start];
        if (b == 0xFEu || b == 0xFFu) {
            // The datasheet mentions speed-adjust bytes during spin-up; keep counting them
            // but do not feed them into packet parsing.
            diagnostics_.speed_bytes_discarded++;
        }
        ++start;
    }

    if (start > 0) {
        diagnostics_.resync_discards += start;
        raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + static_cast<std::ptrdiff_t>(start));
    }

    if (raw_buffer_.size() < 10) {
        return false;
    }

    const std::uint8_t mt = raw_buffer_[2];
    const std::uint8_t lsn = raw_buffer_[3];
    const std::uint16_t fsa_raw = read_u16_le(&raw_buffer_[4]);
    const std::uint16_t lsa_raw = read_u16_le(&raw_buffer_[6]);
    const std::uint16_t packet_checksum = read_u16_le(&raw_buffer_[8]);

    const std::uint8_t package_type = static_cast<std::uint8_t>(mt & 0x01u);
    if (package_type != 0u && package_type != 1u) {
        diagnostics_.inner_invalid_length++;
        setLastError("Tipo de paquete interno inválido: mt=" + std::to_string(mt));
        raw_buffer_.erase(raw_buffer_.begin());
        return false;
    }

    if (lsn == 0) {
        diagnostics_.zero_lsn_packets++;
        setLastError("Paquete interno con LSN=0");
        raw_buffer_.erase(raw_buffer_.begin());
        return false;
    }

    if ((fsa_raw & 0x01u) == 0u || (lsa_raw & 0x01u) == 0u) {
        diagnostics_.inner_invalid_length++;
        setLastError("FSA/LSA sin checkbit activo");
        raw_buffer_.erase(raw_buffer_.begin());
        return false;
    }

    const std::size_t packet_size = 10u + static_cast<std::size_t>(lsn) * 3u;
    if (packet_size > MAX_INNER_PACKET_SIZE) {
        diagnostics_.inner_invalid_length++;
        setLastError("Longitud inválida de paquete interno: " + std::to_string(packet_size));
        raw_buffer_.erase(raw_buffer_.begin());
        return false;
    }
    if (raw_buffer_.size() < packet_size) {
        return false;
    }

    std::vector<std::uint8_t> packet(raw_buffer_.begin(), raw_buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
    const std::uint16_t computed_checksum = computeChecksum(packet);
    if (computed_checksum != packet_checksum) {
        diagnostics_.inner_checksum_fail++;
        setLastError("Checksum interno inválido. esperado=" + std::to_string(packet_checksum) +
                     " calculado=" + std::to_string(computed_checksum));
        raw_buffer_.erase(raw_buffer_.begin());
        return false;
    }

    raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));

    ring_start = package_type == 1u || lsn == 1u;
    scan_frequency_hz = (package_type == 1u) ? static_cast<float>(((mt & 0xFEu) >> 1) / 10.0f) : 0.0f;

    diagnostics_.inner_packets_ok++;
    if (ring_start) {
        diagnostics_.inner_start_packets++;
        diagnostics_.ring_starts++;
    } else {
        diagnostics_.inner_data_packets++;
    }

    const float angle_fsa = static_cast<float>((fsa_raw >> 1) / 64.0f);
    const float angle_lsa = static_cast<float>((lsa_raw >> 1) / 64.0f);

    float delta = angle_lsa - angle_fsa;
    if (lsn > 1 && delta < 0.0f) {
        if (angle_fsa > 270.0f && angle_lsa < 90.0f) {
            delta += 360.0f;
        }
    }
    const float increment = (lsn > 1) ? delta / static_cast<float>(lsn - 1) : 0.0f;

    packet_nodes.reserve(lsn);
    const std::uint64_t stamp_ns = current_time_ns();
    for (std::size_t i = 0; i < lsn; ++i) {
        const std::uint8_t s0 = packet[10 + 3 * i + 0];
        const std::uint8_t s1 = packet[10 + 3 * i + 1];
        const std::uint8_t s2 = packet[10 + 3 * i + 2];

        const std::uint16_t intensity = static_cast<std::uint16_t>(((s1 & 0x03u) * 64u) + (s0 >> 2));
        const std::uint16_t distance_mm = static_cast<std::uint16_t>((static_cast<std::uint16_t>(s2) * 64u) + (s1 >> 2));
        const bool high_reflection = (s0 & 0x01u) != 0u;

        RawNode node;
        node.sync_flag = ring_start && i == 0;
        node.angle_deg = normalizeAngleDeg(angle_fsa + increment * static_cast<float>(i));
        node.distance_m = static_cast<float>(distance_mm) / 1000.0f;
        node.intensity = intensity;
        node.high_reflection = high_reflection;
        node.stamp_ns = stamp_ns;
        node.scan_frequency_hz = scan_frequency_hz;
        packet_nodes.push_back(node);
    }

    return true;
}

bool LidarDataProcessing::readExact(std::uint8_t* buffer, std::size_t size, std::uint32_t timeout_ms) {
    std::size_t received = 0;
    const result_t rc = serial_port_.readData(buffer, size, received, timeout_ms);
    return rc == RESULT_OK && received == size;
}

std::uint16_t LidarDataProcessing::computeChecksum(const std::vector<std::uint8_t>& packet) {
    if (packet.size() < 10) {
        return 0;
    }

    // This matches the original vendor driver logic for the intensity-enabled packets:
    //   checksum = PH ^ FSA_raw ^ LSA_raw ^ SampleNumAndCT
    //   for each 3-byte sample: xor sample[0], then xor (sample[1] | sample[2]<<8)
    // The D6 manual also states that the 3rd sample byte is padded on the MSB side
    // when forming 16-bit words, but the vendor implementation used for D2/D6-style
    // packets effectively groups the sample bytes as [s0] and [s1,s2].
    std::uint16_t checksum = DATA_PACKET_HEADER;
    const std::uint16_t sample_num_and_ct = read_u16_le(&packet[2]);
    const std::uint16_t fsa_raw = read_u16_le(&packet[4]);
    const std::uint16_t lsa_raw = read_u16_le(&packet[6]);

    checksum ^= fsa_raw;

    std::uint16_t value = 0;
    for (std::size_t i = 10, payload_index = 0; i < packet.size(); ++i, ++payload_index) {
        const std::uint8_t current = packet[i];
        if ((payload_index % 3u) == 2u) {
            value = static_cast<std::uint16_t>(value + static_cast<std::uint16_t>(current) * 0x100u);
            checksum ^= value;
        } else if ((payload_index % 3u) == 1u) {
            value = current;
        } else {
            value = current;
            checksum ^= current;
        }
    }

    checksum ^= sample_num_and_ct;
    checksum ^= lsa_raw;
    return checksum;
}

float LidarDataProcessing::normalizeAngleDeg(float angle_deg) {
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

bool LidarDataProcessing::waitForAck(const std::uint8_t* ok_ack,
                                     std::size_t ok_size,
                                     const std::uint8_t* error_ack,
                                     std::size_t error_size,
                                     std::uint32_t timeout_ms,
                                     const char* command_name,
                                     bool is_start_command) {
    std::vector<std::uint8_t> ack_buffer;
    const auto deadline = current_time_ms() + timeout_ms;

    auto preserve_non_ack_bytes = [&](std::size_t match_pos, std::size_t match_size) {
        if (match_pos > 0) {
            stream_buffer_.insert(stream_buffer_.end(), ack_buffer.begin(),
                                  ack_buffer.begin() + static_cast<std::ptrdiff_t>(match_pos));
        }
        const std::size_t tail_begin = match_pos + match_size;
        if (tail_begin < ack_buffer.size()) {
            stream_buffer_.insert(stream_buffer_.end(),
                                  ack_buffer.begin() + static_cast<std::ptrdiff_t>(tail_begin),
                                  ack_buffer.end());
        }
        ack_buffer.clear();
    };

    while (current_time_ms() < deadline) {
        const std::size_t ok_pos = findPattern(ack_buffer, ok_ack, ok_size);
        if (ok_pos != std::string::npos) {
            diagnostics_.last_ack = bytesToHex(std::vector<std::uint8_t>(ok_ack, ok_ack + ok_size), ok_size);
            preserve_non_ack_bytes(ok_pos, ok_size);
            if (is_start_command) {
                diagnostics_.start_ack_ok++;
            } else {
                diagnostics_.stop_ack_ok++;
            }
            return true;
        }

        const std::size_t error_pos = findPattern(ack_buffer, error_ack, error_size);
        if (error_pos != std::string::npos) {
            diagnostics_.last_ack = bytesToHex(std::vector<std::uint8_t>(error_ack, error_ack + error_size), error_size);
            preserve_non_ack_bytes(error_pos, error_size);
            if (is_start_command) {
                diagnostics_.start_ack_error++;
            } else {
                diagnostics_.stop_ack_error++;
            }
            setLastError(std::string("ACK de ") + command_name + " reportó error");
            return false;
        }

        const std::uint32_t remaining_ms = static_cast<std::uint32_t>(deadline - current_time_ms());
        const std::size_t available_now = serial_port_.available();
        const std::size_t to_read = available_now > 0 ? std::min<std::size_t>(available_now, 256) : 1;
        std::vector<std::uint8_t> temp(to_read);
        std::size_t received = 0;
        const result_t rc = serial_port_.readData(temp.data(), temp.size(), received, std::min<std::uint32_t>(remaining_ms, 50));

        if (rc == RESULT_FAIL) {
            setLastError(std::string("Fallo al leer ACK de ") + command_name);
            stream_buffer_.insert(stream_buffer_.end(), ack_buffer.begin(), ack_buffer.end());
            return false;
        }
        if (rc == RESULT_TIMEOUT || received == 0) {
            continue;
        }

        diagnostics_.bytes_ingested += received;
        if (received == 1) {
            diagnostics_.single_byte_reads++;
        } else {
            diagnostics_.bulk_reads++;
        }
        appendRecentBytes(temp.data(), received);
        dumpSerialBytes(temp.data(), received);
        ack_buffer.insert(ack_buffer.end(), temp.begin(), temp.begin() + static_cast<std::ptrdiff_t>(received));
    }

    diagnostics_.ack_timeouts++;
    stream_buffer_.insert(stream_buffer_.end(), ack_buffer.begin(), ack_buffer.end());
    setLastError(std::string("Timeout esperando ACK de ") + command_name);
    return false;
}

std::size_t LidarDataProcessing::findPattern(const std::vector<std::uint8_t>& haystack,
                                             const std::uint8_t* needle,
                                             std::size_t needle_size) {
    if (needle == nullptr || needle_size == 0 || haystack.size() < needle_size) {
        return std::string::npos;
    }

    auto it = std::search(haystack.begin(), haystack.end(), needle, needle + static_cast<std::ptrdiff_t>(needle_size));
    if (it == haystack.end()) {
        return std::string::npos;
    }
    return static_cast<std::size_t>(std::distance(haystack.begin(), it));
}

void LidarDataProcessing::appendRecentBytes(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
        return;
    }
    recent_serial_bytes_.insert(recent_serial_bytes_.end(), data, data + static_cast<std::ptrdiff_t>(size));
    if (recent_serial_bytes_.size() > RECENT_HEX_BYTES) {
        const auto extra = recent_serial_bytes_.size() - RECENT_HEX_BYTES;
        recent_serial_bytes_.erase(recent_serial_bytes_.begin(),
                                   recent_serial_bytes_.begin() + static_cast<std::ptrdiff_t>(extra));
    }
}

void LidarDataProcessing::dumpSerialBytes(const std::uint8_t* data, std::size_t size) {
    if (!serial_dump_.is_open() || data == nullptr || size == 0) {
        return;
    }
    if (serial_dump_written_ >= debug_options_.dump_serial_limit_bytes) {
        return;
    }
    const std::size_t remaining = debug_options_.dump_serial_limit_bytes - serial_dump_written_;
    const std::size_t to_write = std::min(remaining, size);
    serial_dump_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(to_write));
    serial_dump_written_ += to_write;
    serial_dump_.flush();
}

void LidarDataProcessing::setLastError(std::string message) {
    diagnostics_.last_error = std::move(message);
}

std::string LidarDataProcessing::bytesToHex(const std::vector<std::uint8_t>& bytes, std::size_t max_count) {
    if (bytes.empty()) {
        return {};
    }

    const std::size_t begin = bytes.size() > max_count ? bytes.size() - max_count : 0;
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (std::size_t i = begin; i < bytes.size(); ++i) {
        oss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i + 1 < bytes.size()) {
            oss << ' ';
        }
    }
    return oss.str();
}
