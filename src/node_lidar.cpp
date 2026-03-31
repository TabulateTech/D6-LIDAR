#include "node_lidar.h"

#include "mtime.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>

NodeLidar::NodeLidar(LidarGeneralInfo general_info, LidarRobotInfo robot_info, bool simulate, DebugOptions debug_options)
    : general_info_(std::move(general_info)),
      robot_info_(std::move(robot_info)),
      debug_options_(std::move(debug_options)) {
    status_.simulate = simulate;
    status_.filter_enable = general_info_.filter_enable;
    optimizer_ = std::make_unique<PointCloudOptimize>(robot_info_);
}

NodeLidar::~NodeLidar() {
    stop();
}

bool NodeLidar::initialize() {
    if (status_.simulate) {
        status_.connected = true;
        device_info_.model = "SIMULATED-COIN-D6";
        device_info_.revision = 1;
        return true;
    }

    serial_port_ = std::make_unique<SerialPort>(general_info_.port, general_info_.baudrate);
    if (!serial_port_->open()) {
        std::cerr << "No se pudo abrir el puerto serial: " << general_info_.port << '\n';
        return false;
    }

    processing_ = std::make_unique<LidarDataProcessing>(*serial_port_);
    processing_->setDebugOptions(debug_options_);
    status_.connected = true;

    DeviceInfo info;
    if (processing_->tryReadDeviceInfo(info, 250)) {
        device_info_ = info;
    } else {
        device_info_.model = "COIN-D6/UNKNOWN";
        device_info_.revision = 0;
    }

    return true;
}

bool NodeLidar::start() {
    if (!status_.connected || status_.running) {
        return false;
    }

    stop_requested_ = false;
    status_.running = true;

    if (!status_.simulate) {
        if (!processing_->sendStartCommand()) {
            status_.running = false;
            return false;
        }
        sleep_ms(100);
        worker_ = std::thread(&NodeLidar::acquisitionLoop, this);
    } else {
        worker_ = std::thread(&NodeLidar::simulatorLoop, this);
    }

    return true;
}

void NodeLidar::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
    }
    scan_cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    if (processing_) {
        processing_->sendStopCommand();
    }
    if (serial_port_) {
        serial_port_->close();
    }
    status_.running = false;
    status_.connected = false;
}

bool NodeLidar::waitForScan(LaserScan& scan, std::uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = scan_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
        return has_new_scan_ || stop_requested_;
    });
    if (!ready || !has_new_scan_) {
        return false;
    }
    scan = latest_scan_;
    has_new_scan_ = false;
    return true;
}

DeviceInfo NodeLidar::deviceInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return device_info_;
}

LidarDiagnosticsSnapshot NodeLidar::diagnostics() const {
    if (!processing_) {
        return {};
    }
    return processing_->diagnostics();
}

void NodeLidar::publishScan(const LaserScan& scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = scan;
    has_new_scan_ = true;
    scan_cv_.notify_one();
}

LaserScan NodeLidar::buildScanFromNodes(const std::vector<RawNode>& nodes, float scan_frequency_hz) const {
    LaserScan scan;
    scan.stamp_ns = current_time_ns();
    scan.scan_frequency_hz = scan_frequency_hz;
    scan.config.min_angle = 0.0f;
    scan.config.max_angle = static_cast<float>(2.0 * PI);
    scan.config.min_range = general_info_.range_min_m;
    scan.config.max_range = general_info_.range_max_m;
    scan.config.scan_time = scan_frequency_hz > 0.0f ? (1.0f / scan_frequency_hz) : 0.1f;
    scan.config.angle_increment = nodes.empty() ? 0.0f : static_cast<float>((2.0 * PI) / std::max<std::size_t>(1, nodes.size()));
    scan.config.time_increment = nodes.empty() ? 0.0f : scan.config.scan_time / std::max<std::size_t>(1, nodes.size());

    scan.points.reserve(nodes.size());
    for (const auto& node : nodes) {
        LaserPoint p;
        p.angle = node.angle_deg;
        p.range = (node.distance_m >= scan.config.min_range && node.distance_m <= scan.config.max_range) ? node.distance_m : 0.0f;
        p.intensity = node.intensity;
        p.high_reflection = node.high_reflection;

        const float corrected_angle = p.angle + robot_info_.install_to_zero;
        const float wrapped = std::fmod(corrected_angle + 360.0f, 360.0f);
        const float rad = wrapped * static_cast<float>(PI / 180.0);
        p.x = p.range * std::cos(rad);
        p.y = p.range * std::sin(rad);
        scan.points.push_back(p);
    }

    optimizer_->applyCoverCut(scan);
    if (status_.filter_enable) {
        optimizer_->pointCloudFilter(scan);
    }
    return scan;
}

void NodeLidar::printDiagnostics(std::size_t current_scan_points, bool packet_received) {
    if (!debug_options_.enabled || !processing_) {
        return;
    }

    const auto d = processing_->diagnostics();
    std::ostringstream oss;
    oss << "[DEBUG] bytes=" << d.bytes_ingested
        << " outer_ok=" << d.outer_frames_ok
        << " info=" << d.outer_info_frames
        << " data=" << d.outer_data_frames
        << " outer_bad_chk=" << d.outer_checksum_fail
        << " outer_bad_len=" << d.outer_invalid_length
        << " inner_ok=" << d.inner_packets_ok
        << " start=" << d.inner_start_packets
        << " data_pkt=" << d.inner_data_packets
        << " inner_bad_chk=" << d.inner_checksum_fail
        << " inner_bad_len=" << d.inner_invalid_length
        << " lsn0=" << d.zero_lsn_packets
        << " ring=" << d.ring_starts
        << " speed_bytes=" << d.speed_bytes_discarded
        << " stream_buf=" << d.stream_buffer_size
        << " raw_buf=" << d.raw_buffer_size
        << " current_scan=" << current_scan_points
        << " packet=" << (packet_received ? "yes" : "no");
    if (!d.last_error.empty()) {
        oss << " last_error='" << d.last_error << "'";
    }
    std::cout << oss.str() << std::endl;
    if (debug_options_.verbose && !d.recent_hex.empty()) {
        std::cout << "[DEBUG] recent_hex: " << d.recent_hex << std::endl;
    }
}

void NodeLidar::acquisitionLoop() {
    std::vector<RawNode> current_scan;
    current_scan.reserve(MAX_SCAN_NODES);
    float last_frequency_hz = 10.0f;
    std::uint64_t last_wait_log_ms = current_time_ms();
    std::uint64_t last_debug_log_ms = current_time_ms();
    bool announced_first_packet = false;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                break;
            }
        }

        std::vector<RawNode> packet;
        bool ring_start = false;
        float scan_frequency_hz = last_frequency_hz;
        const bool packet_ok = processing_->readPacket(packet, ring_start, scan_frequency_hz, 500);
        const std::uint64_t now_ms = current_time_ms();

        if (!packet_ok) {
            if (now_ms - last_wait_log_ms > 3000) {
                std::cout << "Esperando paquetes válidos del LiDAR en " << general_info_.port
                          << " a " << general_info_.baudrate << " bps..." << std::endl;
                last_wait_log_ms = now_ms;
            }
            if (debug_options_.enabled && now_ms - last_debug_log_ms >= debug_options_.print_every_ms) {
                printDiagnostics(current_scan.size(), false);
                last_debug_log_ms = now_ms;
            }
            continue;
        }

        if (!announced_first_packet) {
            std::cout << "Primer paquete válido recibido." << std::endl;
            announced_first_packet = true;
        }
        if (scan_frequency_hz > 0.0f) {
            last_frequency_hz = scan_frequency_hz;
        }

        bool inferred_ring_start = false;
        if (!ring_start && !packet.empty() && !current_scan.empty()) {
            const float previous_angle = current_scan.back().angle_deg;
            const float next_angle = packet.front().angle_deg;
            if (previous_angle > 315.0f && next_angle < 45.0f) {
                inferred_ring_start = true;
            }
        }

        if (debug_options_.enabled && now_ms - last_debug_log_ms >= debug_options_.print_every_ms) {
            printDiagnostics(current_scan.size(), true);
            last_debug_log_ms = now_ms;
        }

        if ((ring_start || inferred_ring_start) && !current_scan.empty()) {
            std::cout << "Vuelta completa: " << current_scan.size() << " puntos, "
                      << last_frequency_hz << " Hz"
                      << (inferred_ring_start && !ring_start ? " (por rollover angular)" : "")
                      << std::endl;
            publishScan(buildScanFromNodes(current_scan, last_frequency_hz));
            current_scan.clear();
        }

        current_scan.insert(current_scan.end(), packet.begin(), packet.end());
        if (current_scan.size() > MAX_SCAN_NODES) {
            current_scan.resize(MAX_SCAN_NODES);
        }
    }
}

void NodeLidar::simulatorLoop() {
    std::default_random_engine rng(static_cast<unsigned int>(current_time_ms()));
    std::normal_distribution<float> noise(0.0f, 0.01f);
    float phase = 0.0f;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                break;
            }
        }

        std::vector<RawNode> nodes;
        nodes.reserve(720);
        for (int i = 0; i < 720; ++i) {
            const float angle_deg = 360.0f * static_cast<float>(i) / 720.0f;
            const float rad = angle_deg * static_cast<float>(PI / 180.0);

            float range = 4.0f;
            const float wall_x = 3.0f;
            const float wall_y = 2.2f;
            const float dx = std::cos(rad);
            const float dy = std::sin(rad);

            if (std::fabs(dx) > 1e-3f) {
                const float t = wall_x / dx;
                if (t > 0.0f) {
                    const float y = t * dy;
                    if (std::fabs(y) < 2.0f) {
                        range = std::min(range, std::fabs(t));
                    }
                }
            }
            if (std::fabs(dy) > 1e-3f) {
                const float t = wall_y / dy;
                if (t > 0.0f) {
                    const float x = t * dx;
                    if (std::fabs(x) < 3.5f) {
                        range = std::min(range, std::fabs(t));
                    }
                }
            }

            const float cx = 1.2f + 0.35f * std::cos(phase);
            const float cy = -1.0f + 0.25f * std::sin(phase * 0.7f);
            const float radius = 0.45f;
            const float b = -2.0f * (cx * dx + cy * dy);
            const float c = cx * cx + cy * cy - radius * radius;
            const float disc = b * b - 4.0f * c;
            if (disc >= 0.0f) {
                const float t1 = (-b - std::sqrt(disc)) * 0.5f;
                if (t1 > 0.0f) {
                    range = std::min(range, t1);
                }
            }

            range += noise(rng);
            range = std::clamp(range, 0.15f, 9.5f);

            RawNode node;
            node.sync_flag = i == 0;
            node.angle_deg = angle_deg;
            node.distance_m = range;
            node.intensity = 100;
            node.high_reflection = false;
            node.stamp_ns = current_time_ns();
            node.scan_frequency_hz = 10.0f;
            nodes.push_back(node);
        }

        publishScan(buildScanFromNodes(nodes, 10.0f));
        phase += 0.12f;
        sleep_ms(100);
    }
}
