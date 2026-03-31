#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H

#include "lidar_data_processing.h"
#include "point_cloud_optimize.h"
#include "serial_port.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class NodeLidar {
public:
    NodeLidar(LidarGeneralInfo general_info, LidarRobotInfo robot_info, bool simulate, DebugOptions debug_options = {});
    ~NodeLidar();

    bool initialize();
    bool start();
    void stop();

    bool waitForScan(LaserScan& scan, std::uint32_t timeout_ms);
    DeviceInfo deviceInfo() const;
    const LidarGeneralInfo& generalInfo() const { return general_info_; }
    LidarDiagnosticsSnapshot diagnostics() const;

private:
    void acquisitionLoop();
    void simulatorLoop();
    LaserScan buildScanFromNodes(const std::vector<RawNode>& nodes, float scan_frequency_hz) const;
    void publishScan(const LaserScan& scan);
    void printDiagnostics(std::size_t current_scan_points, bool packet_received);

    LidarGeneralInfo general_info_;
    LidarRobotInfo robot_info_;
    LidarStatus status_;
    DebugOptions debug_options_;
    mutable std::mutex mutex_;
    std::condition_variable scan_cv_;
    LaserScan latest_scan_;
    bool has_new_scan_ = false;
    bool stop_requested_ = false;
    std::thread worker_;
    DeviceInfo device_info_;

    std::unique_ptr<SerialPort> serial_port_;
    std::unique_ptr<LidarDataProcessing> processing_;
    std::unique_ptr<PointCloudOptimize> optimizer_;
};

#endif
