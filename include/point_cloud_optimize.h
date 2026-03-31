#ifndef POINT_CLOUD_OPTIMIZE_H
#define POINT_CLOUD_OPTIMIZE_H

#include "lidar_information.h"

class PointCloudOptimize {
public:
    explicit PointCloudOptimize(const LidarRobotInfo& robot_info);

    void pointCloudFilter(LaserScan& scan) const;
    void applyCoverCut(LaserScan& scan) const;

private:
    const LidarRobotInfo& robot_info_;
};

#endif
