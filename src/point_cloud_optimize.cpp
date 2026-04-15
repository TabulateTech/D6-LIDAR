#include "point_cloud_optimize.h"

#include <cmath>

namespace {

    float correctedAngleRad(float angle_deg, float install_to_zero_deg) {
        const float wrapped = std::fmod(angle_deg + install_to_zero_deg + 360.0f, 360.0f);
        return wrapped * static_cast<float>(PI / 180.0);
    }

} // namespace

PointCloudOptimize::PointCloudOptimize(const LidarRobotInfo& robot_info)
    : robot_info_(robot_info) {
}

void PointCloudOptimize::applyCoverCut(LaserScan& scan) const {
    if (!robot_info_.lidar_cover_enable || robot_info_.cover_angles.empty()) {
        return;
    }

    for (auto& point : scan.points) {
        for (const auto& cover : robot_info_.cover_angles) {
            if (cover.f_begin < point.angle && point.angle < cover.f_end) {
                point.range = 0.0f;
                point.x = 0.0f;
                point.y = 0.0f;
                break;
            }
        }
    }
}

void PointCloudOptimize::pointCloudFilter(LaserScan& scan) const {
    if (scan.points.size() < 5) {
        return;
    }

    const float filter_ratio = 2.5f;
    const float ang_diff = static_cast<float>(2.0 * PI / static_cast<double>(scan.points.size() - 1));
    const float filter_ratio_adj = filter_ratio * std::sin(ang_diff);

    auto dist_between = [](const LaserPoint& a, const LaserPoint& b) {
        const float ar = a.range;
        const float br = b.range;
        const float dtheta = std::fabs(a.angle - b.angle) * static_cast<float>(PI / 180.0);
        return std::sqrt(ar * ar + br * br - 2.0f * ar * br * std::cos(dtheta));
        };

    for (std::size_t i0 = 0; i0 < scan.points.size(); ++i0) {
        const std::size_t i1 = (i0 + 1) % scan.points.size();
        const std::size_t i2 = (i0 + 2) % scan.points.size();
        const std::size_t i3 = (i0 + 3) % scan.points.size();

        int depth_state = 0;
        depth_state += (scan.points[i0].range != 0.0f) * 1;
        depth_state += (scan.points[i1].range != 0.0f) * 2;
        depth_state += (scan.points[i2].range != 0.0f) * 4;
        depth_state += (scan.points[i3].range != 0.0f) * 8;

        if (depth_state == 0x0F) {
            const float rad0 = correctedAngleRad(scan.points[i0].angle, robot_info_.install_to_zero);
            const float rad3 = correctedAngleRad(scan.points[i3].angle, robot_info_.install_to_zero);

            const float x0 = scan.points[i0].range * std::cos(rad0);
            const float y0 = scan.points[i0].range * std::sin(rad0);
            const float x3 = scan.points[i3].range * std::cos(rad3);
            const float y3 = scan.points[i3].range * std::sin(rad3);
            const float x1 = (x3 + 2.0f * x0) / 3.0f;
            const float y1 = (y3 + 2.0f * y0) / 3.0f;
            const float x2 = (2.0f * x3 + x0) / 3.0f;
            const float y2 = (2.0f * y3 + y0) / 3.0f;
            const float r1 = std::sqrt(x1 * x1 + y1 * y1);
            const float r2 = std::sqrt(x2 * x2 + y2 * y2);

            const float d01 = dist_between(scan.points[i0], scan.points[i1]);
            const float d12 = dist_between(scan.points[i1], scan.points[i2]);
            const float d23 = dist_between(scan.points[i2], scan.points[i3]);

            if (filter_ratio_adj * scan.points[i1].range < d01 && filter_ratio_adj * scan.points[i1].range < d12) {
                scan.points[i1].range = 0.0f;
            }
            if (filter_ratio_adj * scan.points[i2].range < d12 && filter_ratio_adj * scan.points[i2].range < d23) {
                scan.points[i2].range = 0.0f;
            }
            if (scan.points[i1].range == 0.0f || scan.points[i2].range == 0.0f) {
                continue;
            }

            if ((scan.points[i1].range > r1 && scan.points[i2].range < r2) ||
                (scan.points[i1].range < r1 && scan.points[i2].range > r2)) {
                scan.points[i1].range = r1 + 0.4f * (scan.points[i1].range - r1);
                scan.points[i2].range = r2 + 0.4f * (scan.points[i2].range - r2);
            }
        }
        else if ((depth_state & 0x0E) == 0x04) {
            scan.points[i2].range = 0.0f;
        }
        else if ((depth_state & 0x07) == 0x02) {
            scan.points[i1].range = 0.0f;
        }
    }

    for (std::size_t i0 = 0; i0 < scan.points.size(); ++i0) {
        const std::size_t i1 = (i0 + 1) % scan.points.size();
        const std::size_t i2 = (i0 + 2) % scan.points.size();
        const std::size_t i3 = (i0 + 3) % scan.points.size();
        const std::size_t i4 = (i0 + 4) % scan.points.size();

        if (scan.points[i0].range != 0.0f && scan.points[i1].range != 0.0f &&
            scan.points[i2].range != 0.0f && scan.points[i3].range != 0.0f && scan.points[i4].range != 0.0f) {
            const float d01 = dist_between(scan.points[i0], scan.points[i1]);
            const float d12 = dist_between(scan.points[i1], scan.points[i2]);
            const float d23 = dist_between(scan.points[i2], scan.points[i3]);
            const float d34 = dist_between(scan.points[i3], scan.points[i4]);

            if (d01 < filter_ratio_adj * scan.points[i1].range &&
                d34 < filter_ratio_adj * scan.points[i3].range &&
                (d12 > filter_ratio_adj * scan.points[i2].range || d23 > filter_ratio_adj * scan.points[i2].range)) {
                const bool rising = scan.points[i0].range < scan.points[i1].range && scan.points[i3].range < scan.points[i4].range;
                const bool falling = scan.points[i0].range > scan.points[i1].range && scan.points[i3].range > scan.points[i4].range;
                if (rising || falling) {
                    scan.points[i2].range = (scan.points[i1].range + scan.points[i3].range) * 0.5f;
                }
            }
        }
    }

    for (auto& p : scan.points) {
        if (p.range <= 0.0f) {
            p.x = 0.0f;
            p.y = 0.0f;
            continue;
        }

        const float rad = correctedAngleRad(p.angle, robot_info_.install_to_zero);
        p.x = p.range * std::cos(rad);
        p.y = p.range * std::sin(rad);
    }
}
