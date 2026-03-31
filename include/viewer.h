#ifndef VIEWER_H
#define VIEWER_H

#include "lidar_information.h"

#include <string>

class PointCloudViewer {
public:
    PointCloudViewer(int width, int height, float max_range_m, std::string output_dir);
    ~PointCloudViewer();

    bool createWindow(const std::string& title);
    void processEvents(bool& running);
    void render(const LaserScan& scan);
    void close();

private:
    void saveSvg(const LaserScan& scan) const;
    int toScreenX(float x_m) const;
    int toScreenY(float y_m) const;

    int width_ = 900;
    int height_ = 900;
    float max_range_m_ = 10.0f;
    std::string output_dir_;

#ifdef _WIN32
    void* hwnd_ = nullptr;
    void* hdc_ = nullptr;
#elif defined(LIDAR_ENABLE_X11)
    void* display_ = nullptr;
    unsigned long window_ = 0;
    void* gc_ = nullptr;
#endif
};

#endif
