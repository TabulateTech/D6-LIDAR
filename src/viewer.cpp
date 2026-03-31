#include "viewer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#elif defined(LIDAR_ENABLE_X11)
#include <X11/Xlib.h>
#endif

PointCloudViewer::PointCloudViewer(int width, int height, float max_range_m, std::string output_dir)
    : width_(width), height_(height), max_range_m_(max_range_m), output_dir_(std::move(output_dir)) {
    std::filesystem::create_directories(output_dir_);
}

PointCloudViewer::~PointCloudViewer() {
    close();
}

bool PointCloudViewer::createWindow(const std::string& title) {
#ifdef _WIN32
    WNDCLASSA wc{};
    wc.lpfnWndProc = DefWindowProcA;
    wc.hInstance = GetModuleHandleA(nullptr);
    wc.lpszClassName = "LidarPointCloudWindow";
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowA(wc.lpszClassName, title.c_str(), WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                              CW_USEDEFAULT, CW_USEDEFAULT, width_, height_, nullptr, nullptr,
                              wc.hInstance, nullptr);
    if (hwnd == nullptr) {
        return false;
    }
    hwnd_ = hwnd;
    hdc_ = GetDC(hwnd);
    return hdc_ != nullptr;
#elif defined(LIDAR_ENABLE_X11)
    Display* display = XOpenDisplay(nullptr);
    if (!display) {
        return false;
    }
    const int screen = DefaultScreen(display);
    const Window window = XCreateSimpleWindow(display, RootWindow(display, screen), 10, 10,
                                              static_cast<unsigned int>(width_), static_cast<unsigned int>(height_),
                                              1, BlackPixel(display, screen), WhitePixel(display, screen));
    XStoreName(display, window, title.c_str());
    XSelectInput(display, window, ExposureMask | KeyPressMask | StructureNotifyMask);
    XMapWindow(display, window);
    GC gc = XCreateGC(display, window, 0, nullptr);
    display_ = display;
    window_ = static_cast<unsigned long>(window);
    gc_ = gc;
    return true;
#else
    (void)title;
    return true;
#endif
}

void PointCloudViewer::processEvents(bool& running) {
#ifdef _WIN32
    MSG msg{};
    while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) {
            running = false;
        }
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
#elif defined(LIDAR_ENABLE_X11)
    auto* display = static_cast<Display*>(display_);
    if (!display) {
        return;
    }
    while (XPending(display) > 0) {
        XEvent event{};
        XNextEvent(display, &event);
        if (event.type == DestroyNotify) {
            running = false;
        }
    }
#else
    (void)running;
#endif
}

void PointCloudViewer::render(const LaserScan& scan) {
    saveSvg(scan);

#ifdef _WIN32
    if (hwnd_ == nullptr || hdc_ == nullptr) {
        return;
    }
    HDC hdc = static_cast<HDC>(hdc_);
    RECT rect{0, 0, width_, height_};
    FillRect(hdc, &rect, static_cast<HBRUSH>(GetStockObject(WHITE_BRUSH)));

    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(220, 220, 220));
    HPEN axis_pen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    HPEN point_pen = CreatePen(PS_SOLID, 1, RGB(30, 90, 200));

    SelectObject(hdc, grid_pen);
    for (int i = 1; i <= 4; ++i) {
        const int r = static_cast<int>(((std::min)(width_, height_) * 0.45) * (static_cast<float>(i) / 4.0f));
        Arc(hdc, width_ / 2 - r, height_ / 2 - r, width_ / 2 + r, height_ / 2 + r, 0, 0, 0, 0);
    }

    SelectObject(hdc, axis_pen);
    MoveToEx(hdc, 0, height_ / 2, nullptr);
    LineTo(hdc, width_, height_ / 2);
    MoveToEx(hdc, width_ / 2, 0, nullptr);
    LineTo(hdc, width_ / 2, height_);

    SelectObject(hdc, point_pen);
    for (const auto& point : scan.points) {
        if (point.range <= 0.0f || point.range > max_range_m_) {
            continue;
        }
        const int x = toScreenX(point.x);
        const int y = toScreenY(point.y);
        SetPixel(hdc, x, y, RGB(30, 90, 200));
    }

    DeleteObject(grid_pen);
    DeleteObject(axis_pen);
    DeleteObject(point_pen);
#elif defined(LIDAR_ENABLE_X11)
    auto* display = static_cast<Display*>(display_);
    const Window window = static_cast<Window>(window_);
    auto gc = static_cast<GC>(gc_);
    if (!display || window == 0 || gc == nullptr) {
        return;
    }

    XSetForeground(display, gc, WhitePixel(display, DefaultScreen(display)));
    XFillRectangle(display, window, gc, 0, 0, static_cast<unsigned int>(width_), static_cast<unsigned int>(height_));

    XSetForeground(display, gc, 0xDDDDDD);
    for (int i = 1; i <= 4; ++i) {
        const int r = static_cast<int>(((std::min)(width_, height_) * 0.45) * (static_cast<float>(i) / 4.0f));
        XDrawArc(display, window, gc, width_ / 2 - r, height_ / 2 - r, 2 * r, 2 * r, 0, 360 * 64);
    }

    XSetForeground(display, gc, BlackPixel(display, DefaultScreen(display)));
    XDrawLine(display, window, gc, 0, height_ / 2, width_, height_ / 2);
    XDrawLine(display, window, gc, width_ / 2, 0, width_ / 2, height_);

    XSetForeground(display, gc, 0x1E5AC8);
    for (const auto& point : scan.points) {
        if (point.range <= 0.0f || point.range > max_range_m_) {
            continue;
        }
        XDrawPoint(display, window, gc, toScreenX(point.x), toScreenY(point.y));
    }
    XFlush(display);
#endif
}

void PointCloudViewer::close() {
#ifdef _WIN32
    if (hdc_ != nullptr && hwnd_ != nullptr) {
        ReleaseDC(static_cast<HWND>(hwnd_), static_cast<HDC>(hdc_));
        hdc_ = nullptr;
    }
    if (hwnd_ != nullptr) {
        DestroyWindow(static_cast<HWND>(hwnd_));
        hwnd_ = nullptr;
    }
#elif defined(LIDAR_ENABLE_X11)
    auto* display = static_cast<Display*>(display_);
    const Window window = static_cast<Window>(window_);
    auto gc = static_cast<GC>(gc_);
    if (display) {
        if (gc) {
            XFreeGC(display, gc);
            gc_ = nullptr;
        }
        if (window != 0) {
            XDestroyWindow(display, window);
            window_ = 0;
        }
        XCloseDisplay(display);
        display_ = nullptr;
    }
#endif
}

void PointCloudViewer::saveSvg(const LaserScan& scan) const {
    std::filesystem::create_directories(output_dir_);
    const std::filesystem::path path = std::filesystem::path(output_dir_) / "latest_scan.svg";
    std::ofstream out(path);
    if (!out) {
        return;
    }

    out << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width_ << "' height='" << height_ << "' viewBox='0 0 "
        << width_ << ' ' << height_ << "'>\n";
    out << "<rect width='100%' height='100%' fill='white'/>\n";
    out << "<line x1='0' y1='" << height_ / 2 << "' x2='" << width_ << "' y2='" << height_ / 2
        << "' stroke='black' stroke-width='1'/>\n";
    out << "<line x1='" << width_ / 2 << "' y1='0' x2='" << width_ / 2 << "' y2='" << height_
        << "' stroke='black' stroke-width='1'/>\n";

    for (int i = 1; i <= 4; ++i) {
        const float r = ((std::min)(width_, height_) * 0.45f) * (static_cast<float>(i) / 4.0f);
        out << "<circle cx='" << width_ / 2 << "' cy='" << height_ / 2 << "' r='" << r
            << "' fill='none' stroke='#dddddd' stroke-width='1'/>\n";
    }

    for (const auto& point : scan.points) {
        if (point.range <= 0.0f || point.range > max_range_m_) {
            continue;
        }
        out << "<circle cx='" << toScreenX(point.x) << "' cy='" << toScreenY(point.y)
            << "' r='1.6' fill='#1e5ac8'/>\n";
    }

    std::ostringstream label;
    label << "frecuencia=" << scan.scan_frequency_hz << " Hz, puntos=" << scan.points.size();
    out << "<text x='10' y='24' fill='black' font-family='monospace' font-size='18'>" << label.str() << "</text>\n";
    out << "</svg>\n";
}

int PointCloudViewer::toScreenX(float x_m) const {
    const float normalized = x_m / max_range_m_;
    return static_cast<int>(width_ * 0.5f + normalized * (width_ * 0.45f));
}

int PointCloudViewer::toScreenY(float y_m) const {
    const float normalized = y_m / max_range_m_;
    return static_cast<int>(height_ * 0.5f - normalized * (height_ * 0.45f));
}
