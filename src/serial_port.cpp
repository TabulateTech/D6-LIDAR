#include "serial_port.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

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
#else
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

SerialPort::SerialPort(std::string port, std::uint32_t baudrate)
    : port_(std::move(port)), baudrate_(baudrate) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open() {
#ifdef _WIN32
    if (isOpen()) {
        return true;
    }

    std::string full_port = port_;
    if (full_port.rfind("\\\\.\\", 0) != 0) {
        full_port = "\\\\.\\" + full_port;
    }

    HANDLE h = CreateFileA(full_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        handle_ = nullptr;
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    dcb.BaudRate = baudrate_;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &timeouts);

    SetupComm(h, 1 << 15, 1 << 15);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = h;
    return true;
#else
    if (isOpen()) {
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    speed_t speed = B230400;
    switch (baudrate_) {
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
#ifdef B460800
        case 460800: speed = B460800; break;
#endif
        default: speed = B230400; break;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
#endif
}

void SerialPort::close() {
#ifdef _WIN32
    if (handle_ != nullptr) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
#else
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

bool SerialPort::isOpen() const {
#ifdef _WIN32
    return handle_ != nullptr;
#else
    return fd_ >= 0;
#endif
}

result_t SerialPort::writeData(const std::uint8_t* data, std::size_t size) {
    if (!isOpen() || data == nullptr || size == 0) {
        return RESULT_FAIL;
    }
#ifdef _WIN32
    DWORD written = 0;
    if (!WriteFile(static_cast<HANDLE>(handle_), data, static_cast<DWORD>(size), &written, nullptr)) {
        return RESULT_FAIL;
    }
    FlushFileBuffers(static_cast<HANDLE>(handle_));
    return written == size ? RESULT_OK : RESULT_FAIL;
#else
    std::size_t offset = 0;
    while (offset < size) {
        const ssize_t n = ::write(fd_, data + offset, size - offset);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return RESULT_FAIL;
        }
        offset += static_cast<std::size_t>(n);
    }
    tcdrain(fd_);
    return RESULT_OK;
#endif
}

result_t SerialPort::readData(std::uint8_t* data, std::size_t size, std::size_t& bytes_read, std::uint32_t timeout_ms) {
    bytes_read = 0;
    if (!isOpen() || data == nullptr || size == 0) {
        return RESULT_FAIL;
    }

#ifdef _WIN32
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (bytes_read < size) {
        DWORD errors = 0;
        COMSTAT status{};
        ClearCommError(static_cast<HANDLE>(handle_), &errors, &status);
        if (status.cbInQue == 0) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return bytes_read > 0 ? RESULT_OK : RESULT_TIMEOUT;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        DWORD chunk = 0;
        const DWORD to_read = static_cast<DWORD>(std::min<std::size_t>(size - bytes_read, status.cbInQue));
        if (!ReadFile(static_cast<HANDLE>(handle_), data + bytes_read, to_read, &chunk, nullptr)) {
            return RESULT_FAIL;
        }
        bytes_read += static_cast<std::size_t>(chunk);
    }
    return RESULT_OK;
#else
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (bytes_read < size) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return bytes_read > 0 ? RESULT_OK : RESULT_TIMEOUT;
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);

        timeval tv{};
        tv.tv_sec = static_cast<long>(remaining / 1000);
        tv.tv_usec = static_cast<long>((remaining % 1000) * 1000);

        const int rc = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            return RESULT_FAIL;
        }
        if (rc == 0) {
            return bytes_read > 0 ? RESULT_OK : RESULT_TIMEOUT;
        }

        const ssize_t n = ::read(fd_, data + bytes_read, size - bytes_read);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            return RESULT_FAIL;
        }
        if (n == 0) {
            continue;
        }
        bytes_read += static_cast<std::size_t>(n);
    }
    return RESULT_OK;
#endif
}

result_t SerialPort::readByte(std::uint8_t& byte, std::uint32_t timeout_ms) {
    std::size_t bytes_read = 0;
    const result_t rc = readData(&byte, 1, bytes_read, timeout_ms);
    if (rc == RESULT_OK && bytes_read == 1) {
        return RESULT_OK;
    }
    return rc;
}

std::size_t SerialPort::available() const {
#ifdef _WIN32
    if (!isOpen()) {
        return 0;
    }
    DWORD errors = 0;
    COMSTAT status{};
    if (!ClearCommError(static_cast<HANDLE>(handle_), &errors, &status)) {
        return 0;
    }
    return static_cast<std::size_t>(status.cbInQue);
#else
    if (!isOpen()) {
        return 0;
    }
    int count = 0;
    if (ioctl(fd_, FIONREAD, &count) == -1) {
        return 0;
    }
    return static_cast<std::size_t>((std::max)(count, 0));
#endif
}
