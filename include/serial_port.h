#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include "lidar_information.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class SerialPort {
public:
    SerialPort(std::string port, std::uint32_t baudrate);
    ~SerialPort();

    bool open();
    void close();
    bool isOpen() const;

    result_t writeData(const std::uint8_t* data, std::size_t size);
    result_t readData(std::uint8_t* data, std::size_t size, std::size_t& bytes_read, std::uint32_t timeout_ms);
    result_t readByte(std::uint8_t& byte, std::uint32_t timeout_ms);
    std::size_t available() const;

    const std::string& port() const { return port_; }
    std::uint32_t baudrate() const { return baudrate_; }

private:
#ifdef _WIN32
    void* handle_ = nullptr;
#else
    int fd_ = -1;
#endif
    std::string port_;
    std::uint32_t baudrate_ = 230400;
};

#endif
