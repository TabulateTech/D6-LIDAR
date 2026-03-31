#ifndef MTIME_H
#define MTIME_H

#include <cstdint>
#include <string>

std::uint64_t current_time_ns();
std::uint64_t current_time_ms();
void sleep_ms(int ms);
std::string now_string();

#endif
