cmake_minimum_required(VERSION 3.16)
project(D6_LIDAR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

set(SOURCES
    src/main.cpp
    src/mtime.cpp
    src/serial_port.cpp
    src/lidar_data_processing.cpp
    src/point_cloud_optimize.cpp
    src/node_lidar.cpp
    src/viewer.cpp
)

add_executable(D6_LIDAR ${SOURCES})

target_include_directories(D6_LIDAR PRIVATE include)

if(WIN32)
    target_link_libraries(D6_LIDAR PRIVATE user32 gdi32)
elseif(UNIX)
    find_package(X11)
    if(X11_FOUND)
        target_compile_definitions(D6_LIDAR PRIVATE LIDAR_ENABLE_X11)
        target_include_directories(D6_LIDAR PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(D6_LIDAR PRIVATE ${X11_LIBRARIES})
    endif()
endif()
