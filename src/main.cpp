#include "node_lidar.h"
#include "viewer.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

void saveCsv(const LaserScan& scan, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        return;
    }
    out << "angle_deg,range_m,intensity,x_m,y_m,high_reflection\n";
    for (const auto& p : scan.points) {
        out << p.angle << ',' << p.range << ',' << p.intensity << ','
            << p.x << ',' << p.y << ',' << (p.high_reflection ? 1 : 0) << '\n';
    }
}

bool argEquals(const char* a, const char* b) {
    return std::string(a) == b;
}

void printUsage(const char* exe) {
    std::cout
        << "Uso:\n"
        << "  " << exe << " --simulate\n"
        << "  " << exe << " --port /dev/ttyUSB0 --baudrate 230400\n\n"
        << "Opciones:\n"
        << "  --simulate                 Genera una nube sintética sin hardware.\n"
        << "  --port <ruta>              Puerto serial. Linux: /dev/ttyUSB0, Windows: COM3\n"
        << "  --baudrate <valor>         Baudrate del LiDAR.\n"
        << "  --width <pix>              Ancho de ventana.\n"
        << "  --height <pix>             Alto de ventana.\n"
        << "  --max-range <m>            Alcance visual máximo.\n"
        << "  --install-angle <deg>      Compensación angular de montaje.\n"
        << "  --no-filter                Desactiva el filtro de puntos.\n"
        << "  --save-csv                 Guarda la última vuelta en latest_scan.csv\n"
        << "  --output <carpeta>         Carpeta para SVG/CSV.\n"
        << "  --debug                    Imprime diagnósticos del parser y del puerto.\n"
        << "  --verbose                  Muestra también los últimos bytes recibidos en hex.\n"
        << "  --debug-every-ms <ms>      Periodo entre reportes de depuración.\n"
        << "  --dump-serial <archivo>    Guarda bytes crudos del puerto para analizarlos después.\n"
        << "  --dump-limit <bytes>       Límite del volcado serial.\n";
}

} // namespace

int main(int argc, char** argv) {
    LidarGeneralInfo general_info;
    LidarRobotInfo robot_info;
    DebugOptions debug_options;
    bool simulate = false;
    bool save_csv = false;
    int width = 900;
    int height = 900;
    float max_range_m = 10.0f;
    std::string output_dir = "output";

#ifdef _WIN32
    general_info.port = "COM3";
#else
    general_info.port = "/dev/ttyUSB0";
#endif

    for (int i = 1; i < argc; ++i) {
        if (argEquals(argv[i], "--simulate")) {
            simulate = true;
        } else if (argEquals(argv[i], "--port") && i + 1 < argc) {
            general_info.port = argv[++i];
        } else if (argEquals(argv[i], "--baudrate") && i + 1 < argc) {
            general_info.baudrate = static_cast<std::uint32_t>(std::stoul(argv[++i]));
        } else if (argEquals(argv[i], "--width") && i + 1 < argc) {
            width = std::stoi(argv[++i]);
        } else if (argEquals(argv[i], "--height") && i + 1 < argc) {
            height = std::stoi(argv[++i]);
        } else if (argEquals(argv[i], "--max-range") && i + 1 < argc) {
            max_range_m = std::stof(argv[++i]);
        } else if (argEquals(argv[i], "--install-angle") && i + 1 < argc) {
            robot_info.install_to_zero = std::stof(argv[++i]);
        } else if (argEquals(argv[i], "--no-filter")) {
            general_info.filter_enable = false;
        } else if (argEquals(argv[i], "--save-csv")) {
            save_csv = true;
        } else if (argEquals(argv[i], "--output") && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (argEquals(argv[i], "--debug")) {
            debug_options.enabled = true;
        } else if (argEquals(argv[i], "--verbose")) {
            debug_options.enabled = true;
            debug_options.verbose = true;
        } else if (argEquals(argv[i], "--debug-every-ms") && i + 1 < argc) {
            debug_options.enabled = true;
            debug_options.print_every_ms = static_cast<std::uint32_t>(std::stoul(argv[++i]));
        } else if (argEquals(argv[i], "--dump-serial") && i + 1 < argc) {
            debug_options.enabled = true;
            debug_options.dump_serial_path = argv[++i];
        } else if (argEquals(argv[i], "--dump-limit") && i + 1 < argc) {
            debug_options.enabled = true;
            debug_options.dump_serial_limit_bytes = static_cast<std::size_t>(std::stoull(argv[++i]));
        } else if (argEquals(argv[i], "--help") || argEquals(argv[i], "-h")) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Argumento no reconocido: " << argv[i] << "\n\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    std::filesystem::create_directories(output_dir);

    NodeLidar lidar(general_info, robot_info, simulate, debug_options);
    if (!lidar.initialize()) {
        std::cerr << "Fallo al inicializar el proyecto LiDAR.\n";
        return 1;
    }

    const DeviceInfo info = lidar.deviceInfo();
    std::cout << "Modelo: " << info.model << "  Rev: " << static_cast<int>(info.revision) << '\n';

    if (!lidar.start()) {
        std::cerr << "No se pudo iniciar el LiDAR.\n";
        return 1;
    }

    PointCloudViewer viewer(width, height, max_range_m, output_dir);
    if (!viewer.createWindow("LiDAR point cloud viewer ")) {
        std::cout << "No se pudo abrir ventana gráfica. Se seguirá generando latest_scan.svg.\n";
    }

    if (debug_options.enabled && !debug_options.dump_serial_path.empty()) {
        std::cout << "Volcado serial activo en: " << debug_options.dump_serial_path
                  << " (límite " << debug_options.dump_serial_limit_bytes << " bytes)\n";
    }

    std::cout << "Proyecto iniciado. Presiona Ctrl+C para salir.\n";
    bool running = true;
    while (running) {
        viewer.processEvents(running);
        LaserScan scan;
        if (lidar.waitForScan(scan, 200)) {
            viewer.render(scan);
            if (save_csv) {
                saveCsv(scan, std::filesystem::path(output_dir) / "latest_scan.csv");
            }
        }
    }

    lidar.stop();
    viewer.close();
    return 0;
}
