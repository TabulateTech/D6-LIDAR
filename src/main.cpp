#include "node_lidar.h"
#include "viewer.h"

#include <atomic>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

std::atomic<bool> g_stop_requested{false};

extern "C" void handleSignal(int) {
    g_stop_requested.store(true);
}

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

std::filesystem::path normalizeOutputDir(const std::string& output_dir) {
    const std::filesystem::path path(output_dir);
    if (path.empty()) {
        return std::filesystem::path("output");
    }
    return path.lexically_normal();
}

bool isDangerousDirectoryToClear(const std::filesystem::path& dir) {
    if (dir.empty()) {
        return true;
    }

    const std::filesystem::path normalized = dir.lexically_normal();
    if (normalized == normalized.root_path()) {
        return true;
    }

    if (normalized == "." || normalized == "..") {
        return true;
    }

    std::error_code ec;
    const auto current = std::filesystem::current_path(ec);
    if (!ec && std::filesystem::equivalent(normalized, current, ec)) {
        return !ec;
    }

    return false;
}

bool prepareOutputDirectory(const std::filesystem::path& dir, std::string& error_message) {
    std::error_code ec;
    if (std::filesystem::exists(dir, ec) && !std::filesystem::is_directory(dir, ec)) {
        error_message = "La ruta de salida existe pero no es una carpeta: " + dir.string();
        return false;
    }

    if (!std::filesystem::exists(dir, ec)) {
        if (!std::filesystem::create_directories(dir, ec) && ec) {
            error_message = "No se pudo crear la carpeta de salida: " + dir.string();
            return false;
        }
        return true;
    }

    if (isDangerousDirectoryToClear(dir)) {
        error_message = "Se rechazó limpiar una carpeta potencialmente peligrosa: " + dir.string();
        return false;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) {
            error_message = "No se pudo recorrer la carpeta de salida: " + dir.string();
            return false;
        }
        std::filesystem::remove_all(entry.path(), ec);
        if (ec) {
            error_message = "No se pudo limpiar la carpeta de salida: " + dir.string();
            return false;
        }
    }

    return true;
}

std::filesystem::path resolveOutputFilePath(const std::filesystem::path& output_dir,
                                            const std::string& candidate_path) {
    if (candidate_path.empty()) {
        return {};
    }

    const std::filesystem::path path(candidate_path);
    if (path.is_absolute()) {
        return path;
    }
    return (output_dir / path).lexically_normal();
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
        << "  --max-range <m>            Distancia máxima válida del LiDAR.\n"
        << "  --zoom <factor>            Zoom visual independiente. 1.0=normal, 2.0=acerca, 0.5=aleja.\n"
        << "  --install-angle <deg>      Compensación angular de montaje en grados del marco del LiDAR.\n"
        << "  --no-filter                Desactiva el filtro de puntos.\n"
        << "  --save-csv                 Guarda la última vuelta en latest_scan.csv\n"
        << "  --output <carpeta>         Carpeta para SVG/CSV/dumps. Se limpia al iniciar.\n"
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
    float zoom_factor = 1.0f;
    std::string output_dir = "output";

#ifdef _WIN32
    general_info.port = "COM3";
#else
    general_info.port = "/dev/ttyUSB0";
#endif

    std::signal(SIGINT, handleSignal);
#ifdef SIGTERM
    std::signal(SIGTERM, handleSignal);
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
        } else if (argEquals(argv[i], "--zoom") && i + 1 < argc) {
            zoom_factor = std::stof(argv[++i]);
            if (zoom_factor <= 0.0f) {
                std::cerr << "El valor de --zoom debe ser mayor que 0.\n";
                return 1;
            }
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

    const std::filesystem::path output_dir_path = normalizeOutputDir(output_dir);
    std::string prepare_error;
    if (!prepareOutputDirectory(output_dir_path, prepare_error)) {
        std::cerr << prepare_error << "\n";
        return 1;
    }

    if (!debug_options.dump_serial_path.empty()) {
        const auto dump_path = resolveOutputFilePath(output_dir_path, debug_options.dump_serial_path);
        if (!dump_path.parent_path().empty()) {
            std::error_code dump_ec;
            std::filesystem::create_directories(dump_path.parent_path(), dump_ec);
            if (dump_ec) {
                std::cerr << "No se pudo crear la carpeta del volcado serial: " << dump_path.parent_path().string() << "\n";
                return 1;
            }
        }
        std::ofstream dump_probe(dump_path, std::ios::binary | std::ios::trunc);
        if (!dump_probe) {
            std::cerr << "No se pudo preparar el archivo de volcado serial: " << dump_path.string() << "\n";
            return 1;
        }
        debug_options.dump_serial_path = dump_path.string();
    }

    NodeLidar lidar(general_info, robot_info, simulate, debug_options);
    if (!lidar.initialize()) {
        std::cerr << "Fallo al inicializar el proyecto LiDAR.\n";
        return 1;
    }

    const DeviceInfo info = lidar.deviceInfo();
    std::cout << "Modelo: " << info.model << "  Rev: " << static_cast<int>(info.revision)
              << "  sentido=" << (info.clockwise ? "horario" : "antihorario")
              << "  correccion_host=" << (info.needs_angle_correction ? "si" : "no") << '\n';

    if (!lidar.start()) {
        std::cerr << "No se pudo iniciar el LiDAR.\n";
        return 1;
    }

    PointCloudViewer viewer(width, height, max_range_m, zoom_factor, output_dir_path.string());
    if (!viewer.createWindow("LiDAR point cloud viewer ")) {
        std::cout << "No se pudo abrir ventana gráfica. Se seguirá generando latest_scan.svg.\n";
    }

    if (debug_options.enabled && !debug_options.dump_serial_path.empty()) {
        std::cout << "Volcado serial activo en: " << debug_options.dump_serial_path
                  << " (límite " << debug_options.dump_serial_limit_bytes << " bytes)\n";
    }

    std::cout << "Proyecto iniciado. Presiona Ctrl+C para salir.\n";
    bool running = true;
    bool waiting_for_final_scan = false;
    while (running) {
        viewer.processEvents(running);

        if (g_stop_requested.load()) {
            if (!waiting_for_final_scan) {
                waiting_for_final_scan = true;
                std::cout << "Ctrl+C detectado. Se esperará la siguiente vuelta completa para guardar latest_scan y salir.\n";
            }
        }

        LaserScan scan;
        if (lidar.waitForScan(scan, 200)) {
            viewer.render(scan);
            if (save_csv) {
                saveCsv(scan, output_dir_path / "latest_scan.csv");
            }
            if (waiting_for_final_scan) {
                running = false;
            }
        }
    }

    lidar.stop();
    viewer.close();
    return 0;
}
