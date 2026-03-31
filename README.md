# D6 LiDAR

Visualizador y parser en C++ para LiDAR tipo COIN-D6 sin usar ROS.  
Permite leer el sensor por puerto serial, procesar la nube de puntos y visualizarla en una ventana

## Requisitos

### Linux
- CMake
- Compilador con soporte C++17
- Puerto serial disponible (`/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)

### Windows
- Visual Studio 2022 con **Desktop development with C++**
- CMake
- Puerto serial disponible (`COM3`, `COM4`, etc.)

## Build en Linux

```bash
mkdir build
cd build
cmake ..
make -j
```

Ejecutar con simulación:

```bash
./D6_LIDAR --simulate
```

Ejecutar con hardware:

```bash
./D6_LIDAR --port /dev/ttyUSB0 --baudrate 230400
```

## Build en Windows (Visual Studio / CMake)

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

Ejecutar, por ejemplo:

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400
```

## Opciones útiles

```bash
--simulate
--port /dev/ttyUSB0
--baudrate 230400
--width 900
--height 900
--max-range 10
--install-angle 90
--no-filter
--save-csv
--output output
--debug
--verbose
--debug-every-ms 2000
--dump-serial raw_dump.bin
--dump-limit 262144
```

## Salidas

- `output/latest_scan.svg` → último frame de la nube de puntos
- `output/latest_scan.csv` → último frame exportado a CSV si activas `--save-csv`

## Notas importantes

1. El parser implementado sigue la especificación COIN-D6 que describe:
   - comando de inicio `AA 55 F0 0F`
   - comando de paro `AA 55 F5 0A`
   - paquetes `55 AA`
   - muestras de 3 bytes por punto
   - ángulos `FSA/LSA`
   - checksum interno para paquetes de intensidad, agrupando la muestra como `s0` y `(s1,s2)` tal como hace el driver original

2. La parte Linux quedó validada en compilación. La rama Windows se dejó preparada con Win32/GDI y serial WinAPI, pero conviene probarla ya en la máquina final con el puerto real.


## Depuración de protocolo 

Para diagnosticar por qué el LiDAR real no completa una vuelta, usa:

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400 --debug --verbose --dump-serial raw_dump.bin
```

Eso mostrará por consola:

- bytes recibidos
- frames externos válidos e inválidos
- paquetes internos válidos e inválidos
- cuántos paquetes de inicio de anillo detectó
- tamaño actual de los buffers
- último error detectado por el parser
- últimos bytes recibidos en hexadecimal

El archivo `raw_dump.bin` guarda los primeros bytes crudos del puerto para poder analizarlos después.


## Depuración de protocolo 

Si el LiDAR real sigue sin mostrar una nube completa, prueba primero la versión:

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400 --debug --verbose --dump-serial raw_dump.bin
```

Esta versión además intenta cerrar una vuelta cuando el ángulo salta de la zona alta (`>315°`) a la zona baja (`<45°`), aunque no haya llegado un paquete `T=1`.
