# D6 LiDAR

Visualizador y parser en C++ para LiDAR tipo COIN-D6 sin usar ROS.  
Permite leer el sensor por puerto serial, procesar la nube de puntos y visualizarla en una ventana.

## Requisitos

### Windows
- Visual Studio 2022 con el complemento de **Desktop development with C++**
- CMake
- Puerto serial disponible (`COM3`, `COM4`, etc.)

### Linux
- CMake
- Compilador con soporte C++17
- Puerto serial disponible (`/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)

## Build en Windows (Visual Studio / CMake)

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

Ejecutar con hardware:

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400
```

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



## Opciones útiles

### Opciones de línea de comandos

  `--simulate`  
  Ejecuta el programa en modo simulación sin necesidad de un sensor LiDAR conectado.

   `--port`  
  Especifica el puerto serial.

   `--baudrate 230400`  
  Define la velocidad de comunicación serial.

   `--width 900`  
  Establece el ancho de la ventana.

   `--height 900`  
  Establece la altura de la ventana.

   `--max-range 10`  
  Define la distancia máxima, en metros, que se mostrará o procesará.

   `--install-angle 90`  
  Establece el ángulo de instalación del LiDAR en grados.

  `--no-filter`  
  Desactiva el filtrado de puntos, por lo que se usarán directamente los datos crudos del LiDAR.

   `--save-csv`  
  Habilita el guardado de los datos capturados del LiDAR en un archivo CSV.

   `--output output`  
  Define el nombre base o la ruta de los archivos de salida generados.

   `--debug`  
  Activa el modo de depuración para mostrar información adicional de diagnóstico.

   `--verbose`  
  Activa mensajes más detallados durante la ejecución del programa.

   `--debug-every-ms 2000`  
  Muestra información de depuración de forma periódica cada cierta cantidad de milisegundos.

   `--dump-serial raw_dump.bin`  
  Guarda los datos crudos de la comunicación serial en un archivo binario.

  `--dump-limit 262144`  
  Limita la cantidad máxima de bytes que se escribirán en el archivo de volcado serial.

## Salidas

   `output/latest_scan.svg`: último frame de la nube de puntos.
   `output/latest_scan.csv`: último frame exportado a CSV si activas `--save-csv`.

## Notas importantes

1. El parser implementado sigue la especificación COIN-D6 que describe:
   - comando de inicio `AA 55 F0 0F`
   - comando de paro `AA 55 F5 0A`
   - paquetes `55 AA`
   - muestras de 3 bytes por punto
   - ángulos `FSA/LSA`
   - checksum interno para paquetes de intensidad, agrupando la muestra como `s0` y `(s1,s2)` tal como hace el driver original

2. La parte Linux quedó validada en compilación. La rama Windows se dejó preparada con Win32/GDI y serial WinAPI, pero conviene probarla en la máquina final con el puerto real.

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

Si el LiDAR real sigue sin mostrar una nube completa, esta versión además intenta cerrar una vuelta cuando el ángulo salta de la zona alta (`>315°`) a la zona baja (`<45°`), aunque no haya llegado un paquete `T=1`.
