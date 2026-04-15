# D6 LiDAR

![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![CMake](https://img.shields.io/badge/CMake-Build-green)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-lightgrey)
![Status](https://img.shields.io/badge/Status-Experimental-orange)

Visualizador y parser en **C++** para LiDAR tipo **COIN-D6** sin usar ROS.  
El proyecto permite leer el sensor por **puerto serial**, decodificar los paquetes, procesar la nube de puntos y visualizarla en una ventana de forma local.

> [!IMPORTANT]
> Este proyecto está orientado a pruebas directas con el sensor LiDAR COIN-D6 y a la inspección del flujo de datos serial, sin depender de ROS ni de middleware adicional.

---

## Contenido

- [Descripción general](#descripción-general)
- [Características principales](#características-principales)
- [Requisitos](#requisitos)
- [Compilación](#compilación)
  - [Windows](#windows)
  - [Linux](#linux)
- [Uso rápido](#uso-rápido)
- [Opciones de línea de comandos](#opciones-de-línea-de-comandos)
- [Archivos de salida](#archivos-de-salida)
- [Depuración de protocolo](#depuración-de-protocolo)
- [Notas técnicas](#notas-técnicas)

---

## Descripción general

`D6_LIDAR` es una aplicación de escritorio para adquisición y visualización de datos LiDAR.  
Su propósito es facilitar:

- la conexión directa al sensor por puerto serial,
- el análisis de tramas y paquetes del protocolo,
- la visualización local de la nube de puntos,
- la exportación de resultados para inspección posterior.

> [!NOTE]
> El proyecto separa la **distancia máxima procesada** del **zoom visual**, de modo que el usuario puede acercar o alejar la vista sin perder alcance real de medición.

---

## Características principales

- Lectura directa del LiDAR mediante puerto serial.
- Modo de simulación para pruebas sin hardware.
- Visualización de nube de puntos en ventana local.
- Exportación de la última captura en formato **SVG**.
- Exportación opcional a **CSV**.
- Registro binario de datos seriales crudos para depuración.
- Parámetro de **zoom visual** independiente del rango máximo.
- Guardado del último escaneo asociado a una vuelta completa del sensor.

---

## Requisitos

### Windows

- Visual Studio 2022 con el componente **Desktop development with C++**
- CMake
- Puerto serial disponible, por ejemplo: `COM3`, `COM4`, etc.

### Linux

- CMake
- Compilador con soporte para **C++17**
- Puerto serial disponible, por ejemplo: `/dev/ttyUSB0`, `/dev/ttyACM0`, etc.

---

## Compilación

### Windows

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Linux

```bash
mkdir build
cd build
cmake ..
make -j
```


## Uso rápido

### Ejecutar en Windows

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400
```



### Ejecutar en Linux

```bash
./D6_LIDAR --port /dev/ttyUSB0 --baudrate 230400
```

### Ejemplo de ejecución con salida y depuración

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400 --save-csv --debug --zoom 6.0 --dump-serial raw_dump.bin --output output
```


## Opciones de línea de comandos

### Simulación y conexión

- `--simulate`  
  Ejecuta el programa en modo simulación, sin necesidad de conectar un sensor LiDAR real.

- `--port`  
  Especifica el puerto serial utilizado para la comunicación con el sensor.

- `--baudrate 230400`  
  Define la velocidad de comunicación serial.

### Visualización y procesamiento

- `--width 900`  
  Establece el ancho de la ventana de visualización.

- `--height 900`  
  Establece la altura de la ventana de visualización.

- `--max-range 10`  
  Define la distancia máxima, en metros, que se procesará o mostrará.

- `--zoom 1.0`  
  Ajusta el zoom visual de la ventana sin modificar la distancia máxima de medición.  
  Un valor mayor que `1.0` acerca la vista; un valor menor que `1.0` la aleja.

- `--install-angle 90`  
  Establece el ángulo de instalación del LiDAR en grados.

- `--no-filter`  
  Desactiva el filtrado de puntos y utiliza directamente los datos crudos del sensor.

### Exportación y salida de archivos

- `--save-csv`  
  Habilita la exportación de la última captura a un archivo CSV.

- `--output output`  
  Define la carpeta de salida donde se guardarán los archivos generados por el programa.

### Depuración

- `--debug`  
  Activa el modo de depuración para mostrar información adicional de diagnóstico.

- `--verbose`  
  Habilita mensajes más detallados durante la ejecución.

- `--debug-every-ms 2000`  
  Muestra información de depuración de forma periódica cada cierta cantidad de milisegundos.

- `--dump-serial raw_dump.bin`  
  Guarda los datos crudos del puerto serial en un archivo binario.

- `--dump-limit 262144`  
  Limita la cantidad máxima de bytes escritos en el archivo de volcado serial.

> [!WARNING]
> `--max-range` y `--zoom` no cumplen la misma función.  
> `--max-range` controla el rango máximo procesado; `--zoom` solo modifica la escala visual de la vista.

---

## Archivos de salida

Cuando se utiliza `--output output`, los archivos generados se almacenan en `output` la carpeta `build`.

### Archivos principales

- `output/latest_scan.svg`  
  Último escaneo exportado como imagen vectorial.

- `output/latest_scan.csv`  
  Último escaneo exportado en formato CSV, si se activa `--save-csv`.

- `output/raw_dump.bin`  
  Volcado binario de los datos seriales crudos, si se activa `--dump-serial`.

> [!NOTE]
> El archivo `latest_scan` se actualiza a partir de una vuelta completa del LiDAR, con el objetivo de evitar capturas parciales al cerrar el programa.

---

## Depuración de protocolo

Para diagnosticar problemas de comunicación o de ensamblado de paquetes, se recomienda usar una ejecución con depuración ampliada:

```powershell
.\Release\D6_LIDAR.exe --port COM3 --baudrate 230400 --debug --verbose --dump-serial raw_dump.bin --output output
```

Durante la ejecución, el programa puede mostrar información como:

- bytes recibidos por el puerto serial,
- frames externos válidos e inválidos,
- paquetes internos válidos e inválidos,
- detección de inicios de anillo,
- tamaño actual de los buffers,
- último error detectado por el parser,
- últimos bytes recibidos en hexadecimal.

> [!TIP]
> El archivo `raw_dump.bin` permite revisar posteriormente el tráfico serial real del sensor, lo cual resulta útil cuando el LiDAR no completa vueltas o cuando la nube de puntos aparece incompleta.

---

## Notas técnicas

1. El parser implementado sigue la especificación del protocolo **COIN-D6**, incluyendo:
   - comando de inicio `AA 55 F0 0F`,
   - comando de paro `AA 55 F5 0A`,
   - paquetes con cabecera `55 AA`,
   - muestras de 3 bytes por punto,
   - interpretación de ángulos `FSA/LSA`,
   - checksum interno para paquetes de intensidad, agrupando la muestra como `s0` y `(s1,s2)` de forma compatible con el driver original.

2. La rama Linux quedó validada a nivel de compilación.

3. La rama Windows utiliza **Win32/GDI** para la visualización y **WinAPI** para la comunicación serial, por lo que se recomienda validar el comportamiento final directamente en la máquina donde se conectará el sensor real.

> [!CAUTION]
> Si el LiDAR real no muestra una nube completa, no asumas de inmediato un fallo en la visualización.  
> Primero verifica:
> - el puerto serial correcto,
> - la velocidad configurada,
> - la presencia de datos reales en `raw_dump.bin`,
> - y los mensajes del parser en modo `--debug`.

---


