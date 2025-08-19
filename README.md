# Repositorio AutoModelCrofi ROS2

Este repositorio contiene el workspace de ROS2 para el proyecto AutoModelCrofi, que integra módulos de hardware, visión, control y comunicación para un vehículo autónomo basado en ROS2 y Arduino.

## Estructura del repositorio

- **src/**: Código fuente principal, dividido en submódulos:
  - `Arduino/`: Código para microcontroladores Arduino.
  - `hardware/`: Drivers y nodos relacionados con hardware (ej. LIDAR, motores).
  - `vision/`: Procesamiento de imágenes y visión computacional.
  - `control/`: Algoritmos de control y lógica de movimiento.
  - `test/`: Scripts y nodos para pruebas.

- **launch/**: Archivos de lanzamiento (`.py`) para iniciar múltiples nodos ROS2.

- **build/**, **install/**, **log/**: Directorios generados automáticamente por colcon y ROS2 para la construcción, instalación y registro de logs.

- **bin/**: Herramientas auxiliares (ej. `arduino-cli`).

- **LICENSE.txt**: Licencia del proyecto.

## Requisitos

- ROS2 Humble
- Python 3.10+
- Arduino IDE o `arduino-cli`
- Dependencias listadas en los archivos `package.xml` y `setup.py` de cada paquete

## Instalación

1. Clona el repositorio:
   ```sh
   git clone <URL-del-repositorio>
   cd ros2_ws
   ```

2. Instala las dependencias de ROS2:
   ```sh
   sudo apt update
   sudo apt install python3-colcon-common-extensions
   ```

3. Compila el workspace:
   ```sh
   colcon build
   ```

4. Fuente el entorno:
   ```sh
   source install/setup.bash
   ```

## Uso

Ejecuta los archivos de lanzamiento desde el directorio raíz:

```sh
ros2 launch launch/AutoModel.py
```

O lanza nodos individuales según sea necesario.

## Contribución

Las contribuciones son bienvenidas. Por favor, abre un issue o un pull request para sugerencias, correcciones o nuevas funcionalidades.

## Licencia

Este proyecto está bajo la licencia GPLv3. Consulta [LICENSE.txt](LICENSE.txt) para más detalles.

## Contacto

Para dudas o soporte, contacta a Tony <support@ydlidar.com> o abre un issue en este repositorio.
