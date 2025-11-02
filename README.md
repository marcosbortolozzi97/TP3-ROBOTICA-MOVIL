# TP3 – Robótica Móvil  
## Visión Estéreo y Reconstrucción 3D con ROS 2 y OpenCV

---

### **Descripción general**
Este proyecto implementa un pipeline completo de **visión estéreo y estimación 3D** utilizando **ROS 2 Jazzy** y **OpenCV**, incluyendo:

- Calibración estéreo.  
- Rectificación de imágenes.  
- Extracción y emparejamiento de *features*.  
- Triangulación y filtrado mediante RANSAC.  
- Cálculo de mapa de disparidad y reconstrucción densa.  
- Estimación de la pose entre cámaras (visión monocular).  
- Publicación de resultados en ROS 2 y visualización en RViz.

Todo el entorno está contenido dentro de un 'Contenedor Docker', lo que permite una ejecución reproducible y portable.

---

### **Requisitos previos**
Instalar **Docker** si no lo tiene instalado y/o verificar su funcionamiento:

```bash
sudo apt install docker docker.io
```
```bash
docker --version
```

---

### **Clonar el repositorio desde GitHub**
Desde tu home (o cualquier carpeta vacía):  
```bash
cd ~
```
```bash
git clone https://github.com/marcosbortolozzi97/TP3-ROBOTICA-MOVIL.git
```
```bash
cd TP3-ROBOTICA-MOVIL
```
Puede tardar algunos minutos. Luego, verificar la correcta clonación de los archivos en /TP3-ROBOTICA-MOVIL.  

---

### **Calibración Pre ejecución**
Antes de ejecutar el procesamiento estéreo, se realiza la calibración de las cámaras. Para ello, se utiliza el script **calibracion_camaras.py**, que aplica el método clásico de OpenCV.  
Las imágenes extraídas se encuentran en la carpeta calibración_images que abarca ambas cámaras. Es script adjunto toma como entrada las imágenes desde esa carpeta y genera un archivo **stereo_calibration.yml** como salida, el cual contiene los parámetros intrínsecos y extrínsecos utilizados en la ejecución.  

---

### **Guía para ejecución del TP**
1- Construir la imagen Docker  
Desde la carpeta raíz del repositorio (donde está el Dockerfile y el requirements.txt):  
```bash
docker build -t stereo_pointcloud_ros2 .
```
La correcta construcción de la imagen devuelve al final: **Successfully tagged stereo_pointcloud_ros2:latest**  

2- Permitir acceso gráfico  
Para que RViz (y otras interfaces visuales) funcionen correctamente dentro del contenedor Docker, es necesario habilitar el acceso del servidor gráfico X a Docker solo una vez (o cada vez que se reinicie el sistema):  
```bash
xhost +local:docker
```
Debería recibir un mensaje como: **non-network local connections being added to access control list**  
Una vez habilitado, ya se podrán ejecutar las visualizaciones sin necesidad de repetir este paso cada vez que se corra el contenedor.  
  
Cuando finalice la ejecución del programa o cuando desee puede desactivar este permiso mediante  
```bash
xhost -local:docker
```

3- Ejecutar el contenedor  
Ejecutar en una terminal el contenedor con acceso al entorno gráfico y montando la carpeta local del usuario (cambiar <usuario> por tu usuario):  
```bash
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/<usuario>/TP3-ROBOTICA-MOVIL:/ros2_ws \
    stereo_pointcloud_ros2
```
Una vez dentro del contenedor, te encontrarar con root@tu_device_name:/ros2_ws#  

4- Compilar el workspace dentro del contenedor  
Una vez dentro del contenedor:  
```bash
colcon build
```
```bash
source install/setup.bash
```

5- Ejecutar el programa principal  
Para ejecutar el procesamiento estéreo completo y la publicación de resultados en ROS 2:  
```bash
ros2 run stereo_pointcloud Tp3
```

6- Visualización en RViz  
Para visualizar los resultados en RViz, abrir otra terminal y ejecutar el contenedor con acceso gráfico:  
```bash
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/<usuario>/TP3-ROBOTICA-MOVIL:/ros2_ws \
    stereo_pointcloud_ros2
```
Una vez dentro del contenedor, ejecutar:  
```bash
rviz2
```
**Importante:** mientras se esté ejecutando el programa principal se van a cargar los distintos tópicos en rviz, por lo que para poder observarlos se deberá tener activo (tildeado) el tópico a visualizar antes de la ejecución. Una vez concluida la ejecución se pierde la visualizar al destildar. 
  
En el panel de visualización, en Global Options establecer el campo 'Fixed Frame' como **world**, agregar y configurar los siguientes tópicos:  
| Tipo          | Tópico                        | Estilo         | Descripción                 |
| ------------- | ----------------------------- | -------------- | --------------------------- |
| `PointCloud2` | `/stereo/pointcloud_densa`    | `Points`       | Nube densa                  |
| `PointCloud2` | `/stereo/pointcloud_filtrada` | `Points`       | Nube filtrada               |
| `PointCloud2` | `/stereo/pointcloud_original` | `Points`       | Nube original               |
| `Path`        | `/groundtruth_path`           | `Lines`        | Trayectoria real o estimada |
| `Pose`        | `/camera_left_pose`           | `Arrow`        | Visualización de las poses  |
| `Pose`        | `/camera_right_est_pose`      | `Arrow`        | Visualización de las poses  |

Si se pretende modificar el color de las distintas visualizaciones puede modificar en cada tópico el campo 'Color', y en particular en los tópicos PointCloud2 configurar el campo 'Color Transformer' a **FlatColor** y modificar el campo 'Color'.

7- Salir de Contenedor  
```bash
exit
```
Tener en cuenta que el contenedor se elimina automáticamente al salir, debido a que se lo está ejecutando con --rm.  

---

### **Nota:**
Si se desea ejecutar el script **calibracion_camaras.py** dentro del contenedor debe tener en cuenta los siguientes pasos:  
```bash
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/<usuario>/TP3-ROBOTICA-MOVIL:/ros2_ws \
    stereo_pointcloud_ros2
```
```bash
cd /ros2_ws
```
```bash
python3 calibracion_camaras.py

```
  
Por otro lado, en caso de fallar la ejecución por aceleración por hardware (si RViz se cierra o muestra una pantalla negra), se recomienda ejecutar RViz con el comando:  
```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```
esto fuerza que RViz use renderizado por CPU en lugar de GPU (software rendering), que funciona en cualquier entorno Docker sin aceleración 3D.  
