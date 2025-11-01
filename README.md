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
Instalar **Docker** y verificar su funcionamiento:

```bash
sudo apt install docker docker.io
docker --version
```

### **Guía para ejecución del TP**
1- Construir la imagen Docker  
Desde la carpeta raíz del repositorio (donde está el Dockerfile y el requirements.txt):  
```bash
docker build -t stereo_pointcloud_ros2 .
```

2- Permitir acceso gráfico  
Para que RViz 2 funcione dentro del contenedor y pueda mostrar las ventanas gráficas:  
```bash
xhost +local:docker
```

3- Ejecutar el contenedor  
Ejecutar el contenedor con acceso al entorno gráfico y montando la carpeta local del usuario:  
```bash
docker run -it --rm \ --net=host \ -e DISPLAY=$DISPLAY \ -v /tmp/.X11-unix:/tmp/.X11-unix \ -v /home/marcos:/home/marcos \ stereo_pointcloud_ros2
```
