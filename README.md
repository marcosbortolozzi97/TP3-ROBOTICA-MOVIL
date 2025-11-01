# TP3 – Robótica Móvil  
## Visión Estéreo y Reconstrucción 3D con ROS 2 y OpenCV

---

### 🧩 **Descripción general**
Este proyecto implementa un pipeline completo de **visión estéreo y estimación 3D** utilizando **ROS 2 Jazzy** y **OpenCV**, incluyendo:

- Calibración estéreo.  
- Rectificación de imágenes.  
- Extracción y emparejamiento de *features*.  
- Triangulación y filtrado mediante RANSAC.  
- Cálculo de mapa de disparidad y reconstrucción densa.  
- Estimación de la pose entre cámaras (visión monocular).  
- Publicación de resultados en ROS 2 y visualización en **RViz 2**.

Todo el entorno está contenido dentro de un **Docker**, lo que permite una ejecución reproducible y portable.

---

### ⚙️ **Requisitos previos**
Instalar **Docker** y verificar su funcionamiento:

```bash
sudo apt install docker docker.io
docker --version

