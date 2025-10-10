# Imagen base con ROS2 (ejemplo: Humble en Ubuntu 22.04)
FROM ros:humble-ros-base

# Instalar dependencias del sistema
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Crear un workspace en ROS2
WORKDIR /root/ws
RUN mkdir -p src

# Copiar el código del proyecto al contenedor
COPY . /root/ws/src/tp3

# Compilar el workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Fuente del entorno por defecto
CMD ["/bin/bash"]
