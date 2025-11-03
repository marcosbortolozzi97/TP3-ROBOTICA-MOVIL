# =====================================================
# 🔹 BASE: ROS 2 Jazzy + Python + herramientas comunes
# =====================================================
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# =====================================================
# 🔹 INSTALACIÓN DE DEPENDENCIAS DEL SISTEMA
# =====================================================
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-matplotlib \
    python3-pandas \
    ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*


# =====================================================
# 🔹 COPIAR EL WORKSPACE AL CONTENEDOR
# =====================================================
WORKDIR /ros2_ws
COPY . /ros2_ws

# =====================================================
# 🔹 INSTALAR DEPENDENCIAS PYTHON (aislado)
# =====================================================
RUN python3 -m venv /opt/venv \
    && . /opt/venv/bin/activate \
    && pip install --no-cache-dir -r requirements.txt

ENV PATH="/opt/venv/bin:$PATH"

# =====================================================
# 🔹 COMPILAR EL WORKSPACE DE ROS2
# =====================================================
# Usar el setup de ROS2 para que colcon funcione
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# =====================================================
# 🔹 CONFIGURACIÓN POR DEFECTO
# =====================================================
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

