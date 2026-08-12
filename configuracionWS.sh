#!/bin/bash

# 1. Importar los parámetros del archivo .conf
echo "Llamando parametros de configuración..."
source ./configuracionWS.conf

# 2. Crear directorios de trabajo y código fuente
echo "Iniciando instalación en entorno de $ENTORNO..."
mkdir -p "$DIRECTORIO_CODIGO_FUENTE"

# 3. Configurar el entorno de trabajo
echo "Configurando entorno de trabajo..."
cd "$WS_TRABAJO"
colcon build

#Agregar al archivo .bashrc la configuración del work space de ROS 2 Jazzy para que se cargue automáticamente al iniciar la terminal
WS_SETUP_LINE="source $ARCHIVO_CONFIGURACION"

# Verificar si la línea ya existe en el .bashrc para no duplicarla
if ! grep -Fq "$WS_SETUP_LINE" ~/.bashrc; then
    echo "Agregando el work space de ROS 2 Jazzy a ~/.bashrc..."
    echo "$WS_SETUP_LINE" >> ~/.bashrc
    echo "¡Configuración agregada con éxito!"
else
    echo "El work space de ROS 2 Jazzy ya estaba configurado en ~/.bashrc."
fi

#Agregar al archivo .bashrc el autocompeltado de colcon para que se cargue automáticamente al iniciar la terminal
WS_SETUP_LINE="source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"

# Verificar si la línea ya existe en el .bashrc para no duplicarla
if ! grep -Fq "$WS_SETUP_LINE" ~/.bashrc; then
    echo "Agregando el autocompletado de colcon a ~/.bashrc..."
    echo "$WS_SETUP_LINE" >> ~/.bashrc
    echo "¡Configuración agregada con éxito!"
else
    echo "El autocompletado de colcon ya estaba configurado en ~/.bashrc."
fi

# Agregar alias para cargar cambios en el work space
ALIAS_LINE='alias source_ws="source ./install/setup.bash"'
# Verificar si la línea ya existe en el .bashrc para no duplicarla
if ! grep -Fq "$ALIAS_LINE" ~/.bashrc; then
    echo "Agregando el alias source_ws a ~/.bashrc..."
    echo "$ALIAS_LINE" >> ~/.bashrc
    echo "¡Alias agregado con éxito!"
else
    echo "El alias source_ws ya estaba configurado en ~/.bashrc."
fi

#agregar funcion para trabajar con el work space de ROS 2 Jazzy
# Nombre único de la función para verificar si ya fue agregada
FUNCTION_NAME="ros2_create_py_pkg()"

# Verificar si la función ya existe en el .bashrc
if ! grep -Fq "$FUNCTION_NAME" ~/.bashrc; then
    echo "Agregando la función ros2_create_py_pkg al archivo ~/.bashrc..."
    
    # Inyectar la función al final del archivo .bashrc de forma segura
    cat << 'EOF' >> ~/.bashrc

# Función rápida para crear paquetes de ROS 2 en Python
ros2_create_py_pkg() {
    if [ -z "$1" ]; then
        echo "Error: Debes proporcionar el nombre del paquete."
        echo "Uso: ros2_create_py_pkg nombre_de_tu_paquete"
        return 1
    fi
    ros2 pkg create "$1" --build-type ament_python --dependencies rclpy
}
EOF
    echo "¡Función ros2_create_py_pkg() agregada con éxito!"
else
    echo "La función ros2_create_py ya se encuentra configurada en ~/.bashrc."
fi

# Nombre único de la función para verificar si ya fue agregada
FUNCTION_NAME="ros2_create_cpp_pkg()"

# Verificar si la función ya existe en el .bashrc
if ! grep -Fq "$FUNCTION_NAME" ~/.bashrc; then
    echo "Agregando la función ros2_create_cpp_pkg al archivo ~/.bashrc..."
    
    # Inyectar la función al final del archivo .bashrc de forma segura
    cat << 'EOF' >> ~/.bashrc

# Función rápida para crear paquetes de ROS 2 en C++
ros2_create_cpp_pkg() {
    if [ -z "$1" ]; then
        echo "Error: Debes proporcionar el nombre del paquete."
        echo "Uso: ros2_create_cpp_pkg nombre_de_tu_paquete"
        return 1
    fi
    ros2 pkg create "$1" --build-type ament_cmake --dependencies rclcpp
}
EOF
    echo "¡Función ros2_create_cpp_pkg() agregada con éxito!"
else
    echo "La función ros2_create_cpp_pkg ya se encuentra configurada en ~/.bashrc."
fi

cd $DIRECTORIO_ORIGENDIRECTORIO_CODIGO_FUENTE
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master

cd $WS_TRABAJO

sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

colcon build

#Agregar al archivo .bashrc la configuración de  ROS 2 Jazzy para que se cargue automáticamente al iniciar la terminal
WS_SETUP_LINE="source /opt/ros/$ROS_DISTRO/setup.bash"

# Verificar si la línea ya existe en el .bashrc para no duplicarla
if ! grep -Fq "$WS_SETUP_LINE" ~/.bashrc; then
    echo "Agregando la configuración de ROS 2 a ~/.bashrc..."
    echo "$WS_SETUP_LINE" >> ~/.bashrc
    echo "¡Configuración agregada con éxito!"
else
    echo "La configuración de ROS 2 ya estaba configurada en ~/.bashrc."
fi

# Intentar recargar el bashrc en la sesión actual
source ~/.bashrc 2>/dev/null || true

cowsay -f tux "¡Configuración completada con éxito!" | lolcat

cd $DIRECTORIO_ORIGEN

# Aplica los cambios del alias inmediatamente en la terminal actual
exec bash