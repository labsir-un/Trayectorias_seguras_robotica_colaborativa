#!/bin/bash
set -e

echo "Llamando parametros de configuración..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/configuracionWS.conf"

echo "=== Preparando Workspace en: $WS_TRABAJO ==="
mkdir -p "$DIRECTORIO_CODIGO_FUENTE"

# Clonar dependencias de código fuente
if [ ! -d "$DIRECTORIO_CODIGO_FUENTE/realsense-ros" ]; then
    echo "Clonando realsense-ros..."
    git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master "$DIRECTORIO_CODIGO_FUENTE/realsense-ros"
else
    echo "realsense-ros ya está clonado."
fi

# Inicializar y resolver dependencias rosdep
cd "$WS_TRABAJO"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Inicializando rosdep..."
    sudo rosdep init
fi
echo "Actualizando rosdep..."
rosdep update
rosdep install -i --from-path src --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys="librealsense2 warehouse_ros_mongo" -y

# Compilación inicial con colcon cargando el entorno de ROS
echo "Compilando workspace..."
# shellcheck source=/dev/null
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build

agregar_linea_bashrc() {
    local linea="$1"
    if ! grep -Fxq "$linea" "$ARCHIVO_BASHRC"; then
        echo "Agregando línea al archivo $ARCHIVO_BASHRC: $linea"
        echo "$linea" >> "$ARCHIVO_BASHRC"
    else
        echo "la línea $linea ya estaba configurado en ~/.bashrc."
    fi
}

echo "Configurando variables y utilidades en ~/.bashrc..."
agregar_linea_bashrc "source /opt/ros/$ROS_DISTRO/setup.bash"
agregar_linea_bashrc "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
agregar_linea_bashrc "[ -f $ARCHIVO_CONFIGURACION ] && source $ARCHIVO_CONFIGURACION"
agregar_linea_bashrc 'alias source_ws="source ./install/setup.bash"'

# Inyectar funciones personalizadas en bloque único
if ! grep -q "ros2_create_py_pkg()" "$ARCHIVO_BASHRC"; then
cat << 'EOF' >> "$ARCHIVO_BASHRC"

# Función rápida para crear paquetes de ROS 2 en Python
ros2_create_py_pkg() {
    [ -z "$1" ] && { echo "Uso: ros2_create_py_pkg <nombre_paquete>"; return 1; }
    ros2 pkg create "$1" --build-type ament_python --dependencies rclpy
}
EOF
echo "¡Función ros2_create_py_pkg() agregadas con éxito!"
else
    echo "La función ros2_create_py_pkg() ya se encuentra configurada en ~/.bashrc."
fi

# Inyectar funciones personalizadas en bloque único
if ! grep -q "ros2_create_cpp_pkg()" "$ARCHIVO_BASHRC"; then
cat << 'EOF' >> "$ARCHIVO_BASHRC"

# Función rápida para crear paquetes de ROS 2 en C++
ros2_create_cpp_pkg() {
    [ -z "$1" ] && { echo "Uso: ros2_create_cpp_pkg <nombre_paquete>"; return 1; }
    ros2 pkg create "$1" --build-type ament_cmake --dependencies rclcpp
}
EOF
echo "¡Función ros2_create_cpp_pkg() agregada con éxito!"
else
    echo "La función ros2_create_cpp_pkg() ya se encuentra configurada en ~/.bashrc."
fi

# Configuración de Kitty Terminal
echo "Configurando Kitty Terminal ..."
mkdir -p "$(dirname "$KITTY_CONF")"

if command -v kitty &>/dev/null; then
    kitty +kitten themes --reload-in=all Catppuccin-Mocha 2>/dev/null || true
    
    # Limpiar entradas antiguas e insertar personalizadas
    sed -i '/font_size/d' "$KITTY_CONF"
    sed -i '/font_family/d' "$KITTY_CONF"
    sed -i '/hide_window_decorations/d' "$KITTY_CONF"
    sed -i '/tab_bar_min_tabs/d' "$KITTY_CONF"
    sed -i '/tab_bar_edge/d' "$KITTY_CONF"
    sed -i '/tab_bar_style/d' "$KITTY_CONF"
    sed -i '/tab_powerline_style/d' "$KITTY_CONF"
    sed -i '/tab_title_template/d' "$KITTY_CONF"
    sed -i '/active_tab_title_template/d' "$KITTY_CONF"
    
    cat << 'EOF' >> "$KITTY_CONF"
    # --- Configuraciones personalizadas ---
    font_size 16.0
    font_family JetBrainsMono Nerd Font
    hide_window_decorations no
    tab_bar_min_tabs 1
    tab_bar_edge bottom
    tab_bar_style powerline
    tab_powerline_style slanted
    tab_title_template "  {index}:{f'{title[:6]}…{title[-6:]}' if title.rindex(title[-1]) + 1 > 25 else title}{' []' if layout_name == 'stack' else ''} "
    active_tab_title_template " {index}:{f'{title[:6]}…{title[-6:]}' if title.rindex(title[-1]) + 1 > 25 else title}{' []' if layout_name == 'stack' else ''} "
    # --- Fin configuraciones ---
EOF
    kill -SIGUSR1 $(pgrep kitty) 2>/dev/null || true
    echo "¡Configuración de Kitty actualizada con éxito!"
else
    echo "Kitty no está instalado. Saltando configuración de Kitty."
fi

agregar_linea_bashrc "export LIBGL_ALWAYS_SOFTWARE=1"

source ~/.bashrc 2>/dev/null || true
cd $DIRECTORIO_ORIGEN

cowsay -f tux "¡Configuración completada con éxito!" | lolcat

exec bash