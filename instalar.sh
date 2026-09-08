#!/bin/bash
set -euo pipefail

# Obtener directorio del script y cargar configuración
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/configuracionWS.conf"

echo "=== [1/4] Actualizando índices del sistema ==="
sudo apt-get update -y

# Helpers de instalación
instalar_apt() {
    local pkg="$1"
    # Validar que no se haya enviado un argumento vacío
    if [ -z "$pkg" ]; then
        echo "Error interno de script: No se especificó un paquete para validar."
        return 1
    fi
    if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "ok installed"; then
        echo "[APT] $pkg ya está instalado."
    else
        echo "[APT] Instalando $pkg..."
        sudo apt-get install -y "$pkg"
    fi
}

instalar_snap() {
    local pkg="$1"
    local modo="${2:-}"
    # Validar que se haya enviado el nombre del paquete
    if [ -z "$pkg" ]; then
        echo "Error interno de script: No se especificó un paquete Snap."
        return 1
    fi
    if snap list "$pkg" &>/dev/null; then
        echo "[SNAP] $pkg ya está instalado."
    else
        echo "[SNAP] Instalando $pkg..."
        if [ "$modo" = "--classic" ]; then
            sudo snap install "$pkg" --classic
        else
            sudo snap install "$pkg"
        fi
    fi
}

agregar_linea_bashrc() {
    local linea="$1"
    if ! grep -Fxq "$linea" "$ARCHIVO_BASHRC"; then
        echo "Agregando línea al archivo $ARCHIVO_BASHRC: $linea"
        echo "$linea" >> "$ARCHIVO_BASHRC"
    else
        echo "la línea $linea ya estaba configurado en ~/.bashrc."
    fi
}


echo "=== [2/4] Instalando utilidades básicas ==="
instalar_apt cowsay
instalar_apt lolcat
instalar_apt curl
instalar_apt usbutils
intalar_apt v4l-utils
instalar_snap code --classic

echo "=== [3/4] Instalando Kitty Terminal ==="
if [ ! -d "$HOME/.local/kitty.app" ]; then
    mkdir -p "$HOME/.local/bin" "$HOME/.local/share/applications" "$HOME/.config"
    echo "Descargando instalador oficial de Kitty..."
    curl -L https://sw.kovidgoyal.net/kitty/installer.sh | sh /dev/stdin
    echo "Creando accesos directos en ~/.local/bin/..."
    ln -sf "$HOME/.local/kitty.app/bin/kitty" "$HOME/.local/bin/kitty"
    ln -sf "$HOME/.local/kitty.app/bin/kitten" "$HOME/.local/bin/kitten"
    
    echo "Configurando acceso directo en el sistema de aplicaciones..."
    cp "$HOME/.local/kitty.app/share/applications/kitty.desktop" "$HOME/.local/share/applications/"
    sed -i "s|Icon=kitty|Icon=$HOME/.local/kitty.app/share/icons/hicolor/256x256/apps/kitty.png|g" "$HOME/.local/share/applications/kitty*.desktop"
    sed -i "s|Exec=kitty|Exec=$HOME/.local/kitty.app/bin/kitty|g" "$HOME/.local/share/applications/kitty*.desktop"
    echo 'kitty.desktop' > "$HOME/.config/xdg-terminals.list"
    echo "¡Instalaciónde Kitty completada con éxito!"
else
    echo "[KITTY] Ya se encuentra instalado."
fi

echo "=== [4/4] Configurando ROS 2 ($ROS_DISTRO) ==="
# Configuración UTF-8
if ! locale | grep -q -i "UTF-8"; then
    echo "Configuración UTF-8 no detectada. Iniciando instalación y ajuste..."
    sudo apt-get install -y locales
    sudo locale-gen en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Configuración completada con éxito."
else
    echo "El sistema ya cuenta con una configuración UTF-8 activa. No se requieren cambios."
fi

if [ ! -d "/opt/ros/$ROS_DISTRO" ]; then
    echo "ROS 2 Jazzy no detectado. Iniciando el proceso de instalación..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository universe -y
    
    sudo apt update && sudo apt install curl -y
    ROS_APT_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | cut -d'"' -f4)
    UBUNTU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.${UBUNTU_CODENAME}_all.deb"
    
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update -y
    sudo apt-get install -y ros-dev-tools "ros-$ROS_DISTRO-desktop" python3-rosdep "ros-$ROS_DISTRO-urdf-tutorial"
    sudo apt update && sudo apt upgrade
    sudo apt autoremove
else
    echo "¡ROS 2 Jazzy ya está instalado en '/opt/ros/$ROS_DISTRO'! Saltando instalación..."
fi

agregar_linea_bashrc "source /opt/ros/jazzy/setup.bash"

# Dependencias ROS adicionales
instalar_apt "ros-$ROS_DISTRO-realsense2-camera"
instalar_apt "ros-$ROS_DISTRO-moveit"
instalar_apt "ros-$ROS_DISTRO-rmw-cyclonedds-cpp"
instalar_apt "ros-$ROS_DISTRO-ros2-control"
instalar_apt "ros-$ROS_DISTRO-ros2-controllers"
instalar_apt "ros-$ROS_DISTRO-controller-manager"
instalar_apt "ros-$ROS_DISTRO-tf-transformations"

cd $DIRECTORIO_ORIGEN

cowsay -f tux "¡Instalación base completada!" | lolcat

exec bash