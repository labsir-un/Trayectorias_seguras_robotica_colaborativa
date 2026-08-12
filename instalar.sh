#!/bin/bash

# Intalación de herramientas necesarias

# Definición de la función de instalación segura por APT
instalar_si_falta_apt() {
    # Guardar el nombre del paquete pasado como primer argumento
    local PAQUETE="$1"

    # Validar que no se haya enviado un argumento vacío
    if [ -z "$PAQUETE" ]; then
        echo "Error interno de script: No se especificó un paquete para validar."
        return 1
    fi

    echo "Verificando si $PAQUETE ya está instalado..."

    # Verificar el estado del paquete en la base de datos de dpkg
    if dpkg-query -W -f='${Status}' "$PAQUETE" 2>/dev/null | grep -q "ok installed"; then
        echo "¡$PAQUETE ya está instalado! Saltando este paso."
    else
        echo "$PAQUETE no está instalado. Iniciando la instalación..."
        # Actualizar los índices e instalar de forma desatendida (-y)
        sudo apt update && sudo apt install -y "$PAQUETE"
    fi
}

# Función general para instalar paquetes mediante Snap
instalar_si_falta_snap() {
    local PAQUETE="$1"
    local MODO="$2" # Puede ser "classic" o dejarse vacío

    # Validar que se haya enviado el nombre del paquete
    if [ -z "$PAQUETE" ]; then
        echo "Error interno de script: No se especificó un paquete Snap."
        return 1
    fi

    echo "Verificando si el paquete snap '$PAQUETE' ya está instalado..."

    # snap list verifica si el paquete está registrado en el sistema
    if snap list "$PAQUETE" >/dev/null 2>&1; then
        echo "¡El snap '$PAQUETE' ya está instalado! Saltando este paso."
    else
        echo "El snap '$PAQUETE' no está instalado. Iniciando instalación..."
        
        # Evaluar si requiere el parámetro --classic
        if [ "$MODO" = "classic" ]; then
            sudo apt update
            sudo snap install "$PAQUETE" --classic
        else
            sudo apt update
            sudo snap install "$PAQUETE"
        fi
        echo "¡El snap '$PAQUETE' se instaló con éxito!"
    fi
}

#Cowsay
instalar_si_falta_apt cowsay

#lolcat
instalar_si_falta_apt lolcat


#Kitty terminal
PAQUETE="kitty"

echo "Verificando si $PAQUETE Terminal ya está instalado..."

# Verificamos si la carpeta del ejecutable de Kitty ya existe en tu directorio local
if [ -d "$HOME/.local/kitty.app" ]; then
    echo "¡$PAQUETE ya está instalado en ~/.local/kitty.app/! Saltando la instalación."
else
    echo "$PAQUETE no se encuentra en el sistema. Iniciando proceso de instalación..."
    
    # Asegurar que el directorio local de binarios existe
    mkdir -p ~/.local/bin/
    mkdir -p ~/.local/share/applications/

    # Descargar e instalar la versión estable oficial (puedes cambiar a 'installer=nightly' si lo prefieres)
    echo "Descargando instalador oficial de Kitty..."
    curl -L https://sw.kovidgoyal.net/kitty/installer.sh | sh /dev/stdin

    # Crear enlaces simbólicos para ejecutar 'kitty' y 'kitten' desde cualquier lugar
    echo "Creando accesos directos en ~/.local/bin/..."
    ln -sf ~/.local/kitty.app/bin/kitty ~/.local/kitty.app/bin/kitten ~/.local/bin/

    # Copiar el archivo de integración de escritorio (.desktop) para que aparezca en tus aplicaciones
    echo "Configurando acceso directo en el sistema de aplicaciones..."
    cp ~/.local/kitty.app/share/applications/kitty.desktop ~/.local/share/applications/

    # Corregir las rutas del icono y del ejecutable en el archivo .desktop (reemplaza 'kitty' por la ruta absoluta)
    sed -i "s|Icon=kitty|Icon=$(readlink -f ~)/.local/kitty.app/share/icons/hicolor/256x256/apps/kitty.png|g" ~/.local/share/applications/kitty*.desktop
    sed -i "s|Exec=kitty|Exec=$(readlink -f ~)/.local/kitty.app/bin/kitty|g" ~/.local/share/applications/kitty*.desktop

    curl -L https://sw.kovidgoyal.net/kitty/installer.sh | sh /dev/stdin \
    installer=nightly
    
    # Registrar Kitty como una opción de terminal válida en el entorno de escritorio
    echo "Estableciendo Kitty en la lista de terminales XDG..."
    echo 'kitty.desktop' > ~/.config/xdg-terminals.list

    echo "¡Instalación y configuración de Kitty completada con éxito!"
fi



#Visual studio code
instalar_si_falta_snap code --classic

#ROS Jazzy
PAQUETE="ros-jazzy-desktop"
echo "Verificando si $PAQUETE ya está instalado..."

# Comprobar si la salida de locale NO contiene la palabra UTF-8 (ignorando mayúsculas)
if ! locale | grep -q -i "UTF-8"; then
    echo "Configuración UTF-8 no detectada. Iniciando instalación y ajuste..."

    # Actualizar repositorios e instalar el paquete de locales
    sudo apt update && sudo apt install locales

    # Generar la codificación en_US y en_US.UTF-8
    sudo locale-gen en_US en_US.UTF-8

    # Actualizar la configuración del sistema de forma permanente
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    # Exportar la variable para la sesión actual de la terminal
    export LANG=en_US.UTF-8
    
    echo "Configuración completada con éxito."
else
    echo "El sistema ya cuenta con una configuración UTF-8 activa. No se requieren cambios."
fi

if [ -d "/opt/ros/jazzy" ]; then
    echo "¡ROS 2 Jazzy ya está instalado en '/opt/ros/jazzy'! Saltando instalación..."

else
    echo "ROS 2 Jazzy no detectado. Iniciando el proceso de instalación..."

    #Verificar que los repositorios necesarios esten habilitados
    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

    sudo dpkg -i /tmp/ros2-apt-source.deb

    #Intalar herramientas de desarrollo de ROS
    sudo apt update && sudo apt install ros-dev-tools

    #Iniciar intalación de ROS 2 Jazzy
    sudo apt update && sudo apt upgrade
    #Version de escritorio de ROS 2 Jazzy
    sudo apt install ros-jazzy-desktop
    sudo apt update && sudo apt upgrade
    sudo apt autoremove
    
fi

#Configurar ambiente de ROS 2 Jazzy para que se cargue automáticamente al iniciar la terminal
ROS_SETUP_LINE="source /opt/ros/jazzy/setup.bash"

# Verificar si la línea ya existe en el .bashrc para no duplicarla
if ! grep -Fq "$ROS_SETUP_LINE" ~/.bashrc; then
    echo "Agregando el entorno de ROS 2 Jazzy a ~/.bashrc..."
    echo "$ROS_SETUP_LINE" >> ~/.bashrc
    echo "¡Configuración agregada con éxito!"
else
    echo "El entorno de ROS 2 Jazzy ya estaba configurado en ~/.bashrc."
fi


#Instalación de dependencias para RealSense
instalar_si_falta_apt ros-$ROS_DISTRO-realsense2-*

cd $DIRECTORIO_ORIGEN

cowsay -f tux "¡Instalación de herramientas completada con éxito!" | lolcat

exec bash