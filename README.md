<div align="center">
<picture>
    <source srcset="https://imgur.com/5bYAzsb.png" media="(prefers-color-scheme: dark)">
    <source srcset="https://imgur.com/Os03JoE.png" media="(prefers-color-scheme: light)">
    <img src="https://imgur.com/Os03JoE.png" alt="Escudo UNAL" width="350px">
</picture>

<h1>Determinación de trayectorias seguras en entornos de robótica colaborativa supervisada</h1>

<h2>Juan Sebastian De Jesus Daleman Martinez</h2>

<h3>Trabajo de grado</h3>

<h3>Director:<br>
Pedro Fabián Cárdenas Herrera<br>
Profesor de tiempo completo - Departamento de Ingeniería Mecánica y Mecatrónica<br>
Facultad de Ingeniería<br>
Universidad Nacional de Colombia<br><br><br>
<b>Línea de investigación:<b><br>
Automatización y Robótica</h3>
</div>

---

<div id="programming_lenguages" align="center">
<!--
Para generar las medallas 
https://shields.io/badges/static-badge
Iconos para las medallas disponibles
https://simpleicons.org/?q=HTML
Para colores en HEX
https://htmlcolorcodes.com/es/
Algunos diseños ya realizados de medallas
https://github.com/alexandresanlim/Badges4-README.md-Profile#-contact-
-->
<img src="https://img.shields.io/badge/Ubuntu%2024.04%20LST-E95420?logo=ubuntu&logoColor=FFFFFF" alt="Python Badge" height="28px">
<img src="https://img.shields.io/badge/ROS%202%20JAZZY-22314E?logo=ros&logoColor=FFFFFF" alt="ROS 2 Badge"  height="28px">
<img src="https://img.shields.io/badge/MoveIt%202-00599C?logo=ros&logoColor=white" alt="MoveIt 2 Badge" height="28px">
<img src="https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=FFD43B" alt="Python Badge" height="28px">
<img src="https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white" alt="C++ Badge" height="28px">
<img src="https://img.shields.io/badge/IRB%20140-FF000F?logo=ABB&logoColor=FFFFFF" alt="ABB Badge"  height="28px">
<img src="https://img.shields.io/badge/REALSENSE%20D435i-0071C5?logo=intel&logoColor=white" alt="LaTeX Badge"  height="28px">
<img src="https://img.shields.io/badge/Perception-Robot%20Vision-8A2BE2" alt="Robot Vision" height="28px">
<img src="https://img.shields.io/badge/Trajectory-Safe%20Planning-228B22" alt="Safe Trajectory Planning" height="28px">
<img src="https://img.shields.io/badge/LaTeX-008000?logo=latex&logoColor=white" alt="LaTeX Badge"  height="28px">
</div>


---

Implementación de determinación de trayectorias seguras en entrono de robótica colaborativa supervisada. Proyecto de grado de maestría. Juan Sebastian Daleman

---


<details>
    <summary>Tabla de contenido</summary>

<!-- TOC -->
- [Resumen de archivos](#resumen-de-archivos)
- [Resumen carpeta robotstudio](#resumen-carpeta-robotstudio)
- [Resumen de paquetes](#resumen-de-paquetes)
- [Configuración para WLS](#configuración-para-wls)
  - [Configuración para habilitar dispositivos conectador por USB](#configuración-para-habilitar-dispositivos-conectador-por-usb)
- [Agradecimientos](#agradecimientos)
</details>

---

## Resumen de archivos

- `instalar.sh`: Instalación automática de programas utiliados.
- `configuracionWS.conf`: Parámetros de configuración inicial del Work Space.
- `configuracionWS.sh`: Configuración del Work Space inicial y de comandos para trabajo.

## Resumen carpeta robotstudio

- `abb_driver`: Programa desarrollado con PC SDK para comunicación entre el controlador IRC5 y un socket.
- `ColabControl`: Modulos de programa de RAPID para la tarea colaborativa y parser de mensajes del socket.
- `pack_and_go`: Pack and Go de la estación final para simulacion.

## Resumen de paquetes

- `colab_supervision_bringup`: Paquete para archivos launch e inicialización de procesos.
- `colab_supervision_commader`: Paquete para nodos de control por medio de la API de C++ de moveit.
- `colab_supervision_moveit_config`: Paquete de configuración de moveit2.
- `colab_supervision_description`: Paquete de configuración de URDF del ABB IRB140.
- `colab_supervision_interfaces`: Paquete para interfaces de comunicación entre nodos.
- `realsense-ros`: ROS Wrapper para camaras RealSense.
- `ros_industrial_abb_driver`: Paquete de ROS-industrial con modificaciones de [abb_ros2](https://github.com/PickNikRobotics/abb_ros2/tree/rolling) para comunicaciones con el controlador IRC5.

## Configuración para WLS

### Configuración para habilitar dispositivos conectador por USB

En PowerShell como Administrador lanzar los siguientes comandos.

```sh
#Instalar  USBIPD
winget install --interactive --exact dorssel.usbipd-win
#Ver el BUSID de las camaras
usbipd list
#Compartirlos con el WLS
usbipd bind --busid <$BUSID>
#Conectar el dispositvo con WLS
usbipd attach --wsl --busid <$BUSID>
``` 
<div align="center">
    <img src="https://imgur.com/vJ7GwkB.png" alt="Ej_config_dispositivo_sub_compoartido">
</div>

>[!IMPORTANT] 
>Cada vez que se reinicie el WLS o se use el recurso compartido se debe verificar que este unido al WLS para su funcionamiento.


## Agradecimientos

- [PickNikRobotics - ABB driver ros2](https://github.com/PickNikRobotics/abb_ros2)
- [abb_irb140_support](https://github.com/FreddyMartinez/abb_irb140_support/tree/master)
- [Moviet2 Documentation](https://moveit.picknik.ai/main/index.html)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)