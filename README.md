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
- [Resumen de paquetes](#resumen-de-paquetes)
- [Configuración para WLS](#configuración-para-wls)
</details>

---

## Resumen de archivos

- `instalar.sh`: Instalación automática de programas utiliados.
- `configuracionWS.conf`: Parámetros de configuración inicial del Work Space.
- `configuracionWS.sh`: Configuración del Work Space inicial y de comandos para trabajo.

## Resumen de paquetes

- `colab_supervision_bringup`: Paquete para archivos launch e inicialización de procesos.
- `colab_supervision_commader`: Paquete para nodos de control por medio de la API de C++ de moveit.
- `colab_supervision_moveit_config`: Paquete de configuración de moveit2.
- `colab_supervision_description`: Paquete de configuración de URDF del ABB IRB140.
- `colab_supervision_interfaces`: Paquete para interfaces de comunicación entre nodos.
- `realsense-ros`: ROS Wrapper para camaras RealSense.

## Configuración para WLS

En PowerShell como Administrador

```sh
#Instalar  USBIPD
winget install --interactive --exact dorssel.usbipd-win
#Ver el BUSID de las camaras
usbipd list
#Compartirlos con el WLS
usbipd bind --busid <BUSID>
#Conectar el dispositvo con WLS
usbipd attach --wsl --busid <BUSID>
``` 