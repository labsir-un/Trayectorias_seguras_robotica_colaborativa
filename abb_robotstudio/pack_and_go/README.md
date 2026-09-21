# Puesta en marcha y conexión de ColabControl con ABB IRC5

## 1. Objetivo

Este documento describe el procedimiento general para iniciar la estación ABB, ejecutar la interfaz `ColabControl` y establecer la comunicación con el controlador mediante socket TCP/IP.

El flujo contempla abrir la estación, iniciar el FlexPendant, ejecutar la interfaz, iniciar la simulación, activar `Comenzar`, esperar la detección de `abb_driver` y conectarse al socket correspondiente, siguiendo el enfoque del programa de ejemplo `socket_ver.py`.

## 2. Requisitos previos

Antes de iniciar, verificar que se dispone de:

- La estación ABB correspondiente al sistema.
- FlexPendant disponible y operativo.
- Interfaz `ColabControl`.
- `abb_driver` en ejecución y accesible para el sistema.
- Programa de prueba `socket_ver.py` como referencia para la conexión TCP/IP.
- Configuración de red y puerto del socket correspondientes al controlador o entorno de simulación.

> **Nota:** la dirección IP, el puerto y el mecanismo exacto de detección deben corresponder a la configuración real del proyecto. Este documento no fija valores que no hayan sido especificados.

## 3. Procedimiento de puesta en marcha

### Paso 1. Abrir la estación ABB

1. Abrir la estación ABB correspondiente en el entorno de simulación.
2. Esperar a que la estación termine de cargar.
3. Comprobar que el controlador virtual y los componentes de la estación se encuentren disponibles.

### Paso 2. Abrir el FlexPendant

1. Abrir el FlexPendant asociado a la estación.
2. Verificar que el controlador esté operativo.
3. Comprobar que el programa RAPID requerido esté cargado y listo para ejecutarse.

### Paso 3. Ejecutar `ColabControl`

1. Iniciar la interfaz `ColabControl`.
2. Verificar que la interfaz se abra correctamente.
3. Confirmar que los controles y estados requeridos estén disponibles.

### Paso 4. Iniciar la simulación

1. Desde el entorno ABB, iniciar la simulación de la estación.
2. Verificar que el controlador y el programa RAPID estén en el estado requerido para la prueba.
3. Confirmar que no existan errores que impidan continuar.

### Paso 5. Activar `Comenzar`

1. Desde `ColabControl`, activar la opción `Comenzar`.
2. Esta acción inicia el proceso de búsqueda/detección de `abb_driver`.
3. Esperar a que la interfaz identifique el driver.

No se debe intentar establecer la conexión con el socket correspondiente hasta que el proceso de detección indique que `abb_driver` fue encontrado.

### Paso 6. Conectarse al socket

1. Una vez detectado `abb_driver`, seleccionar o utilizar el socket correspondiente a la configuración activa.
2. Establecer la conexión TCP/IP siguiendo el procedimiento de `socket_ver.py`.
3. Verificar que la conexión se haya establecido correctamente antes de enviar comandos.
4. Mantener la conexión activa durante la ejecución de las pruebas y supervisar los estados de comunicación.

## 4. Flujo resumido

```text
Abrir estación ABB
        ↓
Abrir FlexPendant
        ↓
Ejecutar ColabControl
        ↓
Iniciar simulación
        ↓
Activar Comenzar
        ↓
Buscar y detectar abb_driver
        ↓
Conectarse al socket correspondiente
        ↓
Verificar comunicación
        ↓
Enviar comandos y supervisar estados
```

## 5. Verificación de comunicación

Antes de operar, confirmar:

- La estación ABB está abierta y la simulación está iniciada.
- El FlexPendant muestra el controlador y el programa en el estado esperado.
- `ColabControl` está ejecutándose.
- La opción `Comenzar` fue activada.
- `abb_driver` fue detectado por la interfaz.
- La conexión al socket correspondiente se estableció correctamente.
- La comunicación permite intercambiar mensajes conforme al protocolo implementado.

## 6. Referencia a `socket_ver.py`

`socket_ver.py` se utiliza como programa de referencia para el establecimiento de la conexión TCP/IP con el socket. La interfaz `ColabControl` debe seguir el mismo criterio de conexión aplicable al entorno seleccionado.

Los parámetros concretos de conexión —por ejemplo, dirección IP, puerto, formato de mensajes y gestión de desconexiones— deben consultarse en la versión vigente de `socket_ver.py` y en la configuración del sistema.

## 7. Consideraciones

- La detección de `abb_driver` y la conexión TCP/IP son etapas distintas: detectar el driver no implica necesariamente que el socket ya esté conectado.
- No enviar comandos de movimiento hasta verificar que el controlador, el programa RAPID y la comunicación estén en condiciones de operación.
- Ante una desconexión, error de comunicación o estado inesperado, detener la prueba y revisar el estado del controlador y la conexión antes de reanudar.
- Este procedimiento documenta la secuencia de puesta en marcha; no reemplaza los procedimientos de seguridad ni las verificaciones de la celda.
