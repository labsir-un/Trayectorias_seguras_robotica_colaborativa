# Documentación de comandos del sistema de supervisión ABB IRC5

## 1. Introducción

Este documento describe los comandos genéricos implementados para la comunicación con el controlador ABB IRC5 mediante socket TCP/IP. Incluye sintaxis, parámetros, ejemplos y variables relevantes del programa RAPID.

> Los nombres de comandos, variables y objetivos deben coincidir exactamente con los declarados en RAPID y con lo que espera el parser de mensajes.

## 2. Formato y tipos de parámetros

Los campos de los comandos se separan mediante `/`. La estructura concreta depende de cada comando.

| Parámetro | Significado |
|---|---|
| `<num>` | Valor numérico. |
| `<Punto>` | Nombre de un `robtarget` declarado. |
| `<JConf>` | Configuración de articulaciones. |
| `<speeddata>` | Nombre de un `speeddata` declarado. |
| `<None>` | Campo vacío representado por un espacio (`" "`). |
| `<pos>` | Arreglo de tres valores numéricos que representan una posición. |
| `<Variable>` | Variable RAPID de tipo `num`, `bool`, `string`, `speeddata`, `zonedata`, `wobjdata` o `robtarget`. |

## 3. Comandos de velocidad

### 3.1 Cambiar el porcentaje de velocidad nominal

```text
Velocidad/Porcentaje/VelMax/<num>
```

Ejemplo:

```text
Velocidad/Porcentaje/VelMax/80
```

Cambia el porcentaje de velocidad nominal de trabajo al 80 %.

### 3.2 Cambiar la velocidad máxima

```text
Velocidad/Max/VelMax/<num>
```

Ejemplo:

```text
Velocidad/Max/VelMax/1000
```

Cambia la velocidad máxima de movimiento. Según la documentación fuente, el máximo permitido es `2000` y el mínimo `100`.

### 3.3 Actualizar un `speeddata`

```text
Velocidad/ACTVEL/<speeddata>/<num>
```

Ejemplo:

```text
Velocidad/ACTVEL/Vel_Segura/50
```

Cambia la velocidad TCP del `speeddata` indicado a 50 mm/s.

## 4. Comandos de distancia y trayectoria

### 4.1 Consultar postura del TCP en Euler (RPY)

```text
Distancia/GetPostura/Prototipo/TCPEuler
```

### 4.2 Consultar postura del TCP en cuaterniones

```text
Distancia/GetPostura/Prototipo/TCPCua
```

### 4.3 Actualizar puntos de interés de trayectoria

```text
Distancia/CambiarTrayectoria/ActualizarPuntos/Interes
```

Actualiza los puntos de interés actuales asociados a las condiciones de trayectoria.

### 4.4 Guardar el punto de salida actual

```text
Distancia/CambiarTrayectoria/ActualizarPuntos/GuardarPSalida
```

Guarda el punto de salida de la trayectoria como punto actual de interés.

### 4.5 Establecer offsets

```text
Distancia/CambiarTrayectoria/SetOffsets/<pos>
```

Ejemplo:

```text
Distancia/CambiarTrayectoria/SetOffsets/[0.1, 0.5, -10]
```

Actualiza el offset para modificar la llegada del TCP: `x=0.1`, `y=0.5`, `z=-10`, en el sistema de referencia utilizado por la implementación.

### 4.6 Mover al punto temporal con offsets

Movimiento articular:

```text
Distancia/CambiarTrayectoria/MoveTemp/MoveJoint
```

Movimiento lineal:

```text
Distancia/CambiarTrayectoria/MoveTemp/MoveLinear
```

Ejecuta el movimiento hacia el punto temporal generado mediante offsets, por movimiento articular o lineal.

### 4.7 Mover a un `robtarget`

Movimiento articular:

```text
Distancia/Postura_segura/robtargetJ/<Punto>
```

Movimiento lineal:

```text
Distancia/Postura_segura/robtargetL/<Punto>
```

Ejemplo:

```text
Distancia/Postura_segura/robtargetJ/p_salida
```

Ordena un movimiento articular hacia `p_salida`.

### 4.8 Mover a una configuración articular

```text
Distancia/Postura_segura/JoinConf/<JConf>
```

Ejemplo:

```text
Distancia/Postura_segura/JoinConf/[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]
```

Envía una configuración con las seis articulaciones en cero, usando el formato de configuración esperado por RAPID.

## 5. Comandos de variables

### 5.1 Obtener una variable

```text
Variable/GET/<variable>/<tipo_dato>
```

Para `wobjdata` y `robtarget` se contemplan representaciones de orientación en Euler o cuaterniones:

- `wobjdataEuler`
- `wobjdataCua`
- `robtargetEuler`
- `robtargetCua`

Ejemplo:

```text
Variable/GET/Target_Home_20/robtargetEuler
```

Solicita la postura de `Target_Home_20` en ángulos de Euler.

### 5.2 Establecer una variable

```text
Variable/SET/<variable>/<nuevo_valor>
```

La documentación fuente indica que este comando aplica a variables `num`, `bool` y `string`.

Ejemplo:

```text
Variable/SET/Comenzar/TRUE
```

## 6. `robtargets` importantes

| Objetivo | Descripción |
|---|---|
| `Target_Mantenimiento` | Posición de mantenimiento. |
| `Target_Espera` | Posición de espera. |
| `Target_Home_10` | Posición con las articulaciones en cero. |
| `Target_Home_20` | Punto de transición entre movimientos. |
| `Target_APuntosRef_10` … `Target_APuntosRef_150` | Serie de puntos de referencia/contacto de la aplicación. |

### 6.1 Puntos de contacto identificados

| Objetivo | Ubicación |
|---|---|
| `Target_APuntosRef_20` | Contacto central de la tapa. |
| `Target_APuntosRef_50` | Contacto izquierdo. |
| `Target_APuntosRef_80` | Contacto frontal. |
| `Target_APuntosRef_110` | Contacto derecho. |

La ubicación concreta de los demás puntos de la serie no se especifica en la información proporcionada.

## 7. Velocidades declaradas

```rapid
VAR speeddata Vel_Acercamiento:=v80;
VAR speeddata Vel_Acercamiento_Lateral:=v30;
VAR speeddata Vel_Vacio:=v100;
VAR speeddata Vel_Vacio_Rap:=v200;
VAR speeddata Vel_Proceso:=v60;
VAR speeddata Vel_Segura:=v30;
VAR num VelMax:=250;
VAR num VelPor:=100;
```

| Variable | Valor inicial | Uso indicado |
|---|---:|---|
| `Vel_Acercamiento` | `v80` | Acercamiento. |
| `Vel_Acercamiento_Lateral` | `v30` | Acercamiento lateral. |
| `Vel_Vacio` | `v100` | Movimiento en vacío. |
| `Vel_Vacio_Rap` | `v200` | Movimiento rápido en vacío. |
| `Vel_Proceso` | `v60` | Movimiento de proceso. |
| `Vel_Segura` | `v30` | Movimiento seguro. |
| `VelMax` | `250` | Límite numérico de velocidad del programa. |
| `VelPor` | `100` | Porcentaje de velocidad nominal. |

Los nombres de uso se interpretan a partir de las variables proporcionadas. La velocidad efectiva depende también de la lógica del programa y de los límites configurados.

## 8. Zonas de movimiento declaradas

```rapid
VAR zonedata Z_Acercamiento:=z1;
VAR zonedata Z_VacioMov:=z10;
VAR zonedata Z_Vacio_Rap:=z20;
VAR zonedata Z_Proceso:=z1;
VAR zonedata Z_Paro:=fine;
VAR zonedata Z_Segura:=z1;
```

| Variable | Valor inicial | Uso indicado |
|---|---|---|
| `Z_Acercamiento` | `z1` | Acercamiento. |
| `Z_VacioMov` | `z10` | Movimiento en vacío. |
| `Z_Vacio_Rap` | `z20` | Movimiento rápido en vacío. |
| `Z_Proceso` | `z1` | Movimiento de proceso. |
| `Z_Paro` | `fine` | Llegada exacta al punto. |
| `Z_Segura` | `z1` | Movimiento seguro. |

## 9. Variables de trayectoria

```rapid
PERS string path_contacto_actual;
PERS string path_vacio_actual;

VAR robtarget p_OrigenActual;
VAR robtarget p_DestinoActual;
PERS robtarget p_MoveActual;
PERS robtarget p_objActual;

PERS robtarget p_contacto_origen;
PERS robtarget p_contacto_destino;
PERS robtarget p_vacio_origen;
PERS robtarget p_vacio_destino;

PERS robtarget p_salida_guardado;

PERS robtarget p_actual;
PERS robtarget p_salida;

PERS bool Postura_segura:=FALSE;
```

| Variable | Tipo | Descripción |
|---|---|---|
| `path_contacto_actual` | `string` | Identificador de trayectoria de contacto actual. |
| `path_vacio_actual` | `string` | Identificador de trayectoria en vacío actual. |
| `p_OrigenActual` | `robtarget` | Punto de origen actual. |
| `p_DestinoActual` | `robtarget` | Punto de destino actual. |
| `p_MoveActual` | `robtarget` | Punto de movimiento actual. |
| `p_objActual` | `robtarget` | Punto objetivo actual. |
| `p_contacto_origen` / `p_contacto_destino` | `robtarget` | Origen y destino de trayectoria de contacto. |
| `p_vacio_origen` / `p_vacio_destino` | `robtarget` | Origen y destino de trayectoria en vacío. |
| `p_salida_guardado` | `robtarget` | Punto de salida guardado. |
| `p_actual` | `robtarget` | Punto actual. |
| `p_salida` | `robtarget` | Punto de salida. |
| `Postura_segura` | `bool` | Indicador de postura segura; inicializado en `FALSE`. |

Se conservan los prefijos `PERS` y `VAR` de las declaraciones proporcionadas.

## 10. Variables de control por socket

```rapid
PERS bool Sock_Parada:=FALSE;
PERS bool Sock_Pausa:=FALSE;
PERS bool Sock_Bloquear:=TRUE;
PERS bool Sock_Reinicio:=FALSE;
```

| Variable | Valor inicial | Función indicada |
|---|---:|---|
| `Sock_Parada` | `FALSE` | Control/estado de parada por socket. |
| `Sock_Pausa` | `FALSE` | Control/estado de pausa por socket. |
| `Sock_Bloquear` | `TRUE` | Variable de bloqueo; inicia activa. |
| `Sock_Reinicio` | `FALSE` | Solicitud de reinicio por socket. |

La acción efectiva depende de la lógica RAPID que supervise estas variables.

## 11. Secuencias de ejemplo

### 11.1 Ir a articulaciones cero y recuperar el punto de salida

```text
Distancia/Postura_segura/JoinConf/[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]
Distancia/Postura_segura/robtargetJ/p_salida
Variable/SET/Sock_Reinicio/TRUE
```

### 11.2 Ir a articulaciones cero y recuperar un punto con offset

```text
Distancia/Postura_segura/JoinConf/[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]
Distancia/CambiarTrayectoria/ActualizarPuntos/GuardarPSalida
Distancia/CambiarTrayectoria/ActualizarPuntos/Interes
Distancia/CambiarTrayectoria/SetOffsets/[0.1, 0.5, -10]
Distancia/CambiarTrayectoria/MoveTemp/MoveJoint
Distancia/CambiarTrayectoria/SetOffsets/[0.0, 0.0, -8]
Distancia/CambiarTrayectoria/MoveTemp/MoveLinear
Distancia/Postura_segura/robtargetL/p_salida_guardado
Variable/SET/Sock_Reinicio/TRUE
```

### 11.3 Movimiento articular a un punto y retorno

```text
Distancia/Postura_segura/robtargetJ/Target_RobLab_50
Distancia/Postura_segura/robtargetJ/p_salida
Variable/SET/Sock_Reinicio/TRUE
```

### 11.4 Consultar orientación de un objetivo

```text
Variable/GET/Target_Home_20/robtargetEuler
Variable/GET/Target_Home_20/robtargetCua
```

## 12. Consideraciones

- Respetar exactamente los nombres de comandos, variables y objetivos.
- Mantener el orden de campos y separadores que espera el parser.
- Enviar cada valor con el formato que espera el tipo RAPID correspondiente.
- Verificar que los datos estén declarados en un ámbito accesible para el código que procesa el comando.
- Ejecutar secuencias de movimiento únicamente cuando el estado del sistema y las condiciones de seguridad lo permitan.
- Esta documentación describe la interfaz de comandos; no sustituye la validación de seguridad de la aplicación ni las funciones de seguridad del controlador.
