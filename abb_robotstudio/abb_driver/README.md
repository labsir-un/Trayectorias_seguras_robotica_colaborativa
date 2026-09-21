# ABB Bridge Socket

Bridge de comunicación entre un nodo externo, como un nodo de **ROS 2**, y un controlador **ABB IRC5** mediante **TCP/IP** y **ABB PC SDK**.

El programa actúa como intermediario entre el cliente TCP y las variables RAPID del controlador. El nodo externo envía comandos como cadenas de texto, el Bridge los escribe en una variable RAPID llamada `cmd` y RAPID se encarga de interpretar y ejecutar el comando.

Las respuestas generadas por RAPID se almacenan en `respuesta_robot`. El Bridge detecta los cambios de esta variable mediante `ValueChanged` y transmite su contenido nuevamente al nodo TCP.

---

## 1. Descripción general

La arquitectura de comunicación es:

```text
┌──────────────────────┐
│      Nodo ROS 2      │
│   Cliente TCP/IP     │
└──────────┬───────────┘
           │
           │ TCP/IP
           │ Puerto 5000
           ▼
┌─────────────────────────────┐
│      ABB Bridge Socket      │
│                             │
│      C# / ABB PC SDK        │
└──────────┬──────────────────┘
           │
           │ ABB PC SDK
           ▼
┌─────────────────────────────┐
│        ABB IRC5             │
│                             │
│ T_ROB1                      │
│   └── Socket                │
│       ├── cmd               │
│       ├── respuesta_robot   │
│       └── app_conect        │
└─────────────────────────────┘
```

El Bridge **no interpreta el contenido de los comandos**. Su función es recibir la cadena enviada por el nodo y escribirla completa en `cmd`.

Por ejemplo:

```text
Variable/SET/Comenzar/TRUE
```

El procesamiento posterior corresponde al programa RAPID.

---

# 2. Variables RAPID utilizadas

El programa utiliza tres variables ubicadas en:

```text
Task:       T_ROB1
Módulo:     Socket
```

Las variables son:

| Variable | Tipo esperado | Función |
|---|---|---|
| `cmd` | `string` | Recibe el comando enviado por el nodo |
| `respuesta_robot` | `string` | Contiene la respuesta generada por RAPID |
| `app_conect` | `bool` | Indica si existe una conexión activa con el cliente TCP |

El Bridge obtiene estas variables utilizando:

```text
T_ROB1/Socket/cmd
T_ROB1/Socket/respuesta_robot
T_ROB1/Socket/app_conect
```

La carga de estas variables se realiza mediante `GetRapidData()`.

---

# 3. Flujo de comandos

El flujo de envío de un comando es:

```text
Nodo ROS 2
    │
    │ "Variable/SET/Comenzar/TRUE\n"
    ▼
TCP Server
    │
    ▼
HandleClient()
    │
    ▼
InyectarComandoAlRobot()
    │
    ▼
T_ROB1/Socket/cmd
    │
    ▼
Interrupción RAPID
    │
    ▼
Procesamiento del string
    │
    ▼
Acción correspondiente
```

El Bridge recibe los datos desde `NetworkStream`, mantiene un buffer y utiliza el carácter de nueva línea `\n` como delimitador de comandos. Esto permite recibir un comando completo incluso cuando TCP entrega los datos en varios fragmentos.

Por ejemplo, el nodo debe enviar:

```text
Variable/SET/Comenzar/TRUE\n
```

El Bridge reconstruye la cadena y finalmente ejecuta:

```csharp
InyectarComandoAlRobot(command);
```

---

# 4. Escritura del comando en RAPID

El Bridge no separa ni analiza el contenido del comando.

Para el Bridge:

```text
Variable/SET/Comenzar/TRUE
```

es simplemente una cadena.

La cadena se convierte a:

```csharp
ABB.Robotics.Controllers.RapidDomain.String
```

y se escribe en:

```text
T_ROB1/Socket/cmd
```

La escritura se realiza mediante Mastership del dominio `Rapid`, necesario para modificar datos RAPID.

El flujo es:

```text
texto recibido
       ↓
RapidDomain.String
       ↓
Mastership.Request(controller.Rapid)
       ↓
cmd.Value = rapidString
       ↓
Mastership liberado
```

Esto permite que el procesamiento del comando permanezca completamente del lado de RAPID.

---

# 5. Procesamiento en RAPID

El Bridge no ejecuta directamente operaciones como:

```text
Comenzar := TRUE
CambiarTrayectoria
ActualizarPuntos
Distancia
```

Estas operaciones corresponden al programa RAPID.

Por ejemplo, cuando el nodo envía:

```text
Variable/SET/Comenzar/TRUE
```

el Bridge solamente realiza:

```text
cmd := "Variable/SET/Comenzar/TRUE"
```

Posteriormente, la interrupción RAPID detecta el cambio de `cmd`, analiza la cadena y realiza la operación correspondiente.

Ejemplo conceptual:

```text
cmd cambia
    ↓
Interrupción RAPID
    ↓
procesa string
    ↓
busca "Comenzar"
    ↓
Comenzar := TRUE
```

---

# 6. Procesamiento de `respuesta_robot`

El canal de respuesta funciona de manera independiente al envío de comandos.

El Bridge se suscribe al evento:

```csharp
_rapidRespuesta.ValueChanged +=
    OnRobotVariablesChanged;
```

La suscripción se realiza durante la inicialización y nuevamente después de una reconexión.

Cuando RAPID modifica:

```text
respuesta_robot
```

se ejecuta:

```csharp
OnRobotVariablesChanged()
```

El programa obtiene la referencia actual de `_rapidRespuesta` y lee su valor:

```csharp
ABB.Robotics.Controllers.RapidDomain.String respuesta =
    (ABB.Robotics.Controllers.RapidDomain.String)
    responseData.Value;
```

Después obtiene:

```csharp
string valorActual =
    respuesta.Value;
```

y finalmente envía ese contenido al cliente TCP mediante:

```csharp
SendToClient(valorActual);
```



---

# 7. Flujo completo de respuesta

El flujo de respuesta es:

```text
RAPID
   │
   │ cambia respuesta_robot
   ▼
ValueChanged
   │
   ▼
OnRobotVariablesChanged()
   │
   ▼
_read respuesta_robot_
   │
   ▼
valorActual
   │
   ▼
SendToClient()
   │
   ▼
NetworkStream.Write()
   │
   ▼
Nodo ROS 2
```

El Bridge agrega `\n` al final de la respuesta antes de enviarla:

```text
respuesta_robot + "\n"
```

Esto permite que el cliente pueda utilizar nuevamente el carácter de nueva línea como delimitador.

### Importante

El Bridge **no genera respuestas artificiales**.

No existen:

```text
ACK
NACK
```

La respuesta enviada al nodo corresponde directamente al contenido de:

```text
respuesta_robot
```

---

# 8. Variable `app_conect`

La variable:

```text
app_conect
```

indica el estado de comunicación entre el Bridge y el nodo TCP.

Cuando un cliente TCP se conecta:

```text
Nodo ROS 2
    ↓
Bridge
    ↓
app_conect := TRUE
```

El Bridge utiliza Mastership para modificar esta variable.

Cuando el cliente se desconecta:

```text
Nodo ROS 2
    X
Bridge
    ↓
app_conect := FALSE
```

El cierre se realiza desde `CloseClient()`.

En RAPID se puede utilizar esta variable para conocer si la aplicación externa continúa conectada.

---

# 9. Servidor TCP

El Bridge funciona como servidor TCP y escucha en:

```text
Puerto: 5000
IP:     127.0.0.1
```

La implementación utiliza:

```csharp
TcpListener(
    IPAddress.Any,
    SocketPort
);
```

con:

```text
SocketPort = 5000
```

El servidor acepta un cliente TCP y obtiene su `NetworkStream`.

La comunicación utilizada es:

```text
Nodo = cliente TCP
Bridge = servidor TCP
```

---

# 10. Protocolo TCP

## Comando

Los comandos se transmiten como una cadena terminada en:

```text
\n
```

Ejemplo:

```text
Variable/SET/Comenzar/TRUE\n
```

Otro ejemplo:

```text
Distancia/Postura_segura/JoinConf/[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]\n
```

El Bridge conserva la cadena completa y la escribe en `cmd`.

## Respuesta

Las respuestas provenientes de RAPID también se envían terminadas en:

```text
\n
```

Ejemplo:

```text
Variable/OK/Comenzar/TRUE\n
```

El contenido exacto depende de lo que RAPID escriba en:

```text
respuesta_robot
```

---

# 11. Selección del controlador

Al iniciar el programa se muestra un menú:

```text
============================================================
 SELECCION DE CONTROLADOR
============================================================
1. Controlador real
2. Controlador virtual
3. Cerrar
============================================================
```

Para un IRC5 físico se debe seleccionar:

```text
1. Controlador real
```

El Bridge ejecuta un `NetworkScanner`, obtiene los controladores disponibles y muestra:

```text
IP
ID
Nombre
Estado
```

para seleccionar el controlador.

---

# 12. Conexión con el IRC5

Después de seleccionar el controlador, el programa realiza:

```text
NetworkScanner
      ↓
ControllerInfo
      ↓
Controller.Connect()
      ↓
Logon(UserInfo.DefaultUser)
      ↓
GetRapidData()
      ↓
suscripción a eventos
```

La conexión utiliza:

```csharp
Controller.Connect(
    controllerInfo,
    ConnectionType.Standalone
);
```

y posteriormente:

```csharp
controller.Logon(
    UserInfo.DefaultUser
);
```



---

# 13. Reconexión automática

Una de las funciones principales del programa es recuperar la comunicación cuando el IRC5 deja temporalmente de responder.

Cuando PC SDK informa:

```text
ConnectionChanged
Connected = false
```

se inicia un hilo independiente de reconexión.

El proceso es:

```text
Pérdida de conexión
        ↓
ConnectionChanged = false
        ↓
Iniciar reconexión
        ↓
Nuevo NetScan
        ↓
Buscar controlador por ID
        ↓
Buscar controlador por IP
        ↓
Comprobar PING
        ↓
PING estable
        ↓
Controller.Connect()
        ↓
Logon()
        ↓
GetRapidData()
        ↓
suscribir eventos
        ↓
conexión restaurada
```

---

# 14. Identificación del controlador durante la reconexión

El programa guarda:

```text
Controller ID
IP
ControllerInfo
```

Por ejemplo:

```text
ID = 140-102229
IP = 192.168.125.1
```

Durante la reconexión no reutiliza únicamente el objeto `ControllerInfo` anterior.

Realiza nuevamente:

```text
NetworkScanner.Scan()
```

y busca el controlador mediante su:

1. ID
2. IP

Esto permite trabajar con un `ControllerInfo` actualizado.

---

# 15. Verificación mediante PING

Antes de intentar una nueva conexión PC SDK, el programa verifica que el controlador responda a ICMP.

Se requieren tres respuestas consecutivas:

```text
PING = True
PING = True
PING = True
```

Después se espera un segundo adicional antes de ejecutar:

```text
Controller.Connect()
```

La finalidad es evitar intentar la conexión exactamente durante el momento en que la interfaz de red del IRC5 acaba de reaparecer.

---

# 16. Diagnóstico de la conexión

El Bridge genera un archivo de registro automáticamente.

La carpeta es:

```text
logs/
```

y el archivo tiene un nombre similar a:

```text
bridge_diagnostic_20260920_152000.log
```

El registro incluye hora con milisegundos.

Ejemplo:

```text
[2026-09-20 15:20:10.125] [Nodo -> Bridge] Variable/SET/Comenzar/TRUE
[2026-09-20 15:20:10.132] [COMANDO] cmd escrito correctamente.
[2026-09-20 15:20:10.450] [RAPID EVENT] respuesta_robot.ValueChanged recibido.
[2026-09-20 15:20:10.451] [Robot -> Bridge] Variable/OK/Comenzar/TRUE
[2026-09-20 15:20:10.452] [Bridge -> Nodo] respuesta_robot enviada.
```

Esto permite comparar exactamente:

```text
comando recibido
        ↓
cmd escrito
        ↓
respuesta RAPID
        ↓
respuesta enviada por TCP
```

El sistema de logging y el archivo de diagnóstico se inicializan al arrancar el programa.

---

# 17. Diagnóstico de pérdidas de conexión

El programa también posee un watchdog de red que ejecuta periódicamente:

```text
PING 192.168.125.1
```

y registra cambios como:

```text
[WATCHDOG] El IRC5 dejó de responder a ICMP.
```

o:

```text
[WATCHDOG] El IRC5 vuelve a responder a ICMP.
```

Esto permite diferenciar entre:

```text
PC SDK perdió conexión
```

y:

```text
IRC5 dejó realmente de responder en la red
```

El watchdog se ejecuta en un hilo separado y realiza una comprobación cada segundo.

---

# 18. Ejecución del programa

Al ejecutar el Bridge:

```text
1. Se inicia el sistema de logging.
2. Se busca el controlador ABB.
3. Se selecciona el controlador real o virtual.
4. Se establece la conexión PC SDK.
5. Se realiza el login.
6. Se obtienen las variables RAPID.
7. Se suscriben los eventos.
8. Se inicia el watchdog.
9. Se inicia el servidor TCP.
10. El programa queda esperando al nodo.
```

Una ejecución normal debe mostrar algo similar a:

```text
[NETSCAN] Controladores encontrados: 1

[CONTROLADOR] Seleccionado
IP: 192.168.125.1
ID: 140-102229

[CONTROLADOR] Connected = True
[CONTROLADOR] Logon realizado.

[RAPID] cmd obtenido correctamente.
[RAPID] respuesta_robot obtenido correctamente.
[RAPID] app_conect obtenido correctamente.

[EVENTO] ConnectionChanged suscrito.
[EVENTO] respuesta_robot.ValueChanged suscrito.

[WATCHDOG] Monitor de red iniciado.

[TCP] Bridge escuchando en puerto 5000

[TCP] Esperando conexión del nodo...
```

---

# 19. Conexión desde ROS 2

El nodo ROS 2 debe conectarse como cliente TCP a la dirección IP del computador donde se ejecuta el Bridge.

Ejemplo:

```text
IP del PC: 192.168.125.X
Puerto:    5000
```

La arquitectura es:

```text
ROS 2
  │
  │ TCP client
  ▼
PC ejecutando AbbBrigeSocket
  │
  │ PC SDK
  ▼
IRC5
```

El nodo debe enviar cada comando terminado en `\n`.

Por ejemplo:

```text
Variable/SET/Comenzar/TRUE\n
```

y debe leer las respuestas también utilizando `\n` como delimitador.

---

# 20. Ejemplo de funcionamiento completo

### Paso 1 — conexión

```text
ROS 2
   ↓
TCP connect
   ↓
Bridge
   ↓
app_conect = TRUE
```

### Paso 2 — envío

El nodo envía:

```text
Variable/SET/Comenzar/TRUE
```

El Bridge recibe:

```text
[Nodo -> Bridge] Variable/SET/Comenzar/TRUE
```

y escribe:

```text
T_ROB1/Socket/cmd
```

### Paso 3 — procesamiento RAPID

RAPID detecta el cambio de `cmd`:

```text
cmd
 ↓
interrupción
 ↓
procesamiento del string
 ↓
Comenzar := TRUE
```

### Paso 4 — respuesta

RAPID modifica:

```text
respuesta_robot
```

Por ejemplo:

```text
Variable/OK/Comenzar/TRUE
```

El evento `ValueChanged` se ejecuta.

El Bridge envía:

```text
Variable/OK/Comenzar/TRUE\n
```

al nodo ROS 2.

---

# 21. Comandos y responsabilidades

Es importante separar las responsabilidades de cada componente.

## Nodo ROS 2

Responsable de:

```text
Generar comandos
Enviar comandos TCP
Recibir respuestas
Procesar respuestas
```

## Bridge C#

Responsable de:

```text
Comunicación TCP
Comunicación con PC SDK
Escritura de cmd
Lectura de respuesta_robot
Gestión de app_conect
Detección de pérdida de conexión
Reconexión del controlador
Diagnóstico
```

## RAPID

Responsable de:

```text
Interpretar cmd
Ejecutar interrupciones
Modificar variables
Ejecutar rutinas
Generar respuesta_robot
Controlar la lógica del robot
```

El Bridge **no debe incorporar la lógica específica de las órdenes RAPID**.

---

# 22. Estructura lógica del programa

Las principales funciones del Bridge son:

```text
Main()
│
├── SelectAndConnectController()
│
├── StartNetworkWatchdog()
│
├── StartTcpServer()
│
└── TcpServerLoop()
     │
     ├── HandleClient()
     │    └── InyectarComandoAlRobot()
     │
     └── CloseClient()
```

El canal de respuesta funciona mediante:

```text
respuesta_robot.ValueChanged
        │
        ▼
OnRobotVariablesChanged()
        │
        ▼
SendToClient()
```

La reconexión funciona mediante:

```text
ConnectionChanged
        │
        ▼
StartReconnectThread()
        │
        ▼
ReconnectToRealController()
        │
        ├── FindSelectedRealController()
        ├── WaitForStablePing()
        └── TryReconnectUsingInfo()
```

Estas funciones constituyen los principales bloques de operación del programa.

---

# 23. Consideraciones importantes

### Terminación de comandos

Los comandos deben terminar en:

```text
\n
```

porque el Bridge utiliza este carácter para determinar cuándo recibió un comando completo.

### Terminación de respuestas

Las respuestas transmitidas al nodo también terminan en:

```text
\n
```

### Variables RAPID

Deben existir exactamente en:

```text
T_ROB1/Socket/cmd
T_ROB1/Socket/respuesta_robot
T_ROB1/Socket/app_conect
```

### Conexión

El computador donde se ejecuta el Bridge debe tener conectividad con el IRC5 y el cliente TCP debe poder llegar al puerto:

```text
5000
```

### Reconexión

Cuando el IRC5 desaparece temporalmente de la red, el Bridge mantiene el proceso de búsqueda activo hasta volver a encontrar el controlador.

---

# 24. Resolución básica de problemas

## El Bridge no encuentra el IRC5

Verificar:

```text
IP del IRC5
Conexión Ethernet
NetScan
Configuración de red
```

El Bridge utiliza `NetworkScanner.Scan()` para descubrir el controlador.

---

## El cliente se conecta pero no se reciben comandos

Verificar que el nodo envíe:

```text
comando + "\n"
```

Ejemplo:

```text
Variable/SET/Comenzar/TRUE\n
```

También revisar el log:

```text
[TCP] Bytes recibidos
[TCP] Datos recibidos
[Nodo -> Bridge]
```

---

## RAPID recibe el comando pero el nodo no recibe respuesta

Verificar:

```text
respuesta_robot
```

y revisar que el log contenga:

```text
[RAPID EVENT] respuesta_robot.ValueChanged recibido.
```

Después debe aparecer:

```text
[Robot -> Bridge] ...
[Bridge -> Nodo] respuesta_robot enviada.
```

Si aparece el primer mensaje pero no el segundo, revisar la conexión TCP del cliente.

---

## El IRC5 desaparece temporalmente

Revisar:

```text
[CONTROLADOR] SE PERDIÓ LA CONEXIÓN.
```

junto con:

```text
[WATCHDOG] El IRC5 dejó de responder a ICMP.
```

y:

```text
[NETSCAN] El controlador NO aparece en NetScan.
```

Esto permite determinar si la pérdida afecta únicamente a PC SDK o también a la conectividad de red del IRC5.

---

# 25. Registro de diagnóstico recomendado

Cuando exista un problema de comunicación, guardar:

```text
bridge_diagnostic_*.log
```

y revisar especialmente la secuencia:

```text
[Nodo -> Bridge]
[COMANDO]
[RAPID EVENT]
[Robot -> Bridge]
[Bridge -> Nodo]
[CONTROLADOR]
[WATCHDOG]
[NETSCAN]
[RECONEXIÓN]
```

La relación temporal entre estos mensajes permite determinar en qué punto se interrumpe la comunicación.

---

# 26. Resumen

El ABB Bridge Socket implementa un canal de comunicación bidireccional:

```text
                  TCP/IP
              ┌────────────┐
              │  Nodo ROS 2 │
              └─────┬──────┘
                    │
                    ▼
            ┌───────────────┐
            │ ABB Bridge    │
            │     C#        │
            └──────┬────────┘
                   │
          ┌────────┴────────┐
          │                 │
          ▼                 ▼
        cmd          respuesta_robot
          │                 ▲
          ▼                 │
     RAPID / TRAP ──────────┘
```

El Bridge mantiene separadas las responsabilidades:

```text
TCP        → comunicación externa
PC SDK     → comunicación con IRC5
cmd        → entrada hacia RAPID
RAPID      → interpretación y ejecución
respuesta_robot → salida desde RAPID
app_conect → estado de conexión del cliente
NetScan/PING → diagnóstico y recuperación
```

La implementación actual también contempla reconexión automática cuando el controlador deja temporalmente de responder, utilizando un nuevo `NetScan`, validación por ID/IP, comprobación de PING, nueva conexión PC SDK, recuperación de las variables RAPID y nueva suscripción a los eventos.