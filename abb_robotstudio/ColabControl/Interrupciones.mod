MODULE Interrupciones
    
    !***********************************************************
    !
    ! Modulo:  Interrupciones
    !
    ! Descripción:
    !   Modulo de manejo de interrupciones para manejo de la HMI
    !
    ! Autor: Juan_Daleman
    !
    ! Versión: 1.0
    !
    !***********************************************************
    
     !Variables de interrupcion
     VAR intnum DIHomeActiva;
     VAR intnum DIManteActiva;
     VAR intnum DIEsperaActiva;
     VAR intnum BoolHomeActiva;
     VAR intnum BoolTransActiva;
     VAR intnum BoolManteActiva;
     VAR intnum BoolEsperaActiva;
     VAR intnum BoolRevisionActiva;
     VAR intnum ParadaEmergencia;
     VAR intnum PausaContinuar;
     VAR intnum ReiniciarRut;
     VAR intnum IncrementoIndicesContacto;
     VAR intnum IncrementoIndicesContactoCirculo;
     VAR intnum IncrementoIndicesVacio;
     
     !Variables de mensajes de error para el usuario
     VAR btnres respuesta_parada;
     CONST string mensaje_parada{3} := ["Parada del sistema ejecutada", "Es necesario realizar rearme de operación", "Reinicie el sistema"];
     
     VAR btnres respuesta_reinicio;
     CONST string mensaje_reinicio{3} := ["Reinicio del sistema ejecutado", "Se reiniciara todo el sistema", "es necesario reiniciar el sistema de supervisión"];
     
    TRAP Path_Home_DI
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_02,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
        MoveJ Target_Home_20,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        RestoPath;
        StopMove;
        SetDO DO_01,1;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Home
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_02,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        IF Bool_Home = TRUE THEN
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_Home_20,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
            SetDO DO_01,1;
        ENDIF
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Transicion_Home
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_01,0;
        SetDO DO_02,0;
        SetDO DO_03,0;
        StartMove;
        IF Bool_Transic = TRUE THEN
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
            Test_Transi:=TRUE;
        ENDIF
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Mantenimiento_DI
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_01,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
        MoveJ Target_Mantenimiento,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        SetDO DO_02,1;
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Mantenimiento
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_01,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        IF Bool_Mant = TRUE THEN
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_Mantenimiento,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
            SetDO DO_02,1;
        ENDIF
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_IntEspera_DI
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_01,0;
        SetDO DO_02,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
        MoveJ Target_Espera,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        SetDO DO_03,1;
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_IntEspera
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        SetDO DO_01,0;
        SetDO DO_02,0;
        SetDO DO_03,0;
        Test_Transi:=FALSE;
        StartMove;
        IF Bool_Esper = TRUE THEN
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_Espera,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        ENDIF
        SetDO DO_03,1;
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_PuntosContacto_LabSir
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        Test_Transi:=FALSE;
        StartMove;
        IF Bool_RevWO = TRUE THEN
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_APuntosRef_10,Vel_Vacio_Rap,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_20,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_30,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_APuntosRef_40,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_50,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_60,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_APuntosRef_70,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_80,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_90,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_APuntosRef_100,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_110,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_120,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_APuntosRef_130,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_140,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_APuntosRef_150,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_RobLab_10,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_RobLab_20,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_RobLab_260,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_Frente_10,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Frente_20,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_DibChip_10,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_DibChip_20,Vel_Proceso,Z_Proceso,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_DibChip_510,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveJ Target_Frente_20,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Frente_10,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
            MoveJ Target_Frente_30,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Frente_40,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_DibEng_10,Vel_Vacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_DibEng_20,Vel_Proceso,Z_Proceso,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveL Target_DibEng_790,Vel_Acercamiento,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            WaitUntil Bool_Confirm=TRUE;
            WaitUntil Bool_Confirm=FALSE;
            MoveJ Target_Frente_40,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Frente_30,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
            Test_Transi:=TRUE;
        ENDIF
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Parada
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        Test_LabSIR:=FALSE;
        Test_25:=FALSE;
        Test_165:=FALSE;
        UIMsgBox \Header:="Parada ejecutada",
            mensaje_parada{1} \MsgLine2:=mensaje_parada{2} \MsgLine3:=mensaje_parada{3},
            \Buttons:=btnOK
            \Icon:=iconWarning 
            \Result:=respuesta_parada;
        IWatch ReiniciarRut;
        WHILE Reiniciar <> TRUE DO
            Test_Reinicio:=TRUE;
            WaitTime 0.005;
            Test_Reinicio:=FALSE;
            WaitTime 0.005;
        ENDWHILE
        RestoPath;
        StopMove;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_PausCont
        IDelete socket_com_in;
        IDisable;
        ISleep PausaContinuar;
        StopMove;
        StorePath;
        WaitUntil Paus_Continue=TRUE;
        Paus_Continue:=FALSE;
        RestoPath;
        StartMove;
        WaitTime 0.1;
        IWatch PausaContinuar;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP Path_Reinicio
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        Test_LabSIR:=FALSE;
        Test_25:=FALSE;
        Test_165:=FALSE;
        Test_Reinicio:=TRUE;
        UIMsgBox \Header:="Reinicio ejecutado",
            mensaje_reinicio{1} \MsgLine2:=mensaje_reinicio{2} \MsgLine3:=mensaje_reinicio{3},
            \Buttons:=btnOK
            \Icon:=iconWarning 
            \Result:=respuesta_parada;
        StartMove;
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
        MoveJ Target_Espera,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        Test_Reinicio:=FALSE;
        SetDO DO_03,1;
        RestoPath;
        StopMove;
        IEnable;
        ExitCycle; 
    ENDTRAP
    TRAP IncIndicesContacto
        Incr index_contacto_origen;
        Incr index_contacto_destino;
    ENDTRAP
    TRAP IncIndicesContactoCir
        Incr index_contacto_origen;
        Incr index_contacto_origen;
        Incr index_contacto_destino;
        Incr index_contacto_destino;
    ENDTRAP
    TRAP IncIndicesVacio
        Incr index_vacio_origen;
    ENDTRAP
ENDMODULE