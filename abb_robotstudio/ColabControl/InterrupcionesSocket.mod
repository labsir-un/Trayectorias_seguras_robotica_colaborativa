MODULE InterrupcionesSocket
    !***********************************************************
    !
    ! Modulo:  InterrupcionesSocket
    !
    ! Descripcion:
    !   Interrupciones generadas desde el socket
    !
    ! Autor: Juan_Daleman
    !
    ! Versión: 1.0
    !
    !***********************************************************
    
    !Variables de interrupcion
    VAR intnum SockParadaActiva;
    VAR intnum SockPausaActiva;
    VAR intnum SockBloqueoActiva;
    
    !Interrupción de lectura del socket
    VAR intnum socket_com_in;
    VAR intnum socket_com_out;
    
    !Variables de mensajes de error para el usuario
     VAR btnres respuesta_parada_socket;
     CONST string mensaje_parada_socket{3} := ["Parada del sistema ejecutada desde el socket", "Es necesario realizar rearme de operaci�n", "Reinicie el sistema"];
    
    TRAP SockParada
        IDelete socket_com_in;
        IDisable;
        StopMove;
        ClearPath;
        StorePath;
        Sock_Parada:=FALSE;
        Test_LabSIR:=FALSE;
        Test_25:=FALSE;
        Test_165:=FALSE;
        UIMsgBox \Header:="Parada ejecutada",
            mensaje_parada_socket{1} \MsgLine2:=mensaje_parada_socket{2} \MsgLine3:=mensaje_parada_socket{3},
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
    TRAP SockPausa
        IDelete socket_com_in;
        IDisable;
        ISleep PausaContinuar;
        ISleep SockPausaActiva;
        StopMove;
        StorePath;
        Sock_Pausa:=FALSE;
        WaitUntil Paus_Continue=TRUE;
        Paus_Continue:=FALSE;
        RestoPath;
        StartMove;
        WaitTime 0.1;
        IWatch PausaContinuar;
        IWatch SockPausaActiva;
        IEnable;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
    ENDTRAP
    TRAP SockBloqueo
        IDisable;
        ISleep SockBloqueoActiva;
        StopMove;
        IF NOT Postura_segura THEN
            StorePath;
            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
        ENDIF
        Sock_Bloquear:=FALSE;
        WHILE NOT Sock_Reinicio DO
            RevisarSocket;
        ENDWHILE
        Sock_Reinicio:=FALSE;
        RestoPath;
        Postura_segura:=FALSE;
        StartMove;
        WaitTime 0.1;
        IWatch SockBloqueoActiva;
        IEnable;
    ENDTRAP
    TRAP SockDataIn
        IDelete socket_com_in;
        IDisable;
        RevisarSocket;
        CONNECT socket_com_in WITH SockDataIn;
        IPers cmd, socket_com_in;
        IEnable;
    ENDTRAP
    TRAP SockDataOut
         WaitTime tiempo_envio;
    ENDTRAP
ENDMODULE