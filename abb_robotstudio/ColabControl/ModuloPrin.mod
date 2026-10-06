MODULE ModuloPrin
    !***********************************************************
    !
    ! Modulo:  ModuloPrin
    !
    ! Descripci?n:
    !   Modulo de inicio para el programa con:
    !   - main: Inicio de la rutina, 
    !     inicializaci?n de varibles e interrupciones y proceso principal
    !   - Path_Home_Run: Rutina para llegar al punto de Home del robot
    !   - Path_Transicion: Rutina para punto que evita problemas de mecanismo en home
    !     y transici?n entre rutinas o puntos importantes
    !
    ! Autor: Juan_Daleman
    !
    ! Versi?n: 1.0
    !
    !***********************************************************
    
    !Puntos de espera y mantenimiento
    CONST robtarget Target_Mantenimiento:=[[524.308309139,712.143242458,575.732],[0.472131723,-0.518247718,0.524360327,-0.483277546],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Espera:=[[0,-407.163,1030.849],[0.5,0.5,0.5,-0.5],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    !Puntos para Home
    !CONST robtarget Target_Home_10:=[[802.284,0,778.685],[0.382683432,0,0.923879533,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Home_10:=[[329.136851055,0,730.080138956],[0.26115172,0,0.965297767,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Home_20:=[[788.429,0,593.609],[0.382683432,0,0.923879533,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    !Puntos referencia de posicion WO
    CONST robtarget Target_APuntosRef_10:=[[0,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_20:=[[0,0,0],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_30:=[[0,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_40:=[[0,-125,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_50:=[[0,-125,0],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_60:=[[0,-125,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_70:=[[125,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_80:=[[125,0,0],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_90:=[[125,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_100:=[[0,125,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_110:=[[0,125,0],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_120:=[[0,125,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_130:=[[-125,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_140:=[[-125,0,0],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_APuntosRef_150:=[[-125,0,-20],[0.707106781,0,0,0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    !Datos de Herramienta y Work Object
    PERS tooldata Prototipo_JSDaleman:=[TRUE,[[118.391,0,273.429],[0.923879533,0,0.382683432,0]],[0.152,[29.47,0,146.09],[1,0,0,0],0.001,0.001,0]];
    !TASK PERS tooldata ProtJSD:=[TRUE,[[120.954,2.88678,283.217],[1,0,0,0]],[0.2,[0,0,0],[1,0,0,0],0,0,0]];
    !Inicial
    !TASK PERS wobjdata WO_PasLabSir:=[FALSE,TRUE,"",[[470,0,305],[0,-0.707106781,0.707106781,0]],[[0,0,0],[1,0,0,0]]];
    !Datos del real
    !WO pruebas
    TASK PERS wobjdata WO_PasLabSir:=[FALSE,TRUE,"",[[472.990021071,1.407503889,411.995881601],[0.003191048,0.710976277,-0.703201352,0.00322633]],[[0,0,0],[1,0,0,0]]];
    TASK PERS wobjdata WO_Pas165:=[FALSE,TRUE,"",[[472.990021071,1.407503889,411.995881601],[0.003191048,0.710976277,-0.703201352,0.00322633]],[[0,0,0],[1,0,0,0]]];
    TASK PERS wobjdata WO_Pas25:=[FALSE,TRUE,"",[[472.990021071,601.407503889,411.995881601],[0.003191048,0.710976277,-0.703201352,0.00322633]],[[0,0,0],[1,0,0,0]]];
    !WO real
    TASK PERS wobjdata WO_PastelDef:=[FALSE,TRUE,"",[[473.27,1.4,412.09],[0.003191048,0.710976277,-0.703201352,0.00322633]],[[0,0,0],[1,0,0,0]]];
    !TASK PERS wobjdata WO_PastelDef_Real:=[FALSE,TRUE,"",[[447.012,-13.1784,423.553],[0.0419663,-0.711647,0.701237,-0.00803403]],[[0,0,0],[1,0,0,0]]];
    !TASK PERS wobjdata WO_PastelDef:=[FALSE,TRUE,"",[[447.012,-13.1784,423.553],[0.0419663,-0.711647,0.701237,-0.00803403]],[[0,0,0],[1,0,0,0]]];
    !Velocidades del proceso
    VAR speeddata Vel_Acercamiento:=v80;
    VAR speeddata Vel_Acercamiento_Lateral:=v30;
    VAR speeddata Vel_Vacio:=v100;
    VAR speeddata Vel_Vacio_Rap:=v150;
    VAR speeddata Vel_Proceso:=v60;
    VAR speeddata Vel_Segura:=v30;
    VAR num VelMax:=250;
    VAR num VelPor:=100;
    
    !Zona de acercamiento de proceso
    VAR zonedata Z_Acercamiento:=z1;
    VAR zonedata Z_VacioMov:=z10;
    VAR zonedata Z_Vacio_Rap:=z20;
    VAR zonedata Z_Proceso:=z1;
    VAR zonedata Z_Paro:=fine;
    VAR zonedata Z_Segura:=z1;
    
    !Variables de estado del socket
    PERS string Estado_Socket:="Aplicación en linea. Esperando comandos";
    PERS bool Conectar_Socket:=FALSE;
    PERS robtarget p_actual;
    PERS bool Postura_segura:=FALSE;
    
    !Variables condición de trayectoria
    PERS string path_contacto_actual;
    PERS string path_vacio_actual;
    PERS bool path_punto_en_contacto;
    PERS bool path_a_punto_en_contacto;
    PERS num index_contacto_origen;
    PERS num index_contacto_destino;
    PERS num index_vacio_origen;
    PERS num index_vacio_destino;
    PERS robtarget p_contacto_origen;
    PERS robtarget p_contacto_destino;
    PERS robtarget p_vacio_origen;
    PERS robtarget p_vacio_destino;
    VAR robtarget p_OrigenActual;
    VAR robtarget p_DestinoActual;
    PERS robtarget p_MoveActual;
    PERS robtarget p_objActual;
    VAR triggdata triggContacto;
    VAR triggdata triggContactoCir;
    VAR triggdata triggVacio;
    VAR num cercania:=0.001;
    
    !Variables de seleccion
    PERS string Seleccion:="LabSIR";
    
    !Variables de control de acciones
    PERS bool Bool_Home:=FALSE;
    PERS bool Bool_Transic:=FALSE;
    PERS bool Bool_Mant:=FALSE;
    PERS bool Bool_Esper:=FALSE;
    PERS bool Bool_RevWO:=FALSE;
    PERS bool Bool_Confirm:=FALSE;
    PERS bool Comenzar:=FALSE;
    PERS bool Parada_Emerg:=FALSE;
    PERS bool Paus_Continue:=FALSE;
    PERS bool Reiniciar:=FALSE;
    
    !Variables de control por socket
    PERS bool Sock_Parada:=FALSE;
    PERS bool Sock_Pausa:=FALSE;
    PERS bool Sock_Bloquear:=FALSE;
    PERS bool Sock_Reinicio:=FALSE;
    
    !Testigos de funcionamiento
    PERS bool Test_Transi:=FALSE;
    PERS bool Test_LabSIR:=TRUE;
    PERS bool Test_25:=FALSE;
    PERS bool Test_165:=FALSE;
    PERS bool Test_Reinicio:=FALSE;

    !***********************************************************
    !
    ! Procedimiento Main
    !
    !   Este es el punto de entrada de su programa
    !
    !***********************************************************
    
    PROC main()
        
        !Limpieza de variables
        Vel_Acercamiento:=v80;
        Vel_Acercamiento_Lateral:=v30;
        Vel_Vacio:=v100;
        Vel_Vacio_Rap:=v150;
        Vel_Proceso:=v60;
        Vel_Segura:=v30;
        Vel_Vacio_Rap.v_tcp:=120;
        VelMax:=250;
        VelPor:=100;
        Z_Acercamiento:=z1;
        Z_VacioMov:=z10;
        Z_Vacio_Rap:=z20;
        Z_Proceso:=z1;
        Z_Paro:=fine;
        Z_Segura:=fine;
        p_actual:=CRobT(\Tool:=Prototipo_JSDaleman \WObj:=wobj0);
        Seleccion:="Seleccione un diseño";
        Estado_Socket:=" ";
        cmd:="";
        respuesta_robot:=" ";
        app_conect:=FALSE;
        Conectar_Socket:=FALSE;
        Postura_segura:=FALSE;
        Sock_Parada:=FALSE;
        Sock_Pausa:=FALSE;
        Sock_Bloquear:=FALSE;
        Sock_Reinicio:=FALSE;
        Bool_Home:=FALSE;
        Bool_Transic:=FALSE;
        Bool_Mant:=FALSE;
        Bool_Esper:=FALSE;
        Bool_RevWO:=FALSE;
        Bool_Confirm:=FALSE;
        Comenzar:=FALSE;
        Parada_Emerg:=FALSE;
        Paus_Continue:=FALSE;
        Reiniciar:=FALSE;
        Test_Transi:=FALSE;
        Test_LabSIR:=FALSE;
        Test_25:=FALSE;
        Test_165:=FALSE;
        Test_Reinicio:=FALSE;
        IDelete DIHomeActiva;
        IDelete DIManteActiva;
        IDelete DIEsperaActiva;
        IDelete BoolHomeActiva;
        IDelete BoolTransActiva;
        IDelete BoolManteActiva;
        IDelete BoolEsperaActiva;
        IDelete BoolRevisionActiva;
        IDelete ParadaEmergencia;
        IDelete PausaContinuar;
        IDelete ReiniciarRut;
        IDelete SockParadaActiva;
        IDelete SockPausaActiva;
        IDelete SockBloqueoActiva;
        IDelete IncrementoIndicesContacto;
        IDelete IncrementoIndicesContactoCirculo;
        IDelete IncrementoIndicesVacio;
        IDelete socket_com_in;
        IDelete socket_com_out;
        
        VelSet VelPor, VelMax;
        
        ConfJ \On;
        ConfL \On;
        
        !Limpieza condiciones de path
        path_contacto_actual:="";
        path_vacio_actual:="";
        ReiniciarIndices;
        
        !Limpieza de salidas
        SetDO DO_01,0;
        SetDO DO_02,0;
        SetDO DO_03,0;
        
        !Limpieza de memoria
        ClearPath;
        StopMoveReset;
        
        !Interrupciones
        CONNECT DIHomeActiva WITH Path_Home_DI;
        CONNECT DIManteActiva WITH Path_Mantenimiento_DI;
        CONNECT DIEsperaActiva WITH Path_IntEspera_DI;
        CONNECT BoolHomeActiva WITH Path_Home;
        CONNECT BoolTransActiva WITH Path_Transicion_Home;
        CONNECT BoolManteActiva WITH Path_Mantenimiento;
        CONNECT BoolEsperaActiva WITH Path_IntEspera;
        CONNECT BoolRevisionActiva WITH Path_PuntosContacto_LabSir;
        CONNECT ParadaEmergencia WITH Path_Parada;
        CONNECT PausaContinuar WITH Path_PausCont;
        CONNECT ReiniciarRut WITH Path_Reinicio;
        CONNECT IncrementoIndicesContacto WITH IncIndicesContacto;
        CONNECT IncrementoIndicesContactoCirculo WITH IncIndicesContactoCir;
        CONNECT IncrementoIndicesVacio WITH IncIndicesVacio;
        
        !Interrupciones Socket
        CONNECT SockParadaActiva WITH SockParada;
        CONNECT SockPausaActiva WITH SockPausa;
        CONNECT SockBloqueoActiva WITH SockBloqueo;
        CONNECT socket_com_in WITH SockDataIn;
        CONNECT socket_com_out WITH SockDataOut;
        
        !Activadores interrupcion
        ISignalDI DI_01, 1, DIHomeActiva;
        ISignalDI DI_02, 1, DIManteActiva;
        ISignalDI DI_03, 1, DIEsperaActiva;
        IPers Bool_Home, BoolHomeActiva; 
        IPers Bool_Transic, BoolTransActiva;
        IPers Bool_Mant, BoolManteActiva;
        IPers Bool_Esper, BoolEsperaActiva;
        IPers Bool_RevWO, BoolRevisionActiva;
        IPers Parada_Emerg, ParadaEmergencia;
        IPers Paus_Continue, PausaContinuar;
        IPers Reiniciar, ReiniciarRut;
        
        !Activadores interrupcion socket
        IPers Sock_Parada, SockParadaActiva;
        IPers Sock_Pausa, SockPausaActiva;
        IPers Sock_Bloquear, SockBloqueoActiva;
        IPers cmd, socket_com_in;
        IPers respuesta_robot, socket_com_out;
        
        !Activador de llegadas a los puntos
        TriggInt triggContacto, cercania, IncrementoIndicesContacto;
        TriggInt triggContactoCir, cercania, IncrementoIndicesContactoCirculo;
        TriggInt triggVacio, cercania, IncrementoIndicesVacio;
        
        !Conexi?n socket TCP/IP
        Estado_Socket:="Esperando crear conexión socket";
        WaitUntil Conectar_Socket = TRUE;
        
        Estado_Socket:="Esperando aplicación socket";
        WaitUntil app_conect = TRUE;
        
        Estado_Socket:="Aplicación en linea. Esperando comandos";
        
        !Comienzo en punto de transicion principal
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        Test_Transi:=TRUE;
        p_actual := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=wobj0);
        
        Seleccion:="Seleccione un diseño";
        WHILE TRUE DO
          ! C?digo principal del programa
          TEST Seleccion
                CASE "LabSIR":
                    ! Si la selecci?n es tipo "LabSIR", Decora con LabSIR
                    Test_Transi:=FALSE;
                    Test_LabSIR:=TRUE;
                    Decorar_LABSIR WO_PastelDef;
                    Test_LabSIR:=FALSE;
                    Path_Espera;
                    Seleccion:="Seleccione un diseño";
                CASE "25 años mecatrónica":
                    ! Si la selecci?n es tipo "25 a?os mecatr?nica", Decora con 25 a?os mecatronica
                    Test_Transi:=FALSE;
                    Test_25:=TRUE;
                    Decorar_Meca25 WO_PastelDef;
                    Test_25:=FALSE;
                    Path_Espera;
                    Seleccion:="Seleccione un diseño";
                CASE "165 años ingeniería":
                    !  Si la selecci?n es tipo "165 a?os ingenier?a", Decora con 165 a?os facultad ingenieria
                    Test_Transi:=FALSE;
                    Test_165:=TRUE;
                    Decorar_Ing165 WO_PastelDef;
                    Test_165:=FALSE;
                    Path_Espera;
                    Seleccion:="Seleccione un diseño";
                DEFAULT:
                    ! Si no se tiene selecci?n se espera
                    StopMove;
                    WaitUntil Comenzar ;
                    StartMove;
            ENDTEST
        ENDWHILE
        
    ENDPROC
    PROC Path_Home_Run()
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
    ENDPROC
    PROC Path_Transicion()
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
    ENDPROC
    PROC Path_Espera()
        SetDO DO_01,0;
        SetDO DO_02,0;
        Test_Transi:=FALSE;
        MoveJ Target_Home_10,Vel_Vacio_Rap,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=wobj0;
        MoveJ Target_Espera,Vel_Vacio_Rap,Z_Paro,Prototipo_JSDaleman\WObj:=wobj0;
        SetDO DO_03,1;
        StopMove;
    ENDPROC
    PROC Path_contacto_ejecucion(robtarget PuntProc{*}, num index_inicial, num index_final, PERS wobjdata WO_PastelDef, bool Fin_contacto)
        IF index_final > Dim(PuntProc, 1) THEN
            RETURN;
        ENDIF
        p_objActual:=PuntProc{index_inicial};
        Mover_a_contacto;
        IF Fin_contacto THEN
            index_vacio_destino:=index_final;
        ENDIF
        TriggL p_objActual,Vel_Acercamiento,triggContacto,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
        IF (index_final-index_inicial) <> 1 THEN
            Mover_en_contacto;
            FOR index FROM index_inicial+1 TO index_final-1 DO
                p_objActual := PuntProc{index};
                TriggL p_objActual,Vel_Proceso,triggContacto,Z_Proceso,Prototipo_JSDaleman\WObj:=WO_PastelDef;
            ENDFOR
        ENDIF
        p_objActual:=PuntProc{index_final};
        IF Fin_contacto THEN
            Mover_a_vacio;
            TriggL p_objActual,Vel_Acercamiento,triggContacto,Z_Acercamiento,Prototipo_JSDaleman\WObj:=WO_PastelDef;
        ELSE
            Mover_en_contacto;
            TriggL p_objActual,Vel_Proceso,triggContacto,Z_Proceso,Prototipo_JSDaleman\WObj:=WO_PastelDef;
        ENDIF
    ENDPROC
    PROC Path_vacio_ejecucion(robtarget PuntProc{*}, num index, PERS wobjdata WO_PastelDef, bool Vel_rapida)
        IF index > Dim(PuntProc, 1) THEN
            RETURN;
        ENDIF
        p_objActual:=PuntProc{index};
        Mover_en_vacio;
        IF Vel_rapida THEN
            TriggJ p_objActual,Vel_Vacio_Rap,triggVacio,Z_Vacio_Rap,Prototipo_JSDaleman\WObj:=WO_PastelDef;
        ELSE
            TriggJ p_objActual,Vel_Vacio,triggVacio,Z_VacioMov,Prototipo_JSDaleman\WObj:=WO_PastelDef;
        ENDIF
    ENDPROC
    PROC ActualizarPuntosInteres()
        VAR robtarget path_aux;
        
        VAR num path_estado;
        
        IF path_contacto_actual = "" RETURN;
        IF path_Vacio_actual = "" RETURN;
        
        IF (NOT path_a_punto_en_contacto) AND (NOT path_punto_en_contacto) THEN
            path_estado:=1;
        ELSEIF (NOT path_a_punto_en_contacto) AND  path_punto_en_contacto THEN
            path_estado:=2;
        ELSEIF path_a_punto_en_contacto AND (NOT path_punto_en_contacto) THEN
            path_estado:=3;
        ELSEIF path_a_punto_en_contacto AND path_punto_en_contacto THEN
            path_estado:=4;
        ENDIF
        TEST path_estado
            CASE 1:
                GetDataVal path_contacto_actual, path_aux;
                IF (index_vacio_origen = 0) AND (index_vacio_destino = 0) THEN
                    p_OrigenActual:=CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                ELSE
                    p_OrigenActual:=Punto_Array(path_vacio_actual, index_vacio_origen);                  
                ENDIF
                IF Dim(path_aux,1) < index_contacto_destino THEN
                    p_OrigenActual:=Punto_Array(path_vacio_actual, index_vacio_destino);
                    p_DestinoActual:=CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                ELSE 
                    p_DestinoActual:=Punto_Array(path_vacio_actual, index_vacio_origen+1);
                ENDIF
            CASE 2:
                p_OrigenActual:=Punto_Array(path_vacio_actual, index_vacio_origen);
                p_DestinoActual:=Punto_Array(path_contacto_actual, index_contacto_destino);
            CASE 3:
                p_OrigenActual:=Punto_Array(path_contacto_actual, index_contacto_origen);
                p_DestinoActual:=Punto_Array(path_contacto_actual, index_contacto_destino);
            CASE 4:
                p_OrigenActual:=Punto_Array(path_contacto_actual, index_contacto_origen);
                p_DestinoActual:=Punto_Array(path_contacto_actual, index_vacio_destino);
        ENDTEST
        ERROR
            TEST ERRNO
                CASE ERR_INVDIM:
                    Estado_Socket:="Error de indices, no se ejecuto el comienzo de la trayectoria";
            ENDTEST
    ENDPROC
    PROC ReiniciarIndices()
        index_contacto_origen:=0;
        index_contacto_destino:=1;
        index_vacio_origen:=0;
        index_vacio_destino:=0;
    ENDPROC
    PROC Mover_en_vacio()
        path_punto_en_contacto:=FALSE;
        path_a_punto_en_contacto:=FALSE;
    ENDPROC
    PROC Mover_a_contacto()
        path_punto_en_contacto:=FALSE;
        path_a_punto_en_contacto:=TRUE;
    ENDPROC
    PROC Mover_a_vacio()
        path_punto_en_contacto:=TRUE;
        path_a_punto_en_contacto:=FALSE;
    ENDPROC
    PROC Mover_en_contacto()
        path_punto_en_contacto:=TRUE;
        path_a_punto_en_contacto:=TRUE;
    ENDPROC
    FUNC robtarget Punto_Array(string nombre_path, num indice_punto)
        TEST nombre_path
            CASE "p_contacto_labSIR":
                RETURN p_contacto_labSIR{indice_punto};
            CASE "p_vacio_labSIR":
                RETURN p_vacio_labSIR{indice_punto};
            CASE "p_contacto_labSIR":
                RETURN p_contacto_labSIR_FrenteL{indice_punto};
            CASE "p_vacio_labSIR":
                RETURN p_vacio_labSIR_FrenteL{indice_punto};
            CASE "p_contacto_labSIR":
                RETURN p_contacto_labSIR_FrenteR{indice_punto};
            CASE "p_vacio_labSIR":
                RETURN p_vacio_labSIR_FrenteR{indice_punto};
            CASE "p_contacto_165":
                RETURN p_contacto_165{indice_punto};
            CASE "p_vacio_165":
                RETURN p_vacio_165{indice_punto};
            CASE "p_contacto_25":
                RETURN p_contacto_25{indice_punto};
            CASE "p_vacio_25":
                RETURN p_vacio_25{indice_punto};
            DEFAULT:
                RETURN Target_Home_20;
        ENDTEST
    ENDFUNC
ENDMODULE