MODULE ComandosSocket
    !***********************************************************
    !
    ! Modulo:  ComandosSocket
    !
    ! Descripción:
    !   Comandos a ejecutar con los datos que llegan del socket
    !
    ! Autor: Juan_Daleman
    !
    ! Versión: 1.0
    !
    !***********************************************************
    VAR jointtarget JoinConf := [[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR robtarget Temp_p := [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    VAR num offx := 0;
    VAR num offy := 0;
    VAR num offz := 0;
    PERS robtarget p_salida;
    PERS robtarget p_salida_guardado;
    
    PROC EjecutarComando(string Prefijo, string ObjComando, string Variable, string Valor)
        TEST Prefijo
        CASE "Velocidad":
            ComandoVelociad ObjComando, Variable, Valor;
        CASE "Distancia":
            ComandoDistancia ObjComando, Variable, Valor;
        CASE "Variable":
            ComandoVariable ObjComando, Variable, Valor;
        DEFAULT:
            Estado_Socket:="Comando no identificado.";
            EnviarDatos Estado_Socket;
            RETURN;
        ENDTEST
    ENDPROC
    PROC ComandoVelociad(string ObjComando, string Variable, string Valor)
        VAR num t_num;
        VAR string Velocidad_encontrada;
        VAR datapos Bloque_memoria;
        VAR speeddata velocidad_temporal;
        
        TEST ObjComando
            CASE "Porcentaje":
                IF StrToVal(Valor, t_num) THEN
                    IF t_num > 100 THEN
                        t_num:=100;
                    ELSEIF t_num < 0 THEN
                        t_num:=0;
                    ENDIF
                    IF Variable = "VelMax" THEN
                        VelPor:=t_num;
                        VelSet VelPor, VelMax;
                        Estado_Socket:= "Porcentaje de velocidad actualiado a: " + ValToStr(t_num);
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ELSE
                        Estado_Socket:= "Error: Variable no soportada " + Variable;
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ENDIF
                ELSE
                    Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                    EnviarDatos Estado_Socket;
                    RETURN;
                ENDIF

            CASE "Max":
                IF StrToVal(Valor, t_num) THEN
                    IF t_num > 2000 THEN
                        t_num:=2000;
                    ELSEIF t_num < 100 THEN
                        t_num:=100;
                    ENDIF
                    IF Variable = "VelMax" THEN
                        VelMax:=t_num;
                        VelSet VelPor, VelMax;
                        Estado_Socket:= "Velocidad maxima actualiada a: " + ValToStr(t_num);
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ELSE
                        Estado_Socket:= "Error: Variaeble no soportada " + Variable;
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ENDIF
                ELSE
                    Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                    EnviarDatos Estado_Socket;
                    RETURN;
                ENDIF
            CASE "ACTVEL":
                SetDataSearch "speeddata" \Object:=Variable \PersSym \VarSym \InMOD:="ModuloPrin";
                IF GetNextSym(Velocidad_encontrada, Bloque_memoria) THEN
                    IF StrToVal(Valor, t_num) THEN
                        GetDataVal Velocidad_encontrada, velocidad_temporal;
                        velocidad_temporal.v_tcp := t_num;
                        SetDataVal Velocidad_encontrada \Block:=Bloque_memoria, velocidad_temporal;
                        GetDataVal Velocidad_encontrada, velocidad_temporal;
                        t_num:=velocidad_temporal.v_tcp;
                        Estado_Socket:= "Valor de velocidad " + Variable + " cambiado a: " + ValToStr(t_num);
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ELSE
                        Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ENDIF
                ENDIF
                Estado_Socket:="Error: no se encontro la velocidad " + Variable;
                EnviarDatos Estado_Socket;
            DEFAULT:
                Estado_Socket:="Objetivo de velocidad no identificado.";
                EnviarDatos Estado_Socket;
        ENDTEST
    ENDPROC
    PROC ComandoDistancia(string ObjComando, string Variable, string Valor)
        VAR datapos Bloque_memoria;
        VAR string punto_encontrado;
        VAR string t_str;
        VAR robtarget punto_temporal;
        VAR jointtarget t_JoinConf;
        
        VAR num anglex;
        VAR num angley;
        VAR num anglez;

        VAR string string_a_enviar := "";
        
        VAR pos t_pos;
        VAR bool ok;

        TEST ObjComando
            CASE "GetPostura":
                IF Variable = "Prototipo" THEN
                    TEST Valor
                        CASE "TCPEuler":
                            p_actual := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=wobj0);
                            anglex:=EulerZYX(\X, p_actual.rot);
                            angley:=EulerZYX(\Y, p_actual.rot);
                            anglez:=EulerZYX(\Z, p_actual.rot);
                                                   
                            string_a_enviar := "{Trans: {" 
                                                   + ValToStr(p_actual.trans.x) + "," 
                                                   + ValToStr(p_actual.trans.y) + "," 
                                                   + ValToStr(p_actual.trans.z) 
                                                   + "}, Rot: {"
                                                   + ValToStr(anglex) + "," 
                                                   + ValToStr(angley) + ","
                                                   + ValToStr(anglez) + "}}";
                            Estado_Socket:="TCPeul: ";
                            EnviarDatos Estado_Socket + string_a_enviar;
                            RETURN;
                        CASE "TCPCua":
                            p_actual := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=wobj0);
                                                   
                            string_a_enviar := "{Trans: {" 
                                                   + NumToStr(p_actual.trans.x, 3) + "," 
                                                   + NumToStr(p_actual.trans.y, 3) + "," 
                                                   + NumToStr(p_actual.trans.z, 3)
                                                   + "}, Rot: {"
                                                   + NumToStr(p_actual.rot.q1, 3) + "," 
                                                   + NumToStr(p_actual.rot.q2, 3) + ","
                                                   + NumToStr(p_actual.rot.q3, 3) + ","
                                                   + NumToStr(p_actual.rot.q4, 3) + "}}";
                            !Estado_Socket:="TCPcua: ";
                            Estado_Socket:="TCPcua: ";
                            EnviarDatos Estado_Socket + string_a_enviar;
                            RETURN;
                        DEFAULT:
                            Estado_Socket:="Error en solicitud de postura actual del TCP.";
                            EnviarDatos Estado_Socket;
                            RETURN;
                    ENDTEST
                    Estado_Socket:="Error en nombre del valor" + Valor;
                    EnviarDatos Estado_Socket;
                    RETURN;
                ENDIF
                Estado_Socket:="Error en nombre de la herramienta" + Variable;
                EnviarDatos Estado_Socket;
                RETURN;   
            CASE "CambiarTrayectoria":
                TEST Variable
                    CASE "ActualizarPuntos":
                        TEST Valor
                            CASE "Interes":
                                ActualizarPuntosInteres;
                                Estado_Socket:="Puntos de origen y destino actuzaliados";
                                EnviarDatos Estado_Socket;
                                RETURN;
                            CASE "GuardarPSalida":
                                p_salida_guardado := p_salida;
                                Estado_Socket:="Punto de salida guardado como p_salida_guardado de interes";
                                EnviarDatos Estado_Socket;
                                RETURN;
                        ENDTEST
                    CASE "SetOffsets":
                        ok := StrToVal(Valor, t_pos);
                        IF ok THEN
                            offx := t_pos.x;
                            offy := t_pos.y;  
                            offz := t_pos.z;
                            Estado_Socket:="Valores de offsets actualizados";
                            EnviarDatos Estado_Socket;
                            RETURN;
                        ENDIF
                        Estado_Socket:="Error en la configuracion del arreglo " + Variable;
                        RETURN;
                    CASE "MoveTemp":
                        TEST Valor
                            CASE "MoveJoint":
                                StopMove;
                                StorePath;
                                StartMove;
                                IF p_salida_guardado.rot.q1 = 0 AND p_salida_guardado.rot.q2 = 0 AND p_salida_guardado.rot.q3 = 0 AND p_salida_guardado.rot.q4 = 0 THEN
                                    Estado_Socket:="El p_salida_guardado no esta definido";
                                    EnviarDatos Estado_Socket;
                                    RETURN;
                                ENDIF
                                Temp_p := p_salida_guardado;
                                Temp_p := Offs(Temp_p, offx, offy, offz);
                                MoveJ Temp_p, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=WO_PastelDef;
                                RestoPath;
                                StopMove;
                                Estado_Socket:="El robot esta en: Destino modificado";
                                EnviarDatos Estado_Socket;
                                Sock_Bloquear:=TRUE;
                                Postura_segura:=TRUE;
                                RETURN;
                            CASE "MoveLinear":
                                StopMove;
                                StorePath;
                                StartMove;
                                IF p_salida_guardado.rot.q1 = 0 AND p_salida_guardado.rot.q2 = 0 AND p_salida_guardado.rot.q3 = 0 AND p_salida_guardado.rot.q4 = 0 THEN
                                    Estado_Socket:="El p_salida_guardado no esta definido";
                                    EnviarDatos Estado_Socket;
                                    RETURN;
                                ENDIF
                                Temp_p := p_salida_guardado;
                                Temp_p := Offs(Temp_p, offx, offy, offz);
                                MoveL Temp_p, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=WO_PastelDef;
                                RestoPath;
                                StopMove;
                                Estado_Socket:="El robot esta en: Destino modificado";
                                EnviarDatos Estado_Socket;
                                Sock_Bloquear:=TRUE;
                                Postura_segura:=TRUE;
                                RETURN;
                            DEFAULT:
                                Estado_Socket:="Error en la declaracion de " + Variable;
                        ENDTEST
                    DEFAULT:
                         Estado_Socket:="Error en objetivo del comando " + ObjComando;
                        RETURN;
                ENDTEST
            CASE "Postura_segura":
                TEST Variable
                    CASE "robtargetJ":
                        SetDataSearch "robtarget" \Object:=Valor \PersSym \VarSym \ConstSym \InMOD:="ModuloPrin";
                        IF GetNextSym(punto_encontrado, Bloque_memoria \Recursive) THEN
                            GetDataVal punto_encontrado, punto_temporal;
                            StopMove;
                            StorePath;
                            StartMove;
                            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                            MoveJ punto_temporal, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=wobj0;
                            RestoPath;
                            StopMove;
                            Estado_Socket:="Movimiento Join a: " + Valor;
                            EnviarDatos Estado_Socket;
                            Sock_Bloquear:=TRUE;
                            Postura_segura:=TRUE;
                            RETURN;
                        ENDIF
                        SetDataSearch "robtarget" \Object:=Valor \PersSym \ConstSym \InTask;
                        IF GetNextSym(punto_encontrado, Bloque_memoria \Recursive) THEN
                            GetDataVal punto_encontrado, punto_temporal;
                            StopMove;
                            StorePath;
                            StartMove;
                            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                            MoveJ punto_temporal, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=WO_PastelDef;
                            RestoPath;
                            StopMove;
                            Estado_Socket:="Movimiento Join a: " + Valor;
                            EnviarDatos Estado_Socket;
                            Sock_Bloquear:=TRUE;
                            Postura_segura:=TRUE;
                            RETURN;
                        ENDIF
                        Estado_Socket:="El punto " + Valor + " no fue encontrado.";
                        EnviarDatos Estado_Socket;
                        RETURN;
                    CASE "robtargetL":
                        SetDataSearch "robtarget" \Object:=Valor \PersSym \VarSym \ConstSym \InMOD:="ModuloPrin";
                        IF GetNextSym(punto_encontrado, Bloque_memoria \Recursive) THEN
                            GetDataVal punto_encontrado, punto_temporal;
                            StopMove;
                            StorePath;
                            StartMove;
                            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                            MoveL punto_temporal, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=wobj0;
                            RestoPath;
                            StopMove;
                            Estado_Socket:="Movimiento Lineal a: " + Valor;
                            EnviarDatos Estado_Socket;
                            Sock_Bloquear:=TRUE;
                            Postura_segura:=TRUE;
                            RETURN;
                        ENDIF
                        SetDataSearch "robtarget" \Object:=Valor \PersSym \ConstSym \InTask ;
                        IF GetNextSym(punto_encontrado, Bloque_memoria \Recursive) THEN
                            GetDataVal punto_encontrado, punto_temporal;
                            StopMove;
                            StorePath;
                            StartMove;
                            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                            MoveL punto_temporal, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=WO_PastelDef;
                            RestoPath;
                            StopMove;
                            Estado_Socket:="Movimiento Lineal a: " + Valor;
                            EnviarDatos Estado_Socket;
                            Sock_Bloquear:=TRUE;
                            Postura_segura:=TRUE;
                            RETURN;
                        ENDIF
                        Estado_Socket:="El punto " + Valor + " no fue encontrado.";
                        EnviarDatos Estado_Socket;
                        RETURN;
                    CASE "JoinConf":
                        IF StrToVal(Valor, t_JoinConf) THEN
                            StopMove;
                            StorePath;
                            StartMove;
                            p_salida := CRobT(\Tool:=Prototipo_JSDaleman \WObj:=WO_PastelDef);
                            MoveAbsJ t_JoinConf, Vel_Segura, Z_Segura, Prototipo_JSDaleman\WObj:=wobj0;
                            RestoPath;
                            StopMove;
                            Estado_Socket:="Movimiento de Joins a: " + Valor;
                            EnviarDatos Estado_Socket;
                            Sock_Bloquear:=TRUE;
                            Postura_segura:=TRUE;
                            RETURN;
                        ELSE
                            Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                            EnviarDatos Estado_Socket;
                            RETURN;
                        ENDIF
                    DEFAULT:
                        Estado_Socket:="Error: Tipo de variable '" + Variable + "' no soportado.";
                        EnviarDatos Estado_Socket;
                        RETURN;
                ENDTEST
            DEFAULT:
                Estado_Socket:="Objetivo de distancia no identificado.";
                EnviarDatos Estado_Socket;
                RETURN;
        ENDTEST
    ENDPROC
    PROC ComandoVariable(string ObjComando, string Variable, string Valor)
        VAR num t_num;
        VAR bool t_bool;
        VAR string t_string;
        VAR speeddata t_speed;
        VAR zonedata t_zone;
        VAR wobjdata t_wobj;
        VAR robtarget t_rob;
        VAR jointtarget t_joints;
        VAR num anglex;
        VAR num angley;
        VAR num anglez;
        
        VAR string Variable_encontrada;
        VAR datapos Bloque_memoria;
        VAR string tipo_detectado;
        
        VAR bool datos;
        
        VAR string string_a_enviar := "";
        VAR string string_a_enviar2 := " ";
            
        TEST ObjComando
            CASE "SET":
                
                SetDataSearch "bool" \Object:=Variable \PersSym \VarSym \InMOD:="ModuloPrin";
                IF GetNextSym(Variable_encontrada, Bloque_memoria) THEN
                    IF StrToVal(Valor, t_bool) THEN
                        SetDataVal Variable_encontrada \Block:=Bloque_memoria, t_bool;
                        GetDataVal Variable_encontrada, t_bool;
                        Estado_Socket:= Variable + ", valor cambiado a: " + ValToStr(t_bool);
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ELSE
                        Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ENDIF
                ENDIF
                
                SetDataSearch "num" \Object:=Variable \PersSym \VarSym \InMOD:="ModuloPrin";
                IF GetNextSym(Variable_encontrada, Bloque_memoria) THEN
                    IF StrToVal(Valor, t_num) THEN
                        SetDataVal Variable_encontrada \Block:=Bloque_memoria, t_num;
                        GetDataVal Variable_encontrada, t_num;
                        Estado_Socket:= Variable + ", valor cambiado a: " + ValToStr(t_num);
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ELSE
                        Estado_Socket:="Error: el valor " + Valor + " dado no se puede convertir correctamente";
                        EnviarDatos Estado_Socket;
                        RETURN;
                    ENDIF
                ENDIF
                
                SetDataSearch "string" \Object:=Variable \PersSym \VarSym \InMOD:="ModuloPrin";
                IF GetNextSym(Variable_encontrada, Bloque_memoria) THEN
                    SetDataVal Variable_encontrada \Block:=Bloque_memoria, Valor;
                    GetDataVal Variable_encontrada, t_string;
                    Estado_Socket:= Variable + ", valor cambiado a: " + t_string;
                    EnviarDatos Estado_Socket;
                    RETURN;
                ENDIF
                
                Estado_Socket:="Error: No se encontro ninguna variable con el nombre " + Variable;
                EnviarDatos Estado_Socket; 
                
            CASE "GET":
                TEST Valor
                    CASE "num":
                        GetDataVal Variable, t_num;
                        string_a_enviar := ValToStr(t_num);
                        
                    CASE "bool":
                        GetDataVal Variable, t_bool;
                        IF t_bool THEN
                            string_a_enviar := "TRUE";
                        ELSE
                            string_a_enviar := "FALSE";
                        ENDIF
                    
                    CASE "string":
                        GetDataVal Variable, t_string;
                        string_a_enviar := t_string;
                        
                    CASE "speeddata":
                        GetDataVal Variable, t_speed;
    
                        string_a_enviar := "{" + ValToStr(t_speed.v_tcp) + "," + ValToStr(t_speed.v_ori) + "}";
                        
                    CASE "zonedata":
                        GetDataVal Variable, t_zone;
    
                        IF t_zone.finep = TRUE THEN
                            string_a_enviar := "fine";
                        ELSE
                            string_a_enviar := ValToStr(t_zone.pzone_tcp);
                        ENDIF
                        
                    CASE "wobjdataEuler":
                        GetDataVal Variable, t_wobj;
                        
                        anglex:=EulerZYX(\X, t_wobj.uframe.rot);
                        angley:=EulerZYX(\Y, t_wobj.uframe.rot);
                        anglez:=EulerZYX(\Z, t_wobj.uframe.rot);
                        
                        string_a_enviar := "Pose: {Trans: {" 
                                               + ValToStr(t_wobj.uframe.trans.x) + "," 
                                               + ValToStr(t_wobj.uframe.trans.y) + "," 
                                               + ValToStr(t_wobj.uframe.trans.z) 
                                               + "}, Rot: {"
                                               + ValToStr(anglex) + "," 
                                               + ValToStr(angley) + ","
                                               + ValToStr(anglez) + "}}";
                    CASE "wobjdataCua":
                        GetDataVal Variable, t_wobj;
                        
                        string_a_enviar := "Pose: {Trans: {" 
                                               + ValToStr(t_wobj.uframe.trans.x) + "," 
                                               + ValToStr(t_wobj.uframe.trans.y) + "," 
                                               + ValToStr(t_wobj.uframe.trans.z) +
                                               "}, Rot: {"
                                               + ValToStr(t_wobj.uframe.rot.q1) + "," 
                                               + ValToStr(t_wobj.uframe.rot.q2) + ","
                                               + ValToStr(t_wobj.uframe.rot.q3) + ","
                                               + ValToStr(t_wobj.uframe.rot.q4) + "}}";
                                               
                    CASE "robtargetEuler":
                        GetDataVal Variable, t_rob;
                        
                        anglex:=EulerZYX(\X, t_rob.rot);
                        angley:=EulerZYX(\Y, t_rob.rot);
                        anglez:=EulerZYX(\Z, t_rob.rot);
                                               
                        string_a_enviar := "Pose: {Trans: {" 
                                               + ValToStr(t_rob.trans.x) + "," 
                                               + ValToStr(t_rob.trans.y) + "," 
                                               + ValToStr(t_rob.trans.z) 
                                               + "}, Rot: {"
                                               + ValToStr(anglex) + "," 
                                               + ValToStr(angley) + ","
                                               + ValToStr(anglez) + "}}";
                    CASE "robtargetCua":
                        GetDataVal Variable, t_rob;
                                               
                        string_a_enviar := "Pose: {Trans: {" 
                                               + ValToStr(t_rob.trans.x) + "," 
                                               + ValToStr(t_rob.trans.y) + "," 
                                               + ValToStr(t_rob.trans.z) + 
                                               "}, Rot: {"
                                               + NumToStr(t_rob.rot.q1, 3) + "," 
                                               + NumToStr(t_rob.rot.q2, 3) + ","
                                               + NumToStr(t_rob.rot.q3, 3) + ","
                                               + NumToStr(t_rob.rot.q4, 3) + "}}";
                     
                    CASE "joints":
                        t_joints := CJointT(\TaskName:=Variable) ;
                        string_a_enviar := NumToStr(t_joints.robax.rax_1, 4) + "," +
                             NumToStr(t_joints.robax.rax_2, 4) + "," +
                             NumToStr(t_joints.robax.rax_3, 4) + "," +
                             NumToStr(t_joints.robax.rax_4, 4) + "," +
                             NumToStr(t_joints.robax.rax_5, 4) + "," +
                             NumToStr(t_joints.robax.rax_6, 4);
                    CASE " ":
                        Estado_Socket:="Error en el mensaje de GET" + Variable + ", tipo de dato no declarado";
                        RETURN;
                    DEFAULT:
                        Estado_Socket:="Error: Tipo de dato '" + Valor + "' no soportado.";
                        EnviarDatos Estado_Socket;
                        RETURN;
                ENDTEST
                EnviarDatos Variable+" : "+string_a_enviar;
            DEFAULT:
                Estado_Socket:="Objetivo del comando no identificado.";
                EnviarDatos Estado_Socket;
        ENDTEST
        ERROR
        IF ERRNO = ERR_NAME_INVALID THEN
            Estado_Socket:="Error: Declaración del nombre la variable solicitada.";
            EnviarDatos Estado_Socket;
            RETURN;
        ENDIF
        IF ERRNO = ERR_SYM_ACCESS THEN
            Estado_Socket:="Error: Variable solicitada no encontrada.";
            EnviarDatos Estado_Socket;
            RETURN;
        ENDIF
        IF ERRNO = ERR_STRTOOLNG THEN
            EnviarDatos Variable + " : " +string_a_enviar;
            RETURN;
        ENDIF
    ENDPROC
ENDMODULE