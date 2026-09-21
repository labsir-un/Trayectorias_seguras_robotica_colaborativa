MODULE Socket
    !***********************************************************
    !
    ! Modulo:  Socket
    !
    ! Descripcion:
    !   Comunicacion TCP/IP con para control del robot
    !
    ! Autor: Juan_Daleman
    !
    ! Versión: 1.0
    !
    !***********************************************************
    
    !Elementos socket
    PERS bool app_conect:=TRUE;
    CONST num tiempo_envio:=0.001;
    
    !Mensaje de control
    PERS string respuesta_robot:="Sock_Reinicio, valor cambiado a: TRUE"; 
    PERS string cmd := "";
    VAR string Prefijo:="";
    VAR string ObjComando:="";
    VAR string Variable:="";
    VAR string Valor:="";
    
    PROC LecturaComandos(string comando)
        VAR string str_aux;
        VAR string token_prefijo;
        VAR string token_ObjComando;
        VAR string token_Variable;
        VAR string token_Valor;
        VAR num pos_slash;
        VAR bool ok;
        
        str_aux:=comando;
        
        cmd := "";
        Prefijo:="";
        ObjComando:="";
        Variable:="";
        Valor:="";
        
        pos_slash := StrFind(str_aux, 1, "/");
        IF pos_slash = 0 OR pos_slash > StrLen(str_aux) THEN
            EnviarDatos "Se tiene solo prefijo";
            RETURN;
        ENDIF
        
        token_prefijo := StrPart(str_aux, 1, pos_slash - 1);
        str_aux := StrPart(str_aux, pos_slash + 1, StrLen(str_aux) - pos_slash);
            
        pos_slash := StrFind(str_aux, 1, "/");
        IF pos_slash = 0 OR pos_slash > StrLen(str_aux) THEN
            EnviarDatos "Se tiene solo comando";
            RETURN;
        ENDIF
        token_ObjComando := StrPart(str_aux, 1, pos_slash - 1);
        str_aux := StrPart(str_aux, pos_slash + 1, StrLen(str_aux) - pos_slash);
        
        pos_slash := StrFind(str_aux, 1, "/");
        IF pos_slash = 0 OR pos_slash > StrLen(str_aux) THEN
            EnviarDatos "Se tiene solo variable";
            RETURN;
        ENDIF
        token_Variable := StrPart(str_aux, 1, pos_slash - 1);
        str_aux := StrPart(str_aux, pos_slash + 1, StrLen(str_aux) - pos_slash);
        
        token_Valor := str_aux;
        IF token_Valor = "" THEN
            EnviarDatos "No se tiene valor para la variable";
            RETURN;
        ENDIF
        
        Prefijo:=token_prefijo;
        ObjComando:=token_ObjComando;
        Variable:=token_Variable;
        Valor:=token_Valor;
            
    ENDPROC
    PROC RevisarSocket()
        Estado_Socket:="Se esta revisando el comando";
        IF cmd <> "" THEN
          LecturaComandos cmd;
          IF Prefijo <> "" AND  ObjComando <> "" AND  Variable <> "" AND Valor <> "" THEN
            EjecutarComando Prefijo, ObjComando, Variable, Valor;
          ENDIF
        ENDIF
    ENDPROC
    PROC EnviarDatos(string Datos)
        respuesta_robot:=Datos;
    ENDPROC
ENDMODULE