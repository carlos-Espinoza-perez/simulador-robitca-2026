MODULE TestSingularidad
    ! Herramienta por defecto
    PERS tooldata tool0 := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];

    ! Posición Segura 1: El robot está inclinado, J5 tiene un ángulo cómodo (~45 grados)
    CONST robtarget pSeguro := [[450, 0, 500], [0.92388, 0, 0.38268, 0], [0,0,0,0], [9E9,9E9,9E9,9E9,9E9,9E9]];
    
    ! Posición Segura 2: Al otro lado del espacio de trabajo
    CONST robtarget pDestino := [[450, 300, 500], [0.92388, 0, 0.38268, 0], [0,0,0,0], [9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Target Articular: Posición de Singularidad Pura (J5 = 0 exacto)
    CONST jointtarget jSingular := [[0, 30, -30, 0, 0, 0], [9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC main()
        ! 1. Iniciamos en una posición tranquila y segura
        MoveJ pSeguro, v500, fine, tool0;
        WaitTime 1;

        ! 2. Entramos a la boca del lobo (Forzamos Singularidad)
        ! Usamos MoveAbsJ para obligar a los motores a ir a J5 = 0.
        ! AQUÍ TU SCRIPT DE PYTHON DEBE GRITAR: "estado_general: singular"
        MoveAbsJ jSingular, v200, fine, tool0;
        WaitTime 1;

        ! 3. La trampa mortal: Movimiento Lineal desde una singularidad
        ! En RobotStudio real, intentar un MoveL estando con J5=0 suele lanzar 
        ! el error "Joint 4/6 Velocity Too High" porque el Jacobiano se vuelve loco.
        MoveL pDestino, v100, fine, tool0;
        
    ENDPROC
ENDMODULE