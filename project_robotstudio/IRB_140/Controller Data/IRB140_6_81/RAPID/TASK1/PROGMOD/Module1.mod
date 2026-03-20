MODULE Module1
    CONST jointtarget ZERO:=[[0,0,0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P10:=[[450,0,450],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P20:=[[450,0,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P30:=[[550,0,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P40:=[[550,-100,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P50:=[[450,-100,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
   
   
    CONST robtarget P60:=[[475,-50,450],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P70:=[[475,-50,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
   
    CONST robtarget P80:=[[500,-75,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget P90:=[[525,-50,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
   
   
   
    CONST robtarget P100:=[[500,-25,400],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
   
   
    PROC main()
        MoveAbsJ ZERO\NoEOffs, v1000, fine, tool0;
        MoveJ P10, v1000, fine, tool0;
        MoveL P20, v1000, fine, tool0;
        MoveL P30, v1000, fine, tool0;
        MoveL P40, v1000, fine, tool0;
        MoveL P50, v1000, fine, tool0;
        MoveL P20, v1000, fine, tool0;
        MoveL P10, v1000, fine, tool0;
       
        MoveL P60, v1000, fine, tool0;
        MoveL P70, v1000, fine, tool0;
       
        MoveC P80, P90, v100, fine, tool0;
        MoveC P100, P70, v100, fine, tool0;
       
        MoveL P60, v1000, fine, tool0;
        MoveL P10, v1000, fine, tool0;
       
        MoveAbsJ ZERO\NoEOffs, v1000, fine, tool0;
    ENDPROC
ENDMODULE