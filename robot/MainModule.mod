MODULE MainModule
	TASK PERS tooldata tgriper:=[TRUE,[[-88.6725,3.47772,183.741],[1,0,0,0]],[0.5,[50,0,50],[1,0,0,0],0,0,0]];
	TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[675.582,281.126,-4.86696],[0.307157,-0.00419946,-0.000487573,-0.951649]],[[0,0,0],[1,0,0,0]]];
	TASK PERS wobjdata wobj2:=[FALSE,TRUE,"",[[542.476,-331.342,5.34818],[0.999989,0.0041986,-0.00140461,-0.00182885]],[[0,0,0],[1,0,0,0]]];
	CONST robtarget pcircle:=[[31.83,36.09,5.68],[0.00826435,0.0143333,-0.999821,-0.00916285],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget pcircle10:=[[0.00,204.97,0.00],[0.0283605,-0.0827347,-0.996054,-0.0150531],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget psquare:=[[41.06,163.80,6.46],[0.0286482,0.0144128,-0.999416,-0.0117646],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget psquare10:=[[0.00,204.97,0.00],[0.0283596,-0.0827333,-0.996054,-0.0150527],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget ptriangle:=[[148.10,155.78,6.47],[0.0286464,0.014411,-0.999416,-0.0117655],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget ptriangle10:=[[383.71,186.51,10.68],[0.032109,0.76001,-0.648968,0.0139697],[-1,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget pbase:=[[-6.82,-3.91,6.65],[0.0239716,0.955113,-0.294796,0.0167312],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget pbase10:=[[-3.09,0.33,1.16],[0.0328321,0.794913,-0.605712,0.0121952],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget phome:=[[314.97,104.34,215.83],[0.0239848,0.955119,-0.294775,0.0167204],[0,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget placeTarget;! Will receive full robtarget string
    VAR string client_msg;
    VAR string client_message;
    VAR string server_msg;
    VAR string server_message;
    VAR num sel;
    VAR num cont;
    !VAR pos pickTarget;
    VAR bool pickTarget2;
    VAR robtarget pickTarget; 
    VAR pos pickPos;
    VAR num x_val; 
    VAR num y_val; 
    VAR num a_val; 
    VAR string coord_array{3}; 
    VAR num nSelection; 
    VAR num nobjSelection; 
    VAR num pick_rot_z := 0;
    VAR num pos1;
    VAR num pos2;
     VAR robtarget pick_rotated;
    VAR orient place_orient;
    VAR num euler_x;
    VAR num euler_y;
    VAR num euler_z;
    CONST robtarget phome10:=[[0.00,95.55,0.00],[0.0182535,0.95529,-0.295103,-0.00156628],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget phome20:=[[314.98,104.35,215.83],[0.0239843,0.955117,-0.294784,0.0167225],[0,0,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget phome30:=[[0.00,95.83,0.00],[0.0209588,0.955173,-0.294849,0.0164167],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

	PROC main()
    ! Connect to Server
    RobotAsClientConnect;
    WHILE nobjSelection <> 2 DO
        SetDO doYellowLamp,1;
        TPWrite "Yellow button pressed. Waiting for object selection...";
        TPWrite "Enter 1 for Circle, 2 for Square, 3 for Triangle";
        TPReadNum nSelection, "Select object (1-3):";
        IF nSelection=1 THEN
            client_message:="Circle";
            placeTarget:=pcircle;
        ELSEIF nSelection=2 THEN
            client_message:="Square";
            placeTarget:=psquare;
        ELSEIF nSelection=3 THEN
            client_message:="Triangle";
            placeTarget:=ptriangle;
        ELSE
            client_message:="";
        ENDIF
        RobotClienSendMessage(client_message);
        ! Handle answer from server
        Server_message:=RobotClientReciveMessage();
        WaitDI diOkButton,1;
        SetDO doYellowLamp,0;
        TPWrite "SERVER SAYS: "+Server_message;
        ! Check if quit command is received
        IF Server_message = "quit" THEN
            RobotClientCloseAndDisconnect;
            EXIT;
        ENDIF
        ! Parse the received message for coordinates
        ParseCoordinates Server_message;
        IF pickPos.x <> 0 AND pickPos.y <> 0 THEN
            PickPlace;
        ELSE
            TPWrite "Invalid coordinates received.";
        ENDIF
    ENDWHILE
    client_message:="quit";
    RobotClienSendMessage(client_message);
    RobotClientCloseAndDisconnect;
  
ENDPROC

PROC ParseCoordinates(string message)
    ! Initialize pickPos and pick_rot_z
    pickPos := [0, 0, 0];
    pick_rot_z := 0;
    ! Split the message into an array of strings
    pos1 := StrFind(message, 1, ",");
    TPWrite "pos1=" + NumToStr(pos1, 0);
    pos2 := StrFind(message, pos1+1, ",");
    TPWrite "pos2=" + NumToStr(pos2, 0);
    IF pos1 > 0 AND pos2 > pos1 THEN
        coord_array{1} := StrPart(message, 1, pos1-1);              ! x_val
        coord_array{2} := StrPart(message, pos1+1, pos2-pos1-1);     ! y_val
        coord_array{3} := StrPart(message, pos2+1, StrLen(message)-pos2); ! a_val
        ! Convert strings to numbers
        IF StrToVal(coord_array{1}, x_val) AND StrToVal(coord_array{2}, y_val) AND StrToVal(coord_array{3}, a_val) THEN
            pickPos.x := x_val;
            pickPos.y := y_val;
            pick_rot_z := a_val;
            TPWrite "PARSED: x=" + NumToStr(x_val, 0) + ", y=" + NumToStr(y_val, 0) + ", a=" + NumToStr(pick_rot_z, 0);
        ELSE
            TPWrite "Error converting coordinates to numbers.";
        ENDIF
    ELSE
        TPWrite "Error: Invalid message format.";
    ENDIF
ENDPROC

PROC PickPlace()
     MoveL Offs(pbase, pickPos.x, pickPos.y, 10), v500, fine, tgriper\WObj:=wobj1;
    ! MoveL pbase, v500, fine, tgriper\WObj:=wobj1;
    MoveL Offs(pbase, pickPos.x, pickPos.y, 0), v500, fine, tgriper\WObj:=wobj1;
    WaitTime 3;
    Set doValve1;
    WaitTime 1;
    ! Go home
    MoveL phome, v1000, z50, tgriper\WObj:=wobj1;
    ! Get Euler angles from placeTarget quaternion
    euler_x := EulerZYX(\X, placeTarget.rot);
    euler_y := EulerZYX(\Y, placeTarget.rot);
    euler_z := EulerZYX(\Z, placeTarget.rot);
    ! Add parsed angle to Z Euler angle
    euler_z := euler_z + pick_rot_z;
    ! Rebuild quaternion for placeTarget
    place_orient := OrientZYX(euler_z, euler_y, euler_x);
    placeTarget.rot := place_orient;
    ! Place with updated orientation
    MoveL Offs(placeTarget, 0, 0, 20), v1000, z50, tgriper\WObj:=wobj2;
    MoveL placeTarget, v500, fine, tgriper\WObj:=wobj2;
    WaitTime 1;
    Reset doValve1;
    WaitTime 1;
    ! Retreat
    MoveL Offs(placeTarget, 0, 0, 20), v1000, z50, tgriper\WObj:=wobj2;
    MoveL phome, v1000, z50, tgriper\WObj:=wobj1;
    !MoveL pbase, v1000, z50, tgriper\WObj:=wobj1;
   
ENDPROC

ENDMODULE
