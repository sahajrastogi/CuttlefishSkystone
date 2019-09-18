package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class pracTeleOp extends OpMode {

    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomRightMotor;
    private DcMotor bottomLeftMotor;
    private Gamepad gamepad;

    public void init(){
        topLeftMotor = hardwareMap.dcMotor.get("topLeftMotor;");
        topRightMotor = hardwareMap.dcMotor.get("topRightMotor");
        bottomRightMotor = hardwareMap.dcMotor.get("bottomRightMotor");
        bottomLeftMotor = hardwareMap.dcMotor.get("bottomLeftMotor");
    }

    public void loop(){

    }
}
