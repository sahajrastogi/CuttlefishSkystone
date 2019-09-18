package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class pracTeleOp extends OpMode {

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;
    private Gamepad gamepad;

    double speed = -gamepad.left_stick_y;
    double strafe = gamepad.left_stick_x;
    double turn = gamepad.right_stick_x;


    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");


    }
    public void loop(){
        fl.setPower(Range.clip(speed+turn-strafe,-1,1));
        fr.setPower(Range.clip(speed-turn+strafe,-1,1));
        br.setPower(Range.clip(speed-turn-strafe,-1,1));
        bl.setPower(Range.clip(speed+turn+strafe,-1,1));

    }
}
