package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class skyAutoTesting extends OpMode {

    public skyHMAP robot = new skyHMAP();
    private double ly,lx, rx;
    public DcMotorEx odo;
    public void init()
    {
        odo = hardwareMap.get(DcMotorEx.class,"odo");
    }

    public void start(){
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void loop() {
        odo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("pos", odo.getCurrentPosition());
        telemetry.update();

    }
    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + 1*lx + rx,-1,1));
        robot.bl.setPower(Range.clip(ly - 1*lx + rx,-1,1));
        robot.fr.setPower(Range.clip(ly - 1*lx - rx,-1,1));
        robot.br.setPower(Range.clip(ly + 1*lx - rx,-1,1));
    }

}




