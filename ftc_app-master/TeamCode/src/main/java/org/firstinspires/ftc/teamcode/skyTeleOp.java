package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class skyTeleOp extends OpMode {
    skyHMAP robot = new skyHMAP();
    double ly = 0;
    double lx = 0;
    double rx = 0;

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        ly = gamepad1.left_stick_y;
        lx = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        mecDrive(ly,lx,rx);


    }

    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + lx + rx,1,-1));
        robot.bl.setPower(Range.clip(ly - lx + rx,1,-1));
        robot.fr.setPower(Range.clip(ly - lx - rx,1,-1));
        robot.br.setPower(Range.clip(ly + lx - rx,1,-1));
    }

}
