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

    boolean clawClosed = false;
    int teleClawPos = 0;


    ButtonOp g1a  = new ButtonOp();
    ButtonOp g1x = new ButtonOp();
    ButtonOp g1y = new ButtonOp();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop() {
        g1a.update(gamepad1.a);
        g1x.update(gamepad1.x); // 180 pos
        g1y.update(gamepad1.y); // 270 pos

        ly = gamepad1.left_stick_y;
        lx = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;


        mecDrive(ly, lx, rx);

        //region teleOpClaw
        if (g1a.onPress()){
            if(clawClosed) {
                clawClosed = false;
            }
            else {
                clawClosed = true;
            }
        }

        if(clawClosed){
            robot.teleopClaw.setPosition(1);
        }
        else{
            robot.teleopClaw.setPosition(0);
        }
        //endregion

        //region teleOpClawRotate
        if(g1x.onPress()) {
            if(teleClawPos == 0){
                teleClawPos = 180;
            } else if(teleClawPos == 180){
                teleClawPos = 0;
            } else if(teleClawPos == 270){
                teleClawPos = 180;
            }
        }

        if(g1y.onPress()){
            if(teleClawPos == 0){
                teleClawPos=270;
            } else if(teleClawPos == 180){
                teleClawPos = 270;
            } else if(teleClawPos == 270){
                teleClawPos =0;
            }
        }

        if(teleClawPos == 0){
            robot.teleopRotate.setPosition(0);
        }
        else if(teleClawPos == 180){
            robot.teleopRotate.setPosition(0.666);
        }
        else if(teleClawPos == 270){
            robot.teleopRotate.setPosition(1);
        }

        //endregion
//I can code very well. I'm the best coder on the team
    }

    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + lx + rx,1,-1));
        robot.bl.setPower(Range.clip(ly - lx + rx,1,-1));
        robot.fr.setPower(Range.clip(ly - lx - rx,1,-1));
        robot.br.setPower(Range.clip(ly + lx - rx,1,-1));
    }

}
