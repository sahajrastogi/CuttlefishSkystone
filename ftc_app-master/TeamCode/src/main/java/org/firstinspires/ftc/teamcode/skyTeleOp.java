package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="teleOp")
public class skyTeleOp extends OpMode {
    skyHMAP robot = new skyHMAP();
    double ly = 0;
    double lx = 0;
    double rx = 0;

    boolean clawClosed = false;
    int teleClawPos = 0;

    int liftPos = 0;

    ButtonOp g2a  = new ButtonOp();
    ButtonOp g2x = new ButtonOp();
    ButtonOp g2y = new ButtonOp();
    ButtonOp g2b = new ButtonOp();
    ButtonOp g2dd = new ButtonOp();
    ButtonOp g2du = new ButtonOp();

    public void init()
    {
        robot.init(hardwareMap);
    }

    public void start(){
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPos = 0;
        robot.lift.setTargetPosition(liftPos);

        robot.teleopRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.teleopRotate.setTargetPosition(0);
        robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {
        g2a.update(gamepad2.a);
        g2x.update(gamepad2.x); // 180 pos
        g2y.update(gamepad2.y); // 270 pos
        g2b.update(gamepad2.b);
        g2du.update(gamepad2.dpad_up);
        g2dd.update(gamepad2.dpad_down);

        ly = -gamepad1.left_stick_y;
        lx = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        rx/=3;
        rx*=2;

        if(gamepad1.right_trigger > 0.2){
            ly /= 3;
        }

        mecDrive(ly, lx, rx);

           //region teleOpClaw
        if (g2x.onPress()){
            if(clawClosed) {
                clawClosed = false;
            }
            else {
                clawClosed = true;
            }
        }

        if(clawClosed){
            robot.teleopClawLeft.setPosition(0.5);
            robot.teleopClawRight.setPosition(0.6);
        }
        else{
            robot.teleopClawLeft.setPosition(0.6);
            robot.teleopClawRight.setPosition(0.4);
        }
        //endregion

        //region teleOpClawRotate
        if(g2a.onPress()) {
            if(teleClawPos == 0){
                teleClawPos = 180;
            } else if(teleClawPos == 180){
                teleClawPos = 0;
            } else if(teleClawPos == 270){
                teleClawPos = 180;
            }
        }

        if(g2b.onPress()){
            if(teleClawPos == 0){
                teleClawPos = 270;
            } else if(teleClawPos == 180){
                teleClawPos = 270;
            } else if(teleClawPos == 270){
                teleClawPos = 0;
            }
        }

        if(teleClawPos == 0){
            robot.teleopRotate.setTargetPosition(0);
        }
        else if(teleClawPos == 180){
            robot.teleopRotate.setTargetPosition(180);
        }
        else if(teleClawPos == 270){
            robot.teleopRotate.setTargetPosition(270);
        }

        if(robot.teleopRotate.getCurrentPosition() < robot.teleopRotate.getTargetPosition()){
            robot.teleopRotate.setPower(0.5);
        } else if(robot.teleopRotate.getCurrentPosition() < robot.teleopRotate.getTargetPosition()){
            robot.teleopRotate.setPower(-0.5);
        }
 
        //endregion

        //region lift
        if(g2dd.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(-1);
        } else if(g2du.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(1);
        } else{
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(g2dd.onRelease() || g2du.onRelease()){
                liftPos = robot.lift.getCurrentPosition();
            }
            robot.lift.setTargetPosition(liftPos);

            if(robot.lift.getCurrentPosition() < liftPos){
                robot.lift.setPower(1);
            } else if(robot.lift.getCurrentPosition() > liftPos){
                robot.lift.setPower(-1);
            }
        }
        //endregion

        //region intake
        robot.iL.setPower(gamepad2.right_stick_y);
        robot.iR.setPower(-gamepad2.right_stick_y);
        //endregion

        //region telemetry
        telemetry.addData("pos", robot.lift.getCurrentPosition());
        telemetry.addData("liftPos",liftPos);
        telemetry.addData("teleopRotatePos",robot.teleopRotate.getCurrentPosition());
        //endregion
    }

    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + 1*lx + rx,-1,1));
        robot.bl.setPower(Range.clip(ly - 0.35*lx + rx,-1,1));
        robot.fr.setPower(Range.clip(ly - 1*lx - rx,-1,1));
        robot.br.setPower(Range.clip(ly + 0.35*lx - rx,-1,1));
    }

}
