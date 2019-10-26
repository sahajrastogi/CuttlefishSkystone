package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    boolean hi = false;

    int t2PrevPos = 0;
    int t2CurrPos = 0;
    boolean t2running = false;
    boolean cancel = false;


    boolean timer1bool = false;
    boolean controlled = false;
    int trPos =0;

    int pos0 = 10;
    int pos180 = -330;
    int pos270 = -650;

    double liftPow = 1.0;

    public void init()
    {
        robot.init(hardwareMap,false);
        timer2.startTime();
        timer3.startTime();
    }

    public void start(){
        timer1.startTime();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPos = 0;
        robot.lift.setTargetPosition(liftPos);

        robot.teleopRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.teleopRotate.setTargetPosition(0);
        robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.autoBigFlip.setPosition(0.6);
        robot.autoSmallFlip.setPosition(1);

    }

    public void loop() {
        g2a.update(gamepad2.a);
        g2x.update(gamepad2.x); // 180 pos
        g2y.update(gamepad2.y); // 270 pos
        g2b.update(gamepad2.b);
        g2du.update(gamepad2.dpad_up);
        g2dd.update(gamepad2.dpad_down);

        ly = -Math.pow(gamepad1.left_stick_y,3);
        lx = gamepad1.left_stick_x;
        rx = Math.pow(gamepad1.right_stick_x,3);
        rx/=3;
        rx*=2;

        if(gamepad1.right_trigger > 0.1){
            ly /= 3;
            rx/=3;
        }

        if(gamepad2.left_trigger > 0.1){
            rx/=3;
        }

        mecDrive(ly, lx, rx);

        //region teleOpClaw
        if (g2y.onPress()){
            if(clawClosed) {
                clawClosed = false;
            }
            else {
                clawClosed = true;
            }
        }//i'm the best coder on the team

        if (clawClosed) {
            robot.teleopClawLeft.setPosition(0.2);
            robot.teleopClawRight.setPosition(1);
        } else {
            robot.teleopClawLeft.setPosition(0.35);
            robot.teleopClawRight.setPosition(0.7);
        }
        //endregion


        //region teleOpClawRotate
        if(g2a.onPress()) {
            if(teleClawPos == 0){
                teleClawPos = 180;

                robot.teleopRotate.setTargetPosition(pos180);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(-0.15);
            } else if(teleClawPos == 180){
                teleClawPos = 0;

                timer1.reset();
                timer1bool = true;
                robot.teleopRotate.setTargetPosition(pos0);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(0.15);
            } else if(teleClawPos == 270){
                teleClawPos = 180;


                robot.teleopRotate.setTargetPosition(pos180);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(0.15);
            }
        }

        if(g2b.onPress()){
            if(teleClawPos == 0){
                teleClawPos = 270;

                robot.teleopRotate.setTargetPosition(pos270);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(-0.15);
            } else if(teleClawPos == 180){
                teleClawPos = 270;

                robot.teleopRotate.setTargetPosition(pos270);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(-0.15);
            } else if(teleClawPos == 270){
                teleClawPos = 0;

                timer1.reset();
                timer1bool = true;
                timer3.reset();
                hi = true;
                robot.teleopRotate.setTargetPosition(pos0);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.teleopRotate.setPower(0.15);
            }
        }

        if(timer1bool && timer1.seconds() > 2){
            timer1bool = false;
            robot.teleopRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.teleopRotate.setPower(0);
        }

        if(timer3.seconds() > 0.75 && hi){
            hi = false;
            clawClosed = true;
        }

        if(robot.teleopRotate.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
            robot.teleopRotate.setPower(gamepad2.right_stick_y*2 /3);
        }
 
        //endregion






        //region lift
        if(gamepad2.right_trigger>0.1){
            liftPow = 0.3;
        } else {
            liftPow = 1.0;
        }


        if(g2dd.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(-liftPow);
        } else if(g2du.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(liftPow);
        } else{
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(g2dd.onRelease() || g2du.onRelease()){
                liftPos = robot.lift.getCurrentPosition();
            }
            robot.lift.setTargetPosition(liftPos);

            if(robot.lift.getCurrentPosition() < liftPos){
                robot.lift.setPower(liftPow);
            } else if(robot.lift.getCurrentPosition() > liftPos){
                robot.lift.setPower(-liftPow);
            }
        }


        //endregion

        //region intake
        robot.iL.setPower(gamepad2.left_stick_y);
        robot.iR.setPower(-gamepad2.left_stick_y);
        //endregion

        //region telemetry
        telemetry.addData("pos", robot.lift.getCurrentPosition());
        telemetry.addData("liftPos",liftPos);
        telemetry.addData("teleopRotatePos",robot.teleopRotate.getCurrentPosition());
        telemetry.addData("clawClosed",clawClosed);
        telemetry.addData("teleClawPos",trPos);
        telemetry.addData("joystick",gamepad2.left_stick_y);
        telemetry.addData("mode",robot.teleopRotate.getMode());
        telemetry.addData("power", robot.teleopRotate.getPower());
        telemetry.addData("t2running",t2running);
        telemetry.addData("thing","thing");

                //endregion
    }

    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + 1*lx + rx,-1,1));
        robot.bl.setPower(Range.clip(ly - 0.35*lx + rx,-1,1));
        robot.fr.setPower(Range.clip(ly - 1*lx - rx,-1,1));
        robot.br.setPower(Range.clip(ly + 0.35*lx - rx,-1,1));
    }

}
