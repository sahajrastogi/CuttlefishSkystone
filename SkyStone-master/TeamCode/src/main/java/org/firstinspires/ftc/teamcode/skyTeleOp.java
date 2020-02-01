package org.firstinspires.ftc.teamcode;

import android.widget.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class skyTeleOp extends OpMode {

    public skyHMAP robot = new skyHMAP();
    public double ly, lx, rx;
    public int liftPos;
    public int lift2Pos;
    public int clawPos = 0;
    public boolean fgrabbersUp = true;
    public boolean loopOver = true;
    public boolean grabbed = false;
    public boolean capstoneHeldUp = true;

    ButtonOp dpUp = new ButtonOp();
    ButtonOp dpDown = new ButtonOp();
    ButtonOp g2x = new ButtonOp();
    ButtonOp g2y = new ButtonOp();
    ButtonOp g2a = new ButtonOp();
    ButtonOp g1x = new ButtonOp();

    public void init() {
        robot.init(hardwareMap, false);
    }

    public void start() {
        telemetry.addData("Hello", 1);
    }

    public void loop() {

        //region update buttons
        dpUp.update(gamepad2.dpad_up);
        dpDown.update(gamepad2.dpad_down);
        g2x.update(gamepad2.x);
        g2y.update(gamepad2.y);
        g2a.update(gamepad2.a);
        g1x.update(gamepad1.x);
        //endregion

        //region intake
        robot.iL.setPower(gamepad2.right_stick_y);
        robot.iR.setPower(-gamepad2.right_stick_y);
        //endregion

        //region capstone
        if(g2a.onPress()){
            capstoneHeldUp = !capstoneHeldUp;
        }

        if(capstoneHeldUp){
            robot.cap.setPosition(0);
        } else {
            robot.cap.setPosition(1);
        }

        //endregion
        //region check lift
        if(robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
            if (robot.lift.getCurrentPosition() < liftPos) {
                robot.lift.setPower(1);
            }

            if (robot.lift.getCurrentPosition() > liftPos) {
                robot.lift.setPower(-1);
            }
        }

        if(robot.lift2.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            if(robot.lift2.getCurrentPosition() < lift2Pos){
                robot.lift2.setPower(1);
            }

            if(robot.lift2.getCurrentPosition() > lift2Pos){
                robot.lift2.setPower(-1);
            }
        }



        if (dpUp.onRelease()) {
            lift2Pos = robot.lift2.getCurrentPosition();
            robot.lift2.setTargetPosition(lift2Pos);
            robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftPos = robot.lift.getCurrentPosition();
            robot.lift.setTargetPosition(liftPos);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (dpDown.onRelease()) {
            lift2Pos = robot.lift2.getCurrentPosition();
            robot.lift2.setTargetPosition(lift2Pos);
            robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftPos = robot.lift.getCurrentPosition();
            robot.lift.setTargetPosition(liftPos);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(dpUp.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(gamepad2.right_trigger > 0.1){
                robot.lift.setPower(-0.1);
                robot.lift2.setPower(-0.1);
            } else {
                robot.lift.setPower(-1);
                robot.lift2.setPower(-1);
            }

        } else if(dpDown.isPressed()){
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(gamepad2.right_trigger > 0.1){
                robot.lift.setPower(0.1);
                robot.lift2.setPower(0.1);
            } else {
                robot.lift.setPower(1);
                robot.lift2.setPower(1);
            }
        }
        //endregion

        //region claw
        //X is Pressed
        if(g2x.onPress()) {
            //grab
            if (clawPos == 0 && loopOver) {
                robot.aL.setPosition(0.8);
                robot.aR.setPosition(0.17);
                clawPos = 1;
                loopOver = false;
            }
            //deposit
            if(clawPos == 1 && loopOver) {
                robot.aL.setPosition(0.1);
                robot.aR.setPosition(0.92);
                clawPos = 2;
                loopOver = false;
            }

            //intake
            if(clawPos == 2 && loopOver) {

                robot.aL.setPosition(0.685);
                robot.aR.setPosition(0.285);
                clawPos = 0;
                loopOver = false;

            }
        }

        loopOver = true;

        if(g2y.onPress()){
            grabbed = !grabbed;
        }

        if(grabbed){
            robot.gf.setPosition(0.7);
            robot.gs.setPosition(0.51);
        } else {
            robot.gf.setPosition(0.27);
            robot.gs.setPosition(0.92);
        }

            //endregion

        //region foundation grabbers

        if(g1x.onPress()){
            fgrabbersUp = !fgrabbersUp;
        }
        if(fgrabbersUp){
            robot.fgl.setPosition(0.6);
            robot.fgr.setPosition(0.4);
        } else{
            robot.fgl.setPosition(0.3);
            robot.fgr.setPosition(0.62);
        }
        //endregion

        //region drivetrain
        ly = -gamepad1.left_stick_y;
        lx = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        rx*=3;
        rx/=4;



        if(gamepad1.right_trigger > 0.1) {
            ly /= 2.5;
            rx /= 3;
            lx /= 2;
        }
        mecDrive(ly, lx, rx);
    //endregion

        //region telemetry
        telemetry.addData("clawPos",clawPos);
        telemetry.addData("lpos",robot.aL.getPosition());
        telemetry.addData("rpos",robot.aR.getPosition());
        telemetry.addData("pos",robot.lift.getCurrentPosition());
        telemetry.addData("mode",robot.lift.getMode());
        telemetry.addData("pos2",robot.lift2.getCurrentPosition());
        telemetry.addData("mode2",robot.lift2.getMode());
        telemetry.addData("vl",robot.vl.getCurrentPosition());
        telemetry.addData("vr",robot.vr.getCurrentPosition());
        telemetry.addData("h",robot.h.getCurrentPosition());
        telemetry.addData("grabbed",grabbed);
        //endregion

    }

    public void mecDrive(double ly, double lx, double rx) {
        robot.fl.setPower(Range.clip(ly + lx + rx, -1, 1));
        robot.bl.setPower(Range.clip(ly - lx + rx, -1, 1));
        robot.fr.setPower(Range.clip(ly - lx - rx, -1, 1));
        robot.br.setPower(Range.clip(ly + lx - rx, -1, 1));

    }



}
