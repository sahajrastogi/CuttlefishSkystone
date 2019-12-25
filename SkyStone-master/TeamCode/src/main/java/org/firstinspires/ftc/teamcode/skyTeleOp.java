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
    private double ly, lx, rx;
    private int liftPos;
    public int clawPos = 0;
    private boolean xPressed;
    private boolean yPressed;

    ButtonOp dpUp = new ButtonOp();
    ButtonOp dpDown = new ButtonOp();
    ButtonOp g2x = new ButtonOp();
    ButtonOp g2y = new ButtonOp();
    ButtonOp g2a = new ButtonOp();

    int change  = 1;
    public void init() {
        robot.init(hardwareMap, false);
    }

    public void start() {
        telemetry.addData("Hello", 1);
    }

    public void loop() {
        updateButtons();
        setIntakePower();
        moveClaw();
        checkLift();
        driveTrain();

    }

    public void mecDrive(double ly, double lx, double rx) {
        robot.fl.setPower(Range.clip(ly + 1 * lx + rx, -1, 1));
        robot.bl.setPower(Range.clip(ly - 1 * lx + rx, -1, 1));
        robot.fr.setPower(Range.clip(ly - 1 * lx - rx, -1, 1));
        robot.br.setPower(Range.clip(ly + 1 * lx - rx, -1, 1));
    }

    public void checkLift() {
        if (robot.lift.getCurrentPosition() < liftPos) {
            robot.lift.setPower(1);
        }

        if (robot.lift.getCurrentPosition() > liftPos) {
            robot.lift.setPower(-1);
        }

        if (dpUp.onRelease()) {
            liftPos = robot.lift.getCurrentPosition();
            robot.lift.setTargetPosition(liftPos);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (dpDown.onRelease()) {
            liftPos = robot.lift.getCurrentPosition();
            robot.lift.setTargetPosition(liftPos);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void updateButtons() {
        dpUp.update(gamepad1.dpad_up);
        dpDown.update(gamepad1.dpad_down);
        g2x.update(gamepad2.x);
        g2y.update(gamepad2.y);

    }

    public void setIntakePower(){
        robot.iL.setPower(-gamepad2.left_stick_y);
        robot.iR.setPower(gamepad2.left_stick_y);
    }

    public void driveTrain(){
        ly = -gamepad1.left_stick_y;
        lx = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        rx /= 3;
        rx *= 2;

        if(gamepad1.right_trigger > 0.1) {
            ly /= 3;
            rx /= 3;

        }
        mecDrive(ly, lx, rx);
    }

    public void moveClaw(){

        //X is pressed
        if(g2x.onPress()) {
            if (clawPos == 180) {
                robot.aR.setPosition(0);
                robot.aL.setPosition(0);
                clawPos = 0;

            }
            if(clawPos == 270) {
                robot.aR.setPosition(0.5);
                robot.aL.setPosition(0.5);
                clawPos = 180;

            }
            if(clawPos == 0) {
                robot.aR.setPosition(0.5);
                robot.aL.setPosition(0.5);
                clawPos = 180;
            }
        }
        //Y is Pressed
        if(g2y.onPress()){
            if(clawPos==180){
                robot.aR.setPosition(0.66);
                robot.aL.setPosition(0.66);
                clawPos =270;
            }
            if(clawPos ==270){
                robot.aR.setPosition(0);
                robot.aL.setPosition(0);
                clawPos = 0;
            }
            if(clawPos ==0){
                robot.aR.setPosition(0.66);
                robot.aL.setPosition(0.66);
                clawPos = 270;
            }

        }

        if(g2a.onPress()&&robot.sC.getPosition()==1){
            robot.sC.setPosition(0);
        }
        else if(g2a.onPress()&&robot.sC.getPosition()==0){
            robot.sC.setPosition(1);
        }

    }

}
