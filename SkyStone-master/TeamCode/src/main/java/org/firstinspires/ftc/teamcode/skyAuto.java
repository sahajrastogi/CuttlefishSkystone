package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class skyAuto extends LinearOpMode {

    skyHMAP robot;
    public String telVar;
    public skyAuto(skyHMAP hwmap){

        robot = hwmap;
    }
    public void runOpMode() throws InterruptedException{

    }
    //------------------------------------------------------------------------------------------------------------------------------
    //Driving Power Functions
    void stopDriving() {
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }


    //distance=rate*duration duration=distance/rate
    //power drives forward, -power drives backward
    void drive(double power) {
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
    }


    void rotateRight(double power) {
        robot.fl.setPower(-power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(power);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //Encoder Functions

    void slowAlign90() throws InterruptedException{

    }
    void stopAndReset() {
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void runUsing(){
        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void runToPosition(){
        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void driveDistance(double power, double distance) throws InterruptedException {
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int dist = (int)(distance*robot.ticksPerInch);


        robot.fl.setTargetPosition(dist);
        robot.fr.setTargetPosition(dist);
        robot.br.setTargetPosition(dist);
        robot.bl.setTargetPosition(dist);

        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        drive(power);


        while(robot.fl.isBusy()  ||robot.fr.isBusy() ||  robot.bl.isBusy() || robot.br.isBusy()){
            telemetry.addData("hi",robot.fl.isBusy());
        }

        stopDriving();
    }

    void moveForward(double power){
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(power);
        robot.bl.setPower(power);
    }


    public void gyroDrive ( double speed,
                            double distance,
                            double angle)  throws InterruptedException{

        int     newLeftTarget;
        int newLeftTargetB;
        int     newRightTarget;
        int newRightTargetB;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active

        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * robot.ticksPerInch);
        //newLeftTarget = robot.bl.getCurrentPosition() + moveCounts;
        newLeftTarget = robot.fl.getCurrentPosition() + moveCounts;
        newRightTarget = robot.fr.getCurrentPosition() + moveCounts;
        newLeftTargetB = robot.bl.getCurrentPosition() + moveCounts;
        newRightTargetB = robot.br.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        robot.bl.setTargetPosition(newLeftTarget);
        robot.fl.setTargetPosition(newLeftTarget);
        robot.br.setTargetPosition(newRightTarget);
        robot.fr.setTargetPosition(newRightTarget);

        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        speed = Range.clip(Math.abs(speed), -1, 1.0);
        robot.bl.setPower(speed);
        robot.fl.setPower(speed);
        robot.fr.setPower(speed);
        robot.br.setPower(speed);

        //
        telVar = Boolean.toString(robot.fl.isBusy());
        while (
                (robot.bl.isBusy() && robot.fr.isBusy() && robot.fl.isBusy() && robot.br.isBusy())) {
            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, 0.008);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed + steer;
            rightSpeed = speed - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.bl.setPower(leftSpeed);
            robot.fl.setPower(leftSpeed);
            robot.br.setPower(rightSpeed);
            robot.fr.setPower(rightSpeed);


        }

        // Stop all motion;
        robot.br.setPower(0);
        robot.fr.setPower(0);
        robot.fl.setPower(0);
        robot.bl.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
    }

    public double getHeading2(){
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.secondAngle));
    }

    public double getHeading3(){
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle));
    }

    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }

    public void gyroTurnRobotRightAbsolute(double angle, double power) {
        if(getHeading() <= 180 && getHeading() >= 180-angle-5){
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            while(normalize(getHeading()) > -(angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            while(getHeading() > -(angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }

    public void gyroTurnRobotLeftAbsolute(double angle, double power) {
        if(getHeading() <= 180 && getHeading() >= 180-angle-5){
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(normalize(getHeading()) > (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(getHeading() > (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }

    public void gyroTurnRobotRight(double angle, double power){
        double rightTurnHeading = getHeading();
        if(getHeading() <= 180 && getHeading() >= 180+angle+5){
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            robot.br.setPower(power);
            robot.fr.setPower(power);
            while(normalize(getHeading()) > normalize(rightTurnHeading) - (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            while(getHeading() > rightTurnHeading - (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }

    public void gyroTurnRobotLeft(double angle, double power){
        double rightTurnHeading = getHeading();
        if(getHeading() >= -180 && getHeading() <= -180+(angle+5)){
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(normalize(getHeading()) < normalize(rightTurnHeading) + (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(getHeading() < rightTurnHeading + (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }


    //insert methods
}