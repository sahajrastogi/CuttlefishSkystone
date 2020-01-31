package org.firstinspires.ftc.teamcode.auto.autoFunctions;

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
import org.firstinspires.ftc.teamcode.skyHMAP;

import java.util.ArrayList;
import java.util.Locale;

public class skyAuto extends LinearOpMode {

    skyHMAP robot;
    public static double currFollowRadius = 7;
    public static String telemetryString1 = "a";
    public static String telemetryString2 = "a";
    public static boolean currEndPointIsStopPoint = false;
    public static double currFollowAngle = 90;
    public static boolean loopIsOver = false;
    public static double distToEndPoint = 80;
    public static Point endPoint = new Point(0,0);
    public static double stopRadius = 4;
    public double max = 0;

    public skyAuto(skyHMAP hwmap){

        robot = hwmap;
    }
    public void runOpMode() throws InterruptedException{

    }

    public void followCurve(ArrayList<CurvePoint> allPoints, Pos robotPos){
        CurvePoint followMe = getFollowPointPath(allPoints, robotPos, currFollowRadius);
        goToPoint(robotPos, new Pos(followMe.x, followMe.y,currFollowAngle),followMe.moveSpeed,followMe.turnSpeed);

    }

    public void followCurveReverse(ArrayList<CurvePoint> allPoints, Pos robotPos){
        CurvePoint followMe = getFollowPointPathReverse(allPoints, robotPos, currFollowRadius);
        goToPoint(robotPos, new Pos(followMe.x, followMe.y,currFollowAngle),followMe.moveSpeed,followMe.turnSpeed);
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pos robotPos, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i=0;i<pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunc.lineCircleIntersection(new Point(robotPos.x, robotPos.y), followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000;
            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - robotPos.y, thisIntersection.x - robotPos.x);
                double deltaAngle = Math.abs(MathFunc.AngleWrap(angle - robotPos.theta));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe = new CurvePoint(startLine);
                    currFollowRadius = startLine.followDistance;
                    currFollowAngle = startLine.followAngle;
                    currEndPointIsStopPoint = endLine.isStopPoint;
                    distToEndPoint = Math.hypot(robotPos.x - endLine.x,robotPos.y-endLine.y);
                    endPoint.x = endLine.x;
                    endPoint.y = endLine.y;
                    followMe.setPoint(thisIntersection);
                }

            }
        }

        return followMe;
    }

    public static CurvePoint getFollowPointPathReverse(ArrayList<CurvePoint> pathPoints, Pos robotPos, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i=0;i<pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunc.lineCircleIntersection(new Point(robotPos.x, robotPos.y), followRadius, startLine.toPoint(), endLine.toPoint());

            double furthestAngle = -1;
            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - robotPos.y, thisIntersection.x - robotPos.x);
                double deltaAngle = Math.abs(MathFunc.AngleWrap(angle - robotPos.theta));

                if(deltaAngle > furthestAngle){
                    furthestAngle = deltaAngle;
                    followMe = new CurvePoint(startLine);
                    currFollowRadius = startLine.followDistance;
                    currFollowAngle = startLine.followAngle;
                    currEndPointIsStopPoint = endLine.isStopPoint;
                    distToEndPoint = Math.hypot(robotPos.x - endLine.x,robotPos.y-endLine.y);
                    endPoint.x = endLine.x;
                    endPoint.y = endLine.y;
                    followMe.setPoint(thisIntersection);
                }

            }
        }

        return followMe;
    }

    public void goToPoint(Pos initialPos, Pos targetPos, double movementSpeed, double turnSpeed){
        double distToTarget = Math.hypot(targetPos.x-initialPos.x,targetPos.y-initialPos.y);

        double absAngleToTarget = Math.atan2(targetPos.y-initialPos.y,targetPos.x-initialPos.x);

        double relAngleToTarget = MathFunc.AngleWrap(absAngleToTarget - (initialPos.theta - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relAngleToTarget)*distToTarget;
        double relativeYToPoint = Math.sin(relAngleToTarget)*distToTarget;
        double movementXPower = relativeXToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementXPower *= movementSpeed;
        movementYPower *= movementSpeed;

        double relativeTurnAngle = initialPos.theta - targetPos.theta;

        double movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(60),-1,1)*turnSpeed;

        if(distToTarget < 2){
            movementTurn = 0;
        }

        double flPower = movementYPower + movementXPower + movementTurn;
        double frPower = movementYPower - movementXPower - movementTurn;
        double brPower = movementYPower + movementXPower - movementTurn;
        double blPower = movementYPower - movementXPower + movementTurn;

        flPower = Range.clip(flPower,-1,1);
        frPower = Range.clip(frPower,-1,1);
        brPower = Range.clip(brPower,-1,1);
        blPower = Range.clip(blPower,-1,1);

        max = flPower;
        if(frPower > max){
            max = frPower;
        }
        if(brPower > max){
            max = brPower;
        }
        if(blPower > max){
            max = blPower;
        }

        if(max > 1) {

            flPower /= max;
            frPower /= max;
            brPower /= max;
            blPower /= max;
        }


        if(currEndPointIsStopPoint && distToEndPoint < currFollowRadius){
            //region redo loop
            distToTarget = Math.hypot(endPoint.x-initialPos.x,endPoint.y-initialPos.y);

            absAngleToTarget = Math.atan2(endPoint.y-initialPos.y,endPoint.x-initialPos.x);

            relAngleToTarget = MathFunc.AngleWrap(absAngleToTarget - (initialPos.theta - Math.toRadians(90)));

            relativeXToPoint = Math.cos(relAngleToTarget)*distToTarget;
            relativeYToPoint = Math.sin(relAngleToTarget)*distToTarget;
            movementXPower = relativeXToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            movementYPower = relativeYToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movementXPower *= movementSpeed;
            movementYPower *= movementSpeed;

            relativeTurnAngle = initialPos.theta - targetPos.theta;

            movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(45),-1,1)*turnSpeed;

            flPower = movementYPower + movementXPower + movementTurn;
            frPower = movementYPower - movementXPower - movementTurn;
            brPower = movementYPower + movementXPower - movementTurn;
            blPower = movementYPower - movementXPower + movementTurn;

            max = flPower;
            if(frPower > max){
                max = frPower;
            }
            if(brPower > max){
                max = brPower;
            }
            if(blPower > max){
                max = blPower;
            }

            if(max > 1) {
                flPower /= max;
                frPower /= max;
                brPower /= max;
                blPower /= max;
            }
            double changeCoeff = 10/distToEndPoint;

            if(changeCoeff > 3){
                changeCoeff = 3;
            }

            flPower /= (changeCoeff);
            frPower /= (changeCoeff);
            brPower /= (changeCoeff);
            blPower /= (changeCoeff);
            //endregion

        }

        if(currEndPointIsStopPoint && (distToEndPoint < stopRadius)){
            loopIsOver = true;
        }

        telemetryString1 = distToEndPoint + "";
        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.br.setPower(brPower);
        robot.bl.setPower(blPower);
        //set powers

    }


    //------------------------------------------------------------------------------------------------------------------------------
    //Driving Power Functions
    public void stopDriving() {
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }


    //distance=rate*duration duration=distance/rate
    //power drives forward, -power drives backward
    public void drive(double power) {
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
    }




    //------------------------------------------------------------------------------------------------------------------------------
    //Encoder Functions


    public void stopAndReset() {
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsing(){
        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(){
        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveDistance(double power, double distance) throws InterruptedException {
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

    public double getZAngle(){
        return (robot.imu.getAngularOrientation().firstAngle);
    }

    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }

    public void grabPos(){
        robot.aL.setPosition(0.8);
        robot.aR.setPosition(0.17);
    }

    public void depositPos(){
        robot.aL.setPosition(0.1);
        robot.aR.setPosition(0.92);
    }

    public void intakePos(){
        robot.aL.setPosition(0.7);
        robot.aR.setPosition(0.27);
    }

    public void grab(){
        robot.gf.setPosition(0.7);
        robot.gs.setPosition(0.51);
    }

    public void release(){
        robot.gf.setPosition(0.27);
        robot.gs.setPosition(0.92);
    }

}