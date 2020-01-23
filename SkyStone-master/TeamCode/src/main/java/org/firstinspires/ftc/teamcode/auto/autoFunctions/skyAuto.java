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

    public skyAuto(skyHMAP hwmap){

        robot = hwmap;
    }
    public void runOpMode() throws InterruptedException{

    }

    public void followCurve(ArrayList<CurvePoint> allPoints, Pos robotPos, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, robotPos, allPoints.get(0).followDistance);
        goToPoint(robotPos, new Pos(followMe.x, followMe.y,followAngle),followMe.moveSpeed,followMe.turnSpeed);
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pos robotPos, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i=0;i<pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunc.lineCircleIntersection(new Point(robotPos.x, robotPos.y), followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000;
            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - robotPos.y, thisIntersection.x-robotPos.x);
                double deltaAngle = Math.abs(MathFunc.AngleWrap(angle - robotPos.theta));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }

            }
        }

        return followMe;
    }
//i'm the best coder on the team


    public void goToPoint(Pos initialPos, Pos targetPos, double movementSpeed, double turnSpeed){
        double distToTarget = Math.hypot(targetPos.x-initialPos.x,targetPos.y-initialPos.y);

        double absAngleToTarget = Math.atan2(targetPos.y-initialPos.y,targetPos.x-initialPos.x);

        double relAngleToTarget = MathFunc.AngleWrap(absAngleToTarget - (initialPos.theta - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relAngleToTarget)*distToTarget;
        double relativeYToPoint = Math.sin(relAngleToTarget)*distToTarget;
        double movementXPower = relativeXToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint/(Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relAngleToTarget - Math.toRadians(180) + targetPos.theta; // change

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


}