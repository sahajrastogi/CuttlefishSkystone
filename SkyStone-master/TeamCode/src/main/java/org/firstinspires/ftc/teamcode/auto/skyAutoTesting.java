package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.CurvePoint;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.Point;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.auto.odometry.skyOdometryGlobalPositionThread;
import org.firstinspires.ftc.teamcode.skyHMAP;


import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

@Autonomous
public class skyAutoTesting extends LinearOpMode {

    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;
    private double ly,lx, rx;
    public DcMotorEx odo;
    ElapsedTime timer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,false);
        robotAuto = new skyAuto(robot);
        skyAuto.currFollowRadius = 3;

        robotAuto.intakePos();
        robotAuto.release();


        ArrayList<CurvePoint> allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(0.0,0.0,0.85,0.6,7,Math.toRadians(135),false));
        allPoints.add(new CurvePoint(-1,27.0,0.85,0.6,7,Math.toRadians(135),false));
        allPoints.add(new CurvePoint(-7,45.0,0.5,0.5,7,Math.toRadians(135),true));

        timer.startTime();
        waitForStart();
        robot.iL.setPower(1);
        robot.iR.setPower(-1);
        robot.fgl.setPosition(0.6);
        robot.fgr.setPosition(0.4);

        skyOdometryGlobalPositionThread globalPos = new skyOdometryGlobalPositionThread(robot,50,Math.PI/2,0,0);
        Thread positionThread = new Thread(globalPos);
        positionThread.start();


        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        while(!skyAuto.loopIsOver) {
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());

        }
        robotAuto.stopDriving();
        Thread.sleep(50);


        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(-8,50,0.7,0.8,7,Math.toRadians(110),false));
        allPoints.add(new CurvePoint(0.0,31.0,0.75,0.7,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(70.0,31.0,0.4,0.7,7,Math.toRadians(270),false));
        allPoints.add(new CurvePoint(80,40,0.6,0.6,5,Math.toRadians(270),false));
        allPoints.add(new CurvePoint(80,51,0.6,0.6,5,Math.toRadians(270),true));

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() > 20 && globalPos.returnXCoordinate() < 45){
                robotAuto.grabPos();
                robotAuto.grab();
            }

            if(globalPos.returnXCoordinate() > 70){
                robotAuto.depositPos();
            }

            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurveReverse(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();

        robot.fgl.setPosition(0.3);
        robot.fgr.setPosition(0.7);
        sleep(300);

        robotAuto.release();
        sleep(200);

        skyAuto.stopRadius = 5.5;

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(80,50,0.85,0.6,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(60,30.5,0.9,0.6,7,Math.toRadians(180),true));


        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        while(!skyAuto.loopIsOver) {
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();

        robot.fgl.setPosition(0.6);
        robot.fgr.setPosition(0.4);
        robotAuto.intakePos();

        Thread.sleep(800);

        skyAuto.stopRadius = 6;

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);


        robot.iL.setPower(1);
        robot.iR.setPower(-1);

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(65,30.5,0.9,0.5,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(45,30.5,0.55,0.8,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(26,30.5,0.4,0.8,7,Math.toRadians(140),false));
        allPoints.add(new CurvePoint(8,46,0.4,0.8,7,Math.toRadians(140),true));



        while(!skyAuto.loopIsOver) {
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();
        Thread.sleep(50);


        skyAuto.stopRadius = 6.5;

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);


        robot.iL.setPower(1);
        robot.iR.setPower(-1);

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(11,46,0.9,0.7,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(13,31.5,0.8,0.9,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(40,31,0.8,0.9,7,Math.toRadians(200),false));
        allPoints.add(new CurvePoint(60,37,1,0.5,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(80,39,1,0.5,7,Math.toRadians(180),true));



        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() > 25 && globalPos.returnXCoordinate() < 50){
                robotAuto.grabPos();
                robotAuto.grab();
            }
            if(globalPos.returnXCoordinate() > 65){
                robotAuto.depositPos();
            }

            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurveReverse(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();
        Thread.sleep(50);
        robotAuto.release();
        sleep(300);
        robotAuto.intakePos();

        sleep(400);
        skyAuto.stopRadius = 4;

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);


        robot.iL.setPower(1);
        robot.iR.setPower(-1);

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(90,31,0.8,0.9,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(60,31,0.65,0.5,7,Math.toRadians(180),true));


        while(!skyAuto.loopIsOver) {
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();
        Thread.sleep(50);



    }


}




