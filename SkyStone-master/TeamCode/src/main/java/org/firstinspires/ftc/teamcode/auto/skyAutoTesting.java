package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ButtonOp;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.CurvePoint;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.Point;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.auto.odometry.skyOdometryGlobalPositionThread;
import org.firstinspires.ftc.teamcode.skyHMAP;


import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class skyAutoTesting extends LinearOpMode {

    SamplePipelineRed pipe = new SamplePipelineRed();
    OpenCvCamera webcam;
    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;
    private double ly,lx, rx;
    public DcMotorEx odo;
    ElapsedTime timer = new ElapsedTime();

    String finalVerdict = "";
    boolean boxLeft = true;
    ButtonOp g1a = new ButtonOp();
    ButtonOp g1b = new ButtonOp();
    ButtonOp g1x = new ButtonOp();
    ButtonOp g1y = new ButtonOp();
    ButtonOp g1du = new ButtonOp();
    ButtonOp g1dd = new ButtonOp();
    ButtonOp g1dl = new ButtonOp();
    ButtonOp g1dr = new ButtonOp();
    ButtonOp g1bl = new ButtonOp();
    ButtonOp g1br = new ButtonOp();



    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,false);
        robotAuto = new skyAuto(robot);

        //region vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(pipe);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while(true){
            if(boxLeft){
                if(g1dr.onRelease()){
                    pipe.x1 += 5;
                }
                if(g1dl.onRelease()){
                    pipe.x1 -= 5;
                }
                if(g1du.onRelease()){
                    pipe.y1 -=5;
                }
                if(g1dd.onRelease()){
                    pipe.y1 +=5;
                }

                if(g1b.onRelease()){
                    pipe.x2 += 5;
                }
                if(g1x.onRelease()){
                    pipe.x2 -= 5;
                }
                if(g1y.onRelease()){
                    pipe.y2 -=5;
                }
                if(g1a.onRelease()){
                    pipe.y2 +=5;
                }
            } else {
                if(g1dr.onRelease()){
                    pipe.x3 += 5;
                }
                if(g1dl.onRelease()){
                    pipe.x3 -= 5;
                }
                if(g1du.onRelease()){
                    pipe.y3 -=5;
                }
                if(g1dd.onRelease()){
                    pipe.y3 +=5;
                }

                if(g1b.onRelease()){
                    pipe.x4 += 5;
                }
                if(g1x.onRelease()){
                    pipe.x4 -= 5;
                }
                if(g1y.onRelease()){
                    pipe.y4 -=5;
                }
                if(g1a.onRelease()){
                    pipe.y4 +=5;
                }
            }

            if(g1bl.onPress() || g1br.onPress()){
                boxLeft = !boxLeft;

            }

            if(gamepad1.left_trigger > 0.1){
                pipe.check = true;
            } else {
                pipe.check = false;
            }

            g1a.update(gamepad1.a);
            g1b.update(gamepad1.b);
            g1y.update(gamepad1.y);
            g1x.update(gamepad1.x);
            g1dd.update(gamepad1.dpad_down);
            g1du.update(gamepad1.dpad_up);
            g1dl.update(gamepad1.dpad_left);
            g1dr.update(gamepad1.dpad_right);
            g1bl.update(gamepad1.left_bumper);
            g1br.update(gamepad1.right_bumper);
            if(isStarted()){
                break;
            }
            telemetry.addData("boxLeft",boxLeft);
            telemetry.addData("a1s",pipe.a1s);
            telemetry.addData("a2s",pipe.a2s);
            telemetry.addData("a3s",pipe.a3s);
            telemetry.addData("verdictL",pipe.verdictL);
            telemetry.addData("verdictR",pipe.verdictR);

            telemetry.addData("x1",pipe.x1);
            telemetry.addData("y1",pipe.y1);
            telemetry.addData("x2",pipe.x2);
            telemetry.addData("y2",pipe.y2);
            telemetry.addData("x3",pipe.x3);
            telemetry.addData("y3",pipe.y3);
            telemetry.addData("x4",pipe.x4);
            telemetry.addData("y4",pipe.y4);
            telemetry.update();
        }

        //endregion


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




