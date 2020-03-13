package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ButtonOp;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.CurvePoint;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.Point;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.depositThread;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.auto.odometry.skyOdometryGlobalPositionThread;
import org.firstinspires.ftc.teamcode.skyHMAP;


import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class skyAutoBlue extends LinearOpMode {

    SamplePipelineBlue pipe = new SamplePipelineBlue();
    OpenCvCamera webcam;
    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;
    private double ly,lx, rx;
    public DcMotorEx odo;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();

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

    public depositThread dep;



    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,false,true);
        robotAuto = new skyAuto(robot);

        robot.cap.setPosition(0);
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

        //region vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(pipe);
        webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);

        pipe.check = true;
        //region vision
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
        skyAuto.stopRadius = 5;

        robotAuto.fgrabbersUp();
        robotAuto.intakePos();
        robotAuto.release();

        timer.startTime();
        timer1.startTime();
        waitForStart();

        dep = new depositThread(robot);



        pipe.check = false;
        if(pipe.verdictL.equals("stone") && pipe.verdictR.equals("stone")){
            finalVerdict = "right";
        }
        if(pipe.verdictL.equals("stone") && pipe.verdictR.equals("skystone")){
            finalVerdict = "middle";
        }
        if(pipe.verdictL.equals("skystone") && pipe.verdictR.equals("stone")){
            finalVerdict = "left";
        }

        finalVerdict = "left";

        // region initial intake
        ArrayList<CurvePoint> allPoints = new ArrayList<CurvePoint>();
        if(finalVerdict.equals("left")) {
            allPoints.add(new CurvePoint(0.0, 0.0, 0.75, 0.5, 7, Math.toRadians(45), false));
            allPoints.add(new CurvePoint(1.5, 28, 0.6, 0.7, 7, Math.toRadians(40), false));
            allPoints.add(new CurvePoint(5.85, 48.0, 0.6, 0.7, 7, Math.toRadians(40), true));
        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(0.0, 0.0, 0.65, 0.5, 7, Math.toRadians(45), false));
            allPoints.add(new CurvePoint(7.25, 20, 0.5, 0.7, 7, Math.toRadians(45), false));
            allPoints.add(new CurvePoint(11.25, 52.0, 0.6, 0.7, 7, Math.toRadians(45), true));
        } else {
            allPoints.add(new CurvePoint(0.0, 0.0, 1, 1, 7, Math.toRadians(40), false));
            allPoints.add(new CurvePoint(9.5, 16.5, 0.6, 0.8, 7, Math.toRadians(40), false));
            allPoints.add(new CurvePoint(12, 22, 0.6, 0.8, 7, Math.toRadians(40), false));
            allPoints.add(new CurvePoint(22, 47.0, 0.7, 1, 7, Math.toRadians(40), false));
            allPoints.add(new CurvePoint(22, 60.0, 0.7, 1, 7, Math.toRadians(40), true));

        }
        robotAuto.intake();


        skyOdometryGlobalPositionThread globalPos = new skyOdometryGlobalPositionThread(robot,50,Math.PI/2,0,0);
        Thread positionThread = new Thread(globalPos);
        positionThread.start();


        skyAuto.stopRadius = 5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        while(!skyAuto.loopIsOver) {
            telemetry.addData("finalVerdict",finalVerdict);
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());

            if(!Double.isNaN(robot.dist.getDistance(DistanceUnit.CM)) && robot.dist.getDistance(DistanceUnit.CM) < 20){
                break;
            }
        }
        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region path to foundation
        allPoints = new ArrayList<CurvePoint>();



        if(finalVerdict.equals("left")) {
            allPoints.add(new CurvePoint(6,48,0.8,0.8,7,Math.toRadians(80),false));
            allPoints.add(new CurvePoint(0.0,27,0.8,0.55,7,Math.toRadians(0),false));

        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(11.5,52,0.8,0.8,7,Math.toRadians(82),false));
            allPoints.add(new CurvePoint(9,27,0.8,0.55,7,Math.toRadians(0),false));
        } else {
            allPoints.add(new CurvePoint(24,60,0.8,0.8,7,Math.toRadians(90),false));
            allPoints.add(new CurvePoint(24,47,0.8,0.8,7,Math.toRadians(95),false));
            allPoints.add(new CurvePoint(19,27,0.8,0.8,7,Math.toRadians(0),false));

        }
        allPoints.add(new CurvePoint(-69,30.0,0.5,0.7,7,Math.toRadians(-90),false));
        allPoints.add(new CurvePoint(-78,36,0.5,0.6 ,5,Math.toRadians(-87),false));
        allPoints.add(new CurvePoint(-78,48,0.6,0.6,5,Math.toRadians(-87),true));

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        boolean temp = true;
        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() < -20 && globalPos.returnXCoordinate() > -45){
                robotAuto.grabPos();
                robotAuto.grab();
                robotAuto.fgrabbersMiddle();
            }

            if(globalPos.returnXCoordinate() < -70 && temp){
                Thread thread1 = new Thread(dep);
                thread1.start();
                temp = false;

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
        //endregion

        //region lock grabbers, deposit stone, move foundation

        robotAuto.fgrabbersDown();
        sleep(750);

        skyAuto.stopRadius = 6.5;

        if(finalVerdict.equals("left")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(-80, 47, 1, 1, 7, Math.toRadians(-75), false));
            allPoints.add(new CurvePoint(-75, 30, 1, 1, 7, Math.toRadians(0), false));
            allPoints.add(new CurvePoint(-50, 27, 1, 1, 7, Math.toRadians(0), true));

        } else {
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(-80, 47, 1, 1, 7, Math.toRadians(-75), false));
            allPoints.add(new CurvePoint(-75, 30, 1, 1, 7, Math.toRadians(0), false));
            allPoints.add(new CurvePoint(-50, 29, 1, 1, 7, Math.toRadians(0), true));
        }

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

        robotAuto.fgrabbersUp();

        //endregion

        //region path and intake for second stone
        skyAuto.stopRadius = 6;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);


        robotAuto.intake();

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(-65,28,0.8,0.5,7,Math.toRadians(0),false));
        allPoints.add(new CurvePoint(-45,28,0.8,0.8,7,Math.toRadians(0),false));


        if(finalVerdict.equals("left")) {
            allPoints.add(new CurvePoint(-33,27,0.5,0.9,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-27,27,0.4,0.7,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(5,60,0.5,0.8,7,Math.toRadians(40),true));

        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(-27,27,0.5,0.9,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-21,27,0.5,0.8,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(13,60,0.6,0.8,7,Math.toRadians(40),true));

        } else {
            allPoints.add(new CurvePoint(-19,28,0.5,0.8,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-13,28,0.6,0.8,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(21,60,0.6,0.8,7,Math.toRadians(40),true));

        }

        while(!skyAuto.loopIsOver) {
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());

            if(!Double.isNaN(robot.dist.getDistance(DistanceUnit.CM)) && robot.dist.getDistance(DistanceUnit.CM) < 20){
                break;
            }
        }
        robotAuto.stopDriving();
        //endregion
        timer1.reset();
        robotAuto.stopIntake();

        //region path to foundation for second stone
        skyAuto.stopRadius = 6.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);



        allPoints = new ArrayList<CurvePoint>();

        if(finalVerdict.equals("left")) {
            allPoints.add(new CurvePoint(9,50,0.65,0.7,7,Math.toRadians(65),false));
            allPoints.add(new CurvePoint(-10,25,0.65,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,25,0.65,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-50,25,0.65,0.5,7,Math.toRadians(0),true));
        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(15,60,0.65,0.7,7,Math.toRadians(60),false));
            allPoints.add(new CurvePoint(-5,25,0.65,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,25,0.65,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-50,25,0.65,0.5,7,Math.toRadians(0),true));
        } else {
            allPoints.add(new CurvePoint(15,60,0.8,0.7,7,Math.toRadians(60),false));
            allPoints.add(new CurvePoint(-1,26,0.8,0.9,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,26,0.8,0.9,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-50,26,0.8,0.5,7,Math.toRadians(0),true));

        }



        temp = true;
        while(!skyAuto.loopIsOver) {

            if(timer1.seconds() > 1.5){
                robotAuto.intake();
            }
            if(globalPos.returnXCoordinate() < -20 && globalPos.returnXCoordinate() > -34){
                robotAuto.grabPos();
                robotAuto.grab();
                robotAuto.extake();
            }


            if(globalPos.returnXCoordinate() < -35 && temp){
                Thread thread2 = new Thread(dep);
                thread2.start();
                depositThread.releaseReady = false;
                temp = false;
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

        //endregion

        double finishXPos = -20;
        double finishYPos = 47;

        robotAuto.intake();
        //region path for third stone
        while(!depositThread.releaseReady){

        }

        skyAuto.stopRadius = 5.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        robotAuto.intake();

        allPoints = new ArrayList<CurvePoint>();

        if(finalVerdict.equals("left")){
            allPoints.add(new CurvePoint(-50,25,0.8,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,25,0.5,0.9,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-20,25,0.75,0.8,7,Math.toRadians(13),false));
            allPoints.add(new CurvePoint(18,44,0.8,0.8,7,Math.toRadians(0),true));
        }else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(-50,25,0.8,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,25,0.5,0.9,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-20,25,0.75,0.8,7,Math.toRadians(10),false));
            allPoints.add(new CurvePoint(20,40,0.8,0.8,7,Math.toRadians(0),true));
        }else{
            allPoints.add(new CurvePoint(-50,26.5,0.8,0.5,7,Math.toRadians(-1.5),false));
            allPoints.add(new CurvePoint(-40,26.5,0.5,0.5,7,Math.toRadians(-1.5),false));
            allPoints.add(new CurvePoint(-23.5,26.5,0.6,0.9,7,Math.toRadians(45),false));
            allPoints.add(new CurvePoint(-6,47,0.7,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(20,47,1,0.8,7,Math.toRadians(0),true));
        }


        while(!skyAuto.loopIsOver) {

            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());

            if(!Double.isNaN(robot.dist.getDistance(DistanceUnit.CM)) && robot.dist.getDistance(DistanceUnit.CM) < 20){
                finishXPos = globalPos.returnXCoordinate();
                finishYPos = globalPos.returnYCoordinate();
                break;

            }
        }
        robotAuto.stopDriving();
        robotAuto.stopIntake();
        //endregion
        finishXPos = globalPos.returnXCoordinate();
        finishYPos = globalPos.returnYCoordinate();
        //region path to foundation for third stone
        while(!depositThread.releaseReady){

        }

        skyAuto.stopRadius = 5.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);


        robotAuto.intake();

        if(finalVerdict.equals("left")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.6,0.8,7,Math.toRadians(70),false));
            allPoints.add(new CurvePoint(finishXPos-8,22,0.7,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-20,22,0.7,0.8,7,Math.toRadians(0),true));
            allPoints.add(new CurvePoint(-43,22,1,0.8,7,Math.toRadians(0),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.6,0.8,7,Math.toRadians(70),false));
            allPoints.add(new CurvePoint(finishXPos-8,22,0.7,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-20,22,0.7,0.8,7,Math.toRadians(0),true));
            allPoints.add(new CurvePoint(-43,22,1,0.8,7,Math.toRadians(0),true));
        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos+1,finishYPos,0.6,0.6,7,Math.toRadians(80),false));
            allPoints.add(new CurvePoint(finishXPos,26,0.6,0.6,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-45,26,1,0.8,7,Math.toRadians(0),true));
        }



        temp = true;
        while(!skyAuto.loopIsOver) {

                if (timer1.seconds() > 1.5) {
                    robotAuto.intake();
                }



            if(globalPos.returnXCoordinate() < -20 && globalPos.returnXCoordinate() > -34){
                robotAuto.grabPos();
                robotAuto.grab();
            }

            if(globalPos.returnXCoordinate() < -35 && temp){
                Thread thread3 = new Thread(dep);
                thread3.start();
                depositThread.releaseReady = false;
                temp = false;
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

        //endregion

        //region path for fourth stone
        while(!depositThread.releaseReady){

        }

        skyAuto.stopRadius = 5.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        robotAuto.intake();

        if(finalVerdict.equals("left")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(-50,22,0.8,0.1,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,22,0.8,0.5,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-30,22,-0.1,0.8,7,Math.toRadians(30),false));
            allPoints.add(new CurvePoint(-25,22,0.7,0.8,7,Math.toRadians(30),false));
            allPoints.add(new CurvePoint(20,56,1,0.8,7,Math.toRadians(0),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(-50,22,0.8,0.1,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-40,22,0.8,0.5,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-30,22,-0.1,0.8,7,Math.toRadians(30),false));
            allPoints.add(new CurvePoint(-25,22,0.7,0.8,7,Math.toRadians(30),false));
            allPoints.add(new CurvePoint(20,49,1,0.8,7,Math.toRadians(0),true));

        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(-52,26,0.5,0.6,7,Math.toRadians(-1.5),false));
            allPoints.add(new CurvePoint(-40,26,0.5,0.6,7,Math.toRadians(-1.5),false));
            allPoints.add(new CurvePoint(-30,26,0.6,0.8,7,Math.toRadians(25),false));
            allPoints.add(new CurvePoint(-25,26,0.7,0.8,7,Math.toRadians(25),false));
            allPoints.add(new CurvePoint(20,48,1,0.8,7,Math.toRadians(0),true));

        }

        while(!skyAuto.loopIsOver) {


            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurve(allPoints,globalPos.returnPos());

            if(!Double.isNaN(robot.dist.getDistance(DistanceUnit.CM)) && robot.dist.getDistance(DistanceUnit.CM) < 20){
                break;
            }
        }
        robotAuto.stopDriving();
        timer1.reset();
        robotAuto.stopIntake();
        finishXPos = globalPos.returnXCoordinate();
        finishYPos = globalPos.returnYCoordinate();
        //endregion

        //region path to foundation for fourth stone
        while(!depositThread.releaseReady){

        }
        skyAuto.stopRadius = 5.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        robotAuto.intake();

        if(finalVerdict.equals("left")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.8,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-12,20,0.8,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-82,20,8 ,0.8,7,Math.toRadians(0),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.8,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-12,18,0.8,0.8,7,Math.toRadians(0),false));
            allPoints.add(new CurvePoint(-82,18,8 ,0.8,7,Math.toRadians(0),true));

        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.5,7,Math.toRadians(40),false));
            allPoints.add(new CurvePoint(-10,24,0.8,0.45,7,Math.toRadians(1),false));
            allPoints.add(new CurvePoint(-83,24,0.8 ,0.45,7,Math.toRadians(0),true));


        }


        temp = true;
        while(!skyAuto.loopIsOver) {
            if (timer1.seconds() > 1.5) {
                robotAuto.intake();
            }


            if(globalPos.returnXCoordinate() < -20 && globalPos.returnXCoordinate() > -34){
                robotAuto.grabPos();
                robotAuto.grab();
            }

            if(globalPos.returnXCoordinate() < -35 && temp){
                Thread thread4 = new Thread(dep);
                thread4.start();
                depositThread.releaseReady = false;
                temp = false;
            }
            telemetry.addData("telString1",skyAuto.telemetryString1);
            telemetry.addData("telString2",skyAuto.telemetryString2);
            telemetry.addData("y",globalPos.globaly);
            telemetry.addData("robotAngle", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("loopOver",skyAuto.loopIsOver);
            telemetry.addData("currEndPoint",skyAuto.currEndPointIsStopPoint);
            telemetry.update();

            robotAuto.followCurveReverse(allPoints,globalPos.returnPos());
        }
        robotAuto.stopDriving();
        //endregion

        //region park


        skyAuto.stopRadius = 5.5;
        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        robotAuto.intake();

        allPoints = new ArrayList<CurvePoint>();

        if(finalVerdict.equals("left")){
            allPoints.add(new CurvePoint(-83, 13, 0.75, 0.8, 7, Math.toRadians(0), false));
            allPoints.add(new CurvePoint(-59, 13, 0.75, 0.8, 7, Math.toRadians(0), true));
        }
        if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(-83, 16, 0.75, 0.8, 7, Math.toRadians(0), false));
            allPoints.add(new CurvePoint(-59, 16, 0.75, 0.8, 7, Math.toRadians(0), true));
        } else {
            allPoints.add(new CurvePoint(-83, 21, 0.9, 0.8, 7, Math.toRadians(0), false));
            allPoints.add(new CurvePoint(-61, 21, 0.9 , 0.8, 7, Math.toRadians(0), true));
        }


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

        //endregion

        webcam.stopStreaming();

    }


}




