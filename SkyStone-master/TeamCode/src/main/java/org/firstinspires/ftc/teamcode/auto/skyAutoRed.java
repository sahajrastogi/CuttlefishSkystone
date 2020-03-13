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
public class skyAutoRed extends LinearOpMode {

    ElapsedTime timer1 = new ElapsedTime();
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
        timer1.startTime();
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
        waitForStart();

        dep = new depositThread(robot);



        pipe.check = false;
        if(pipe.verdictL.equals("stone") && pipe.verdictR.equals("stone")){
            finalVerdict = "left";
        }
        if(pipe.verdictL.equals("stone") && pipe.verdictR.equals("skystone")){
            finalVerdict = "right";
        }
        if(pipe.verdictL.equals("skystone") && pipe.verdictR.equals("stone")){
            finalVerdict = "middle";
        }

        // region initial intake
        ArrayList<CurvePoint> allPoints = new ArrayList<CurvePoint>();
        if(finalVerdict.equals("right")) {
            allPoints.add(new CurvePoint(0.0, 0.0, 0.75, 0.5, 7, Math.toRadians(135), false));
            allPoints.add(new CurvePoint(-2, 28, 0.6, 0.7, 7, Math.toRadians(140), false));
            allPoints.add(new CurvePoint(-5, 48.0, 0.6, 0.7, 7, Math.toRadians(140), true));
        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(0.0, 0.0, 0.75, 0.5, 7, Math.toRadians(135), false));
            allPoints.add(new CurvePoint(-7.5, 20, 0.7, 0.7, 7, Math.toRadians(135), false));
            allPoints.add(new CurvePoint(-10.5, 48.0, 0.6, 0.7, 7, Math.toRadians(135), true));
        } else {
            allPoints.add(new CurvePoint(0.0, 0.0, 1, 1, 7, Math.toRadians(140), false));
            allPoints.add(new CurvePoint(-9, 16.5, 0.6, 0.8, 7, Math.toRadians(140), false));
            allPoints.add(new CurvePoint(-12, 22, 0.6, 0.8, 7, Math.toRadians(140), false));
            allPoints.add(new CurvePoint(-22, 47.0, 0.7, 1, 7, Math.toRadians(140), false));
            allPoints.add(new CurvePoint(-22, 60.0, 0.7, 1, 7, Math.toRadians(140), true));

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



        if(finalVerdict.equals("right")) {
            allPoints.add(new CurvePoint(-5,48,0.8,0.8,7,Math.toRadians(110),false));
            allPoints.add(new CurvePoint(0.0,30,0.8,0.7,7,Math.toRadians(180),false));

        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(-11,48,0.8,0.8,7,Math.toRadians(105),false));
            allPoints.add(new CurvePoint(-9,30,0.8,0.55,7,Math.toRadians(180),false));
//i'm the best coder on the team
        } else {
            allPoints.add(new CurvePoint(-24,60,0.8,0.8,7,Math.toRadians(90),false));
            allPoints.add(new CurvePoint(-24,47,0.8,0.8,7,Math.toRadians(105),false));
            allPoints.add(new CurvePoint(-19,30,0.8,0.8,7,Math.toRadians(180),false));

        }
        allPoints.add(new CurvePoint(69.0,30.0,0.5,0.7,7,Math.toRadians(270),false));
        allPoints.add(new CurvePoint(80,36,0.5,0.6 ,5,Math.toRadians(270),false));
        allPoints.add(new CurvePoint(80,48,0.6,0.6,5,Math.toRadians(270),true));

        skyAuto.currFollowRadius = 7;
        skyAuto.currEndPointIsStopPoint = false;
        skyAuto.currFollowAngle = 90;
        skyAuto.loopIsOver = false;
        skyAuto.distToEndPoint = 80;
        skyAuto.endPoint = new Point(0,0);

        boolean temp = true;
        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() > 20 && globalPos.returnXCoordinate() < 45){
                robotAuto.grabPos();
                robotAuto.grab();
                robotAuto.fgrabbersMiddle();
            }

            if(globalPos.returnXCoordinate() > 70 && temp){
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

        allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(80,47,1,1,7,Math.toRadians(250),false));
        allPoints.add(new CurvePoint(76,30,1,1,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(50,29,1,1,7,Math.toRadians(180),true));


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
        allPoints.add(new CurvePoint(65,31,0.8,0.5,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(45,31,0.8,0.8,7,Math.toRadians(180),false));


        if(finalVerdict.equals("right")) {
            allPoints.add(new CurvePoint(33,31,0.6,0.65,7,Math.toRadians(150),false));
            allPoints.add(new CurvePoint(27,31,0.6,0.9,7,Math.toRadians(130),false));
            allPoints.add(new CurvePoint(-7,60,0.6,0.8,7,Math.toRadians(130),true));

        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(28,31,0.6,1,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(21,31,0.5,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(-14,60,0.6,0.8,7,Math.toRadians(140),true));

        } else {
            allPoints.add(new CurvePoint(23,31,0.6,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(17.5,31,0.6,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(-19,60,0.6,0.8,7,Math.toRadians(140),true));

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
        //region path to foundation for second stone
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

        if(finalVerdict.equals("right")) {
            allPoints.add(new CurvePoint(-7,60,0.65,0.7,7,Math.toRadians(120),false));
            allPoints.add(new CurvePoint(10,29.5,0.65,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,29.5,0.65,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(50,29.5,0.65,0.5,7,Math.toRadians(180),true));
        } else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(-14,60,0.65,0.7,7,Math.toRadians(120),false));
            allPoints.add(new CurvePoint(5,30,0.65,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,30,0.65,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(50,30,0.65,0.5,7,Math.toRadians(180),true));
        } else {
            allPoints.add(new CurvePoint(-15,60,0.8,0.7,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(1,33,0.8,0.9,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,33,0.8,0.9,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(50,33,0.8,0.5,7,Math.toRadians(180),true));

        }



        temp = true;
        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() > 20 && globalPos.returnXCoordinate() < 34){
                robotAuto.grabPos();
                robotAuto.grab();
                robotAuto.extake();
            }


            if(globalPos.returnXCoordinate() > 35 && temp){
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


            if(globalPos.returnXCoordinate() > 12 && timer1.seconds() > 1.5){
                robotAuto.intake();
            } else{
                robotAuto.stopIntake();
            }
        }
        robotAuto.stopDriving();

        //endregion

        double finishXPos = -20;
        double finishYPos = 47;
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

        if(finalVerdict.equals("right")){
            allPoints.add(new CurvePoint(50,30,0.6,0.7,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,30,0.5,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(20,30,0.6,0.7,7,Math.toRadians(165),false));
            allPoints.add(new CurvePoint(-15,42.9,0.7,0.8,7,Math.toRadians(180),true));
        }else if(finalVerdict.equals("middle")){
            allPoints.add(new CurvePoint(50,30,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,30,0.5,0.9,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(20,30,0.6,1,7,Math.toRadians(165),false));
            allPoints.add(new CurvePoint(-20,40,1,0.8,7,Math.toRadians(180),true));
        }else{
            allPoints.add(new CurvePoint(50,34,0.8,0.8,7,Math.toRadians(179),false));
            allPoints.add(new CurvePoint(40,34,0.5,0.9,7,Math.toRadians(179),false));
            allPoints.add(new CurvePoint(25.5,34,0.6,0.9 ,7,Math.toRadians(120),false));
            allPoints.add(new CurvePoint(14,57,0.7,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(-20,64,1,0.8,7,Math.toRadians(180),true));
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

        //endregion

        timer1.reset();
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

        if(finalVerdict.equals("right")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.6,0.75,7,Math.toRadians(100),false));
            allPoints.add(new CurvePoint(finishXPos+1,32,0.8,0.75,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(20,32,0.8,0.8,7,Math.toRadians(180),true));
            allPoints.add(new CurvePoint(45,32,1,0.8,7,Math.toRadians(180),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.6,0.8,7,Math.toRadians(100),false));
            allPoints.add(new CurvePoint(finishXPos+1,32,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(20,32,1,0.8,7,Math.toRadians(180),true));
            allPoints.add(new CurvePoint(45,32,1,0.8,7,Math.toRadians(180),true));
        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.6,0.8,7,Math.toRadians(100),false));
            allPoints.add(new CurvePoint(finishXPos+1,35.5,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(45,35.5,1,0.8,7,Math.toRadians(180),true));
        }



        temp = true;
        while(!skyAuto.loopIsOver) {

            if (globalPos.returnXCoordinate() > 18 && timer1.seconds() > 1.5) {
                robotAuto.intake();
            } else {
                robotAuto.stopIntake();
            }


            if(globalPos.returnXCoordinate() > 20 && globalPos.returnXCoordinate() < 34){
                robotAuto.grabPos();
                robotAuto.grab();
            }

            if(globalPos.returnXCoordinate() > 35 && temp){
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

        if(finalVerdict.equals("right")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(45,32,0.8,0.1,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,32,0.8,0.5,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(30,32,-0.1,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(25,32,0.7,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(-20,61,0.7,0.8,7,Math.toRadians(180),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(45,32.5,0.8,0.1,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,32.5,0.8,0.5,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(30,32.5,-0.1,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(25,32.5,0.7,0.8,7,Math.toRadians(140),false));
            allPoints.add(new CurvePoint(-20,65.5,1,0.8,7,Math.toRadians(180),true));

        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(45,35.5,0.8,0.35,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(40,35.5,0.8,0.5,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(30,35.5,-0.1,0.8,7,Math.toRadians(160),false));
            allPoints.add(new CurvePoint(25,35.5,0.7,0.8,7,Math.toRadians(160),false));
            allPoints.add(new CurvePoint(-20,55,1,0.8,7,Math.toRadians(180),true));

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

        if(finalVerdict.equals("right")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.8,7,Math.toRadians(127),false));
            allPoints.add(new CurvePoint(25,33,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(82,33,0.8 ,0.8,7,Math.toRadians(180),true));
        }else if(finalVerdict.equals("middle")){
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.8,7,Math.toRadians(130),false));
            allPoints.add(new CurvePoint(25,33,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(82,33,0.8 ,0.8,7,Math.toRadians(180),true));

        }else{
            allPoints = new ArrayList<CurvePoint>();
            allPoints.add(new CurvePoint(finishXPos,finishYPos,0.8,0.8,7,Math.toRadians(150),false));
            allPoints.add(new CurvePoint(20,35.5,0.8,0.8,7,Math.toRadians(180),false));
            allPoints.add(new CurvePoint(82,35.5,0.8 ,0.8,7,Math.toRadians(180),true));

        }


        temp = true;
        while(!skyAuto.loopIsOver) {

            if(globalPos.returnXCoordinate() > 20 && globalPos.returnXCoordinate() < 34){
                robotAuto.grabPos();
                robotAuto.grab();
            }

            if(globalPos.returnXCoordinate() > 35 && temp){
                Thread thread4 = new Thread(dep);
                thread4.start();
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
        allPoints.add(new CurvePoint(83,34,1,0.8,7,Math.toRadians(180),false));
        allPoints.add(new CurvePoint(61,34,1,0.8,7,Math.toRadians(180),true));


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




