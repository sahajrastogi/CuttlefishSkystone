/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
public class skyAutoBlue extends LinearOpMode
{
    OpenCvCamera phoneCam;
    SamplePipeline pipe = new SamplePipeline();
    skyHMAP robot;
    skyAuto robotAuto;
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


    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new skyHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);
        robot.capstone.setPosition(0.4);
        robot.cap.setPosition(0);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(pipe);

        phoneCam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);

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

        waitForStart();
        pipe.check = true;
        sleep(400);
        pipe.check = false;


        if(pipe.verdictL.equals("stone")&&pipe.verdictR.equals("stone")){
            finalVerdict = "left";
        } else
        if(pipe.verdictL.equals("stone")&&pipe.verdictR.equals("skystone")){
            finalVerdict = "middle";
        }else if(pipe.verdictL.equals("skystone")&&pipe.verdictR.equals("stone")){
            finalVerdict = "right";
        }

        telemetry.addData("verdictL",pipe.verdictL);
        telemetry.addData("verdictR",pipe.verdictR);
        telemetry.addData("final",finalVerdict);
        telemetry.update();


        //region drive forward and turn
        robot.fl.setTargetPositionTolerance(20);
        robot.fr.setTargetPositionTolerance(20);
        robot.bl.setTargetPositionTolerance(20);
        robot.br.setTargetPositionTolerance(20);
        phoneCam.stopStreaming();
        robotAuto.driveDistance(-0.35,-10.7);

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robotAuto.getHeading() > -82){
            robot.fl.setPower(0.15);
            robot.fr.setPower(-0.15);
            robot.bl.setPower(0.15);
            robot.br.setPower(-0.15);
        }

        robotAuto.stopDriving();
        //endregion

        //region case code drive distance, turn and intake, realign

        if(finalVerdict.equals("left")){
            robotAuto.driveDistance(-0.2, -2.5);
        } else if(finalVerdict.equals("middle")) {
            robotAuto.driveDistance(-0.2, -5.8);
        } else if(finalVerdict.equals("right")){
            robotAuto.driveDistance(-0.2, -9.2);
        }
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robotAuto.getHeading() > -135){
            robot.fl.setPower(0.2);
            robot.fr.setPower(-0.2);
            robot.bl.setPower(0.2);
            robot.br.setPower(-0.2);
        }

        robot.iL.setPower(1);
        robot.iR.setPower(-1);
        robot.teleopClawLeft.setPosition(0.35);
        robot.teleopClawRight.setPosition(0.7);
        robotAuto.driveDistance(0.25,5);
        robotAuto.driveDistance(-0.25,-6);

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robotAuto.getHeading() < -97){
            robot.fl.setPower(-0.2);
            robot.fr.setPower(0.2);
            robot.bl.setPower(-0.2);
            robot.br.setPower(0.2);
            telemetry.addData("heading",robotAuto.getHeading());
            telemetry.update();
        }
        robotAuto.stopDriving();
        sleep(100);
        if(robotAuto.getHeading() > -89){
            while(robotAuto.getHeading() >-88) {
                robot.fl.setPower(0.05);
                robot.fr.setPower(-0.05);
                robot.bl.setPower(0.05);
                robot.br.setPower(-0.05);
                telemetry.addData("heading",robotAuto.getHeading());
                telemetry.update();
            }
        }
        robotAuto.stopDriving();
        sleep(500);

        telemetry.addData("heading",robotAuto.getHeading());
        telemetry.update();

        robotAuto.stopDriving();
        sleep(50);


        //endregion

        //region drive to foundation and start lift
        robot.iL.setPower(0);
        robot.iR.setPower(0);
        robot.teleopClawLeft.setPosition(0.2);
        robot.teleopClawRight.setPosition(1);
        robotAuto.stopDriving();
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fl.setPower(0.5);
        robot.fr.setPower(0.5);
        robot.bl.setPower(0.5);
        robot.br.setPower(0.5);


        while(true){
            if(robot.fl.getCurrentPosition() > 1800){
                robot.fl.setPower(0.2);
                robot.fr.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.br.setPower(0.2);
                if(robot.distBack.getDistance(DistanceUnit.CM) <25){
                    break;
                }

            }
            telemetry.addData("dist",robot.distBack.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        robotAuto.stopDriving();
        //endregion


        //region rotate move foundation
        robotAuto.stopAndReset();
        robotAuto.runUsing();
        while(robotAuto.getHeading() > -172 && robotAuto.getHeading() < 0){
            robot.fl.setPower(0.2);
            robot.fr.setPower(-0.2);
            robot.bl.setPower(0.2);
            robot.br.setPower(-0.2);
        }

        robotAuto.driveDistance(0.2,3);
        robotAuto.grabbersDown();
        sleep(350);

        robotAuto.stopAndReset();
        robotAuto.runUsing();
        while(true){
            robot.fl.setPower(-0.25);
            robot.fr.setPower(-0.25);
            robot.bl.setPower(-0.25);
            robot.br.setPower(-0.25);
            if(robot.fl.getCurrentPosition() < -1000){
                if(robot.distBack.getDistance(DistanceUnit.CM) < 4){
                    break;
                }
            }
        }
        robotAuto.stopDriving();
        //endregion

        //region strafe out, redajust it, park
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(-1);
        robot.bl.setPower(-0.37);
        robot.br.setPower(0.37);
        robot.fl.setPower(1);
        Thread.sleep(50);

        robotAuto.grabbersUp();
        while(robot.fr.getCurrentPosition() > -2500){

        }
        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        //change stuff
        robotAuto.driveDistance(0.2,2);

        robot.fl.setPower(-0.2);
        robot.fr.setPower(0.2);



        robotAuto.stopDriving();
        Thread.sleep(50);

        robot.lift.setPower(1);
        robot.lift.setTargetPosition(4500);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotAuto.stopAndReset();
        robotAuto.runUsing();
        robot.fr.setPower(1);
        robot.bl.setPower(0.37);
        robot.br.setPower(-0.37);
        robot.fl.setPower(-1);

        while(robot.fr.getCurrentPosition() < 2000){
            if(robot.fr.getCurrentPosition() > 750){
                robot.teleopRotate.setTargetPosition(-315);
                robot.teleopRotate.setPower(-0.15);
                robot.teleopRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        robot.teleopClawLeft.setPosition(0.35);
        robot.teleopClawRight.setPosition(0.7);

        sleep(100);
        robot.teleopClawLeft.setPosition(0.2);
        robot.teleopClawRight.setPosition(1);

        robot.teleopRotate.setTargetPosition(50);
        robot.teleopRotate.setPower(0.3);
        robot.lift.setTargetPosition(0);
        robot.lift.setPower(-1);
        robotAuto.stopDriving();
        sleep(750);


        robotAuto.stopAndReset();
        robotAuto.runUsing();
        robot.fr.setPower(1);
        robot.bl.setPower(0.43);
        robot.br.setPower(-0.43);
        robot.fl.setPower(-1);

        while(robot.fr.getCurrentPosition() < 2500){

        }

        robotAuto.stopDriving();


        //endregion
    }
    class SamplePipeline extends OpenCvPipeline
    {
        public int x1 = 145;
        public int y1 = 455;
        public int x2 = 360;
        public int y2 = 535;
        public int x3 = 410;
        public int y3 = 445;
        public int x4 = 635;
        public int y4 = 545;
        public String verdictL = "wait";
        public String verdictR = "wait";

        public double a1s;
        public double a2s;
        public double a3s;
        public double b1s;
        public double b2s;
        public double b3s;
        public boolean check = false;
        @Override
        public Mat processFrame(Mat input)
        {

            if(check) {
                a1s = 0;
                a2s = 0;
                a3s = 0;
                for (int i = x1 + 2; i <= x2 - 2; i++) {
                    for (int j = y1 + 2; j <= y2 - 2; j++) {
                        a1s += input.get(j, i)[0];
                        a2s += input.get(j, i)[1];
                        a3s += input.get(j, i)[2];

                    }
                }

                a1s /= (x2 - x1 - 3) * (y2 - y1 - 3);
                a2s /= (x2 - x1 - 3) * (y2 - y1 - 3);
                a3s /= (x2 - x1 - 3) * (y2 - y1 - 3);

                if(a1s < 70){
                    verdictL = "skystone";
                } else {
                    verdictL = "stone";
                }


                b1s = 0;
                b2s = 0;
                b3s = 0;
                for (int i = x3 + 2; i <= x4 - 2; i++) {
                    for (int j = y3 + 2; j <= y4 - 2; j++) {
                        b1s += input.get(j, i)[0];
                        b2s += input.get(j, i)[1];
                        b3s += input.get(j, i)[2];

                    }
                }

                b1s /= (x4 - x3 - 3) * (y4 - y3 - 3);
                b2s /= (x4 - x3 - 3) * (y4 - y3 - 3);
                b3s /= (x4 - x3 - 3) * (y4 - y3 - 3);

                if(b1s < 70){
                    verdictR = "skystone";
                } else {
                    verdictR = "stone";
                }

            }
            Imgproc.rectangle(input,new Point(x1,y1),new Point(x2,y2),new Scalar(255,255,255),2);//x:720,y:960
            Imgproc.rectangle(input,new Point(x3,y3),new Point(x4,y4),new Scalar(255,255,255),2);//x:720,y:960



            return input;
        }
    }
}