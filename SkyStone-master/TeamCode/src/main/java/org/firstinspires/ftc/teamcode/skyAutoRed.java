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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@TeleOp
public class skyAutoRed extends LinearOpMode
{
    OpenCvCamera phoneCam;
    SamplePipeline pipe = new SamplePipeline();
    skyHMAP robot;
    skyAuto robotAuto;
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
    public void runOpMode()
    {
        /*robot = new skyHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);*/

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
        sleep(250);
        pipe.check = false;

        telemetry.addData("verdictL",pipe.verdictL);
        telemetry.addData("verdictR",pipe.verdictR);
        telemetry.update();

        phoneCam.stopStreaming();
        sleep(5000);

    }
    class SamplePipeline extends OpenCvPipeline
    {
        public int x1 = 60;
        public int y1 = 560;
        public int x2 = 300;
        public int y2 = 880;
        public int x3 = 420;
        public int y3 = 560;
        public int x4 = 660;
        public int y4 = 880;
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