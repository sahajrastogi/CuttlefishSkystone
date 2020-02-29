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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

class SamplePipelineRed extends OpenCvPipeline
{
    public int x1 = 265;
    public int y1 = 120;
    public int x2 = 465;
    public int y2 = 215;
    public int x3 = 510;
    public int y3 = 120;
    public int x4 = 690;
    public int y4 = 210;
    public String verdictL = "wait";
    public String verdictR = "wait";

    public double a1s;
    public double a2s;
    public double a3s;
    public double b1s;
    public double b2s;
    public double b3s;
    public boolean check = true;
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

