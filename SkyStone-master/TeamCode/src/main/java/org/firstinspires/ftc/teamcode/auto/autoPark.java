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
public class autoPark extends LinearOpMode {

    SamplePipelineBlue pipe = new SamplePipelineBlue();
    OpenCvCamera webcam;
    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;



    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,false,false);
        robotAuto = new skyAuto(robot);

        robot.cap.setPosition(0);

        waitForStart();
        sleep(26000);
        robot.fl.setPower(-0.3);
        robot.br.setPower(-0.3);
        robot.fr.setPower(-0.3);
        robot.bl.setPower(-0.3);
        sleep(500);
        robotAuto.stopDriving();
    }


}




