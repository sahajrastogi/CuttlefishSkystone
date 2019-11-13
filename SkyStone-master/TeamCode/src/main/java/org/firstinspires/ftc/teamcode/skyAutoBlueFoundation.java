package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class skyAutoBlueFoundation extends LinearOpMode {

    skyHMAP robot;
    skyAuto robotAuto;
    public void runOpMode() throws InterruptedException{
        robot = new skyHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);

        waitForStart();
        robot.autoSmallFlip.setPosition(1);
        robot.fGrabber.setPosition(1);
        robot.fGrabber2.setPosition(0);
        robot.autoBigFlip.setPosition(1);


        robot.fl.setTargetPositionTolerance(50);
        robot.fr.setTargetPositionTolerance(50);
        robot.bl.setTargetPositionTolerance(50);
        robot.br.setTargetPositionTolerance(50);
        robotAuto.driveDistance(0.2,21);
        Thread.sleep(500);
        robot.fGrabber.setPosition(0);
        robot.fGrabber2.setPosition(1);
        Thread.sleep(1000);
        robotAuto.driveDistance(-0.2,-22         );

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(-1);
        robot.bl.setPower(-0.3);
        robot.br.setPower(0.2);
        robot.fl.setPower(0.95);
        Thread.sleep(100);
        robot.fGrabber.setPosition(1);
        robot.fGrabber2.setPosition(0);
        while(robot.fl.getCurrentPosition() < 3500){

        }
        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(0.5);
        robot.br.setPower(0.5);

        Thread.sleep(300);

        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(-1);
        robot.bl.setPower(-0.3);
        robot.br.setPower(0.2);
        robot.fl.setPower(0.95);
        Thread.sleep(100);
        robot.fGrabber.setPosition(1);
        robot.fGrabber2.setPosition(0);
        while(robot.fl.getCurrentPosition() < 4500){

        }
        robotAuto.stopDriving();
    }
}
