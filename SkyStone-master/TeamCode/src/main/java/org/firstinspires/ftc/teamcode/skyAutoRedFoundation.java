package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class skyAutoRedFoundation extends LinearOpMode {

    skyHMAP robot;
    skyAuto robotAuto;
    public void runOpMode() throws InterruptedException{
        robot = new skyHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);

        waitForStart();
        robot.autoSmallFlip.setPosition(1);
        robot.fGrabber.setPosition(1);
        robotAuto.grabbersUp();


        robot.fl.setTargetPositionTolerance(50);
        robot.fr.setTargetPositionTolerance(50);
        robot.bl.setTargetPositionTolerance(50);
        robot.br.setTargetPositionTolerance(50);
        robotAuto.driveDistance(0.2,21);
        Thread.sleep(500);

        robotAuto.grabbersDown();
        robotAuto.driveDistance(-0.2,-21);

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(1);
        robot.bl.setPower(0.43);
        robot.br.setPower(-0.43);
        robot.fl.setPower(-1);
        Thread.sleep(100);

        robotAuto.grabbersUp();
        while(robot.fr.getCurrentPosition() < 3500){

        }
        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fl.setPower(0.5);
        robot.bl.setPower(0.5);

        Thread.sleep(300);

        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(1);
        robot.bl.setPower(0.43);
        robot.br.setPower(-0.43);
        robot.fl.setPower(-1);
        Thread.sleep(100);

        robotAuto.grabbersUp();
        while(robot.fr.getCurrentPosition() < 2700){

        }
        robotAuto.stopDriving();
    }
}
