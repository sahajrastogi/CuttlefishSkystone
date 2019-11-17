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
        robotAuto.grabbersUp();

        robot.fl.setTargetPositionTolerance(50);
        robot.fr.setTargetPositionTolerance(50);
        robot.bl.setTargetPositionTolerance(50);
        robot.br.setTargetPositionTolerance(50);


        robotAuto.stopAndReset();
        robotAuto.runUsing();
        robot.fr.setPower(1);
        robot.bl.setPower(0.37);
        robot.br.setPower(-0.37);
        robot.fl.setPower(-1);

        while(robot.fr.getCurrentPosition() < 1000){

        }

        robotAuto.stopDriving();

        robot.fr.setPower(-0.3);
        robot.bl.setPower(-0.3);
        robot.br.setPower(-0.3);
        robot.fl.setPower(-0.3);

        sleep(1000);


        robotAuto.stopDriving();
        sleep(50);
        robotAuto.driveDistance(0.2,17);
        robotAuto.grabbersDown();

        Thread.sleep(1000);

        robotAuto.driveDistance(-0.2,-17);

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fr.setPower(-1);
        robot.bl.setPower(-0.27);
        robot.br.setPower(0.27);
        robot.fl.setPower(1);
        Thread.sleep(100);

        robotAuto.grabbersUp();
        while(robot.fr.getCurrentPosition() > -3000){

        }
        robotAuto.stopDriving();

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        if(robotAuto.getHeading() > 0){
            robot.fl.setPower(0.1);
            robot.fr.setPower(-0.1);
            robot.bl.setPower(0.1);
            robot.br.setPower(-0.1);

            while(robotAuto.getHeading() >1){

            }
        } else if(robotAuto.getHeading() < 0){
            robot.fl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.bl.setPower(-0.1);
            robot.br.setPower(0.1);

            while(robotAuto.getHeading() <-1){

            }
        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        robotAuto.driveDistance(0.2,9);

        robotAuto.stopAndReset();
        robotAuto.runUsing();
        robot.fr.setPower(1);
        robot.bl.setPower(0.37);
        robot.br.setPower(-0.37);
        robot.fl.setPower(-1);

        while(robot.fr.getCurrentPosition() < 1500){

        }

        robotAuto.stopDriving();

        sleep(50);
        robotAuto.driveDistance(0.1,3);
        sleep(50);


        robotAuto.stopAndReset();
        robotAuto.runUsing();
        robot.fr.setPower(-1);
        robot.bl.setPower(-0.37);
        robot.br.setPower(0.37);
        robot.fl.setPower(1);

        while(robot.fr.getCurrentPosition() > -2000){

        }

        robotAuto.stopDriving();

    }
}
