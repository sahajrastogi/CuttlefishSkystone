package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class skyAutoRed extends LinearOpMode {

    skyHMAP robot;
    skyAuto robotAuto;
    public void runOpMode() throws InterruptedException{
        robot = new skyHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);

        waitForStart();
        robot.autoBigFlip.setPosition(0.11);
        robot.autoSmallFlip.setPosition(0);

        //region drive forward and rotate
        robotAuto.driveDistance(-0.3,-13);

        Thread.sleep(50);
        robotAuto.runUsing();
        robot.fr.setPower(0.15);
        robot.br.setPower(0.15);
        robot.fl.setPower(-0.15);
        robot.bl.setPower(-0.15);
        while(robotAuto.getHeading() < 86){
            if(robotAuto.getHeading() > 70){
                robot.fr.setPower(0.05);
                robot.br.setPower(0.05);
                robot.fl.setPower(-0.05);
                robot.bl.setPower(-0.05);
            }
            telemetry.addData("heading",robotAuto.getHeading());
            telemetry.update();
        }

        robotAuto.stopDriving();
        robotAuto.stopAndReset();
        Thread.sleep(50);
        //endregion

        //region strafe and align
        robotAuto.runUsing();

        while(robot.fr.getCurrentPosition() < 1200) {
            robot.fr.setPower(1);
            robot.bl.setPower(0.4);
            robot.br.setPower(-0.4);
            robot.fl.setPower(-1);
        }
        Thread.sleep(50);
        telemetry.addData("heading",robotAuto.getHeading());
        telemetry.update();

        if(robotAuto.getHeading() < 90){
            while(robotAuto.getHeading() <88){
                robot.fr.setPower(0.05);
                robot.br.setPower(0.05);
                robot.fl.setPower(-0.05);
                robot.bl.setPower(-0.05);
            }

        } else if(robotAuto.getHeading() > 90) {
            while(robotAuto.getHeading() > 92){
                robot.fr.setPower(-0.05);
                robot.br.setPower(-0.05);
                robot.fl.setPower(0.05);
                robot.bl.setPower(0.05);
            }
        }
        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region grab, strafe out, and align
        robot.autoBigFlip.setPosition(0.0);
        robot.autoSmallFlip.setPosition(1);
        Thread.sleep(1000);
        robot.autoBigFlip.setPosition(0.7);
        Thread.sleep(500);

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robot.fl.getCurrentPosition() < 2500) {
            robot.fr.setPower(-1);
            robot.bl.setPower(-0.35);
            robot.br.setPower(0.35);
            robot.fl.setPower(1);
        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        robot.autoBigFlip.setPosition(0.35);

        telemetry.addData("heading",robotAuto.getHeading());
        telemetry.update();

        Thread.sleep(50);
        if(robotAuto.getHeading() < 90){
            while(robotAuto.getHeading() <84){
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
                robot.fl.setPower(-0.1);
                robot.bl.setPower(-0.1);
            }

        } else if(robotAuto.getHeading() > 90) {
            while(robotAuto.getHeading() > 96){
                robot.fr.setPower(-0.1);
                robot.br.setPower(-0.1);
                robot.fl.setPower(0.1);
                robot.bl.setPower(0.1);
            }
        }
        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region go forward, strafe in and place block
        robotAuto.driveDistance(0.5,49);

        Thread.sleep(150);
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robot.fr.getCurrentPosition() < 2500) {
            robot.fr.setPower(1);
            robot.bl.setPower(0.4);
            robot.br.setPower(-0.4);
            robot.fl.setPower(-1);
        }
        robotAuto.stopDriving();
        robot.autoSmallFlip.setPosition(0);
        Thread.sleep(500);
        robot.autoBigFlip.setPosition(0);
        Thread.sleep(1000);
        //endregion

        //region strafe back
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robot.fl.getCurrentPosition() < 500) {
            robot.fr.setPower(-1);
            robot.bl.setPower(-0.3);
            robot.br.setPower(0.3);
            robot.fl.setPower(1);
        }
        robotAuto.stopDriving();
        Thread.sleep(50);
        robotAuto.stopAndReset();
        robotAuto.runUsing();

        while(robot.fl.getCurrentPosition() < 5000) {
            robot.fr.setPower(-1);
            robot.bl.setPower(-0.1);
            robot.br.setPower(0.1);
            robot.fl.setPower(1);
        }
        //endregion


        //park

    }
}
