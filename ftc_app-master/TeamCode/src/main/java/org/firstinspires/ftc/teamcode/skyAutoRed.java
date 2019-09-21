package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class skyAutoRed extends LinearOpMode {

    skyHMAP robot;
    skyAuto robotAuto;
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        robot = new skyHMAP();
        robotAuto = new skyAuto(robot);

        waitForStart();

        robotAuto.driveDistance(0.5,10);


        //some software
    }
}
