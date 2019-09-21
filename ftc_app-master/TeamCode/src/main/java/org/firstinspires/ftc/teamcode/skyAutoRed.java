package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class skyAutoRed extends LinearOpMode {

    skyHMAP robot;
    skyAuto robotAuto;
    public void runOpMode() throws InterruptedException{
        robot = new skyHMAP();
        robot.init(hardwareMap);
        robotAuto = new skyAuto(robot);

        waitForStart();

    }
}
