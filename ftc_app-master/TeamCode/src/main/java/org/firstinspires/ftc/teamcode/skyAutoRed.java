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

        robotAuto.gyroTurnRobotRight(90, 0.2);

        //drop arms
        //strafe in with gyro thingy

        //pick up

        //go forward

        //drop

        //go backward

        //align bitch

        // pick up

        //go forward

        //drop

        //align for foundation

        //do foundation

        //park

    }
}
