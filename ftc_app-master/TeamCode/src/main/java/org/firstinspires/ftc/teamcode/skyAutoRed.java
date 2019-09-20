package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class skyAutoRed extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        skyHMAP robot = new skyHMAP();
        robot.init(hardwareMap);


        waitForStart();
        
    }
}
