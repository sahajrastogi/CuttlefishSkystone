package org.firstinspires.ftc.teamcode.auto.autoFunctions;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.skyHMAP;

@TeleOp
public class depositThreadTest  extends OpMode {

    public skyHMAP robot = new skyHMAP();
    public depositThread dep;
    public Thread depThread;

    public void init(){
        robot.init(hardwareMap,false,false);
        dep = new depositThread(robot);
        depThread = new Thread(dep);
        depThread.start();
    }

    public void loop(){
    }
}
