package org.firstinspires.ftc.teamcode.auto.autoFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.skyHMAP;

import static java.lang.Thread.sleep;

public class depositThread implements Runnable{

    public skyHMAP robot;
    public skyAuto robotAuto;
    public int counter = 0;
    public static boolean releaseReady = false;

    public depositThread(skyHMAP robot){
        this.robot = robot;
        robotAuto = new skyAuto(robot);
        releaseReady = false;
    }

    @Override
    public void run() {
        if (counter == 0) {
            robotAuto.depositPos();
            try {
                sleep(700);
            } catch (Exception e) {
                e.printStackTrace();
            }

            robotAuto.release();
            try {
                sleep(250);
            } catch (Exception e) {
                e.printStackTrace();
            }
            robotAuto.intakePos();
            try {
                sleep(450);
            } catch (Exception e) {
                e.printStackTrace();
            }
            releaseReady = true;
        }
    }
}
