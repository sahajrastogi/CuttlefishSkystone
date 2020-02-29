package org.firstinspires.ftc.teamcode.auto.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.skyHMAP;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */

@TeleOp
public class skyOdometryLocalizationTest extends LinearOpMode {


    skyHMAP robot = new skyHMAP();
    skyAuto robotAuto;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,true,false);
        robotAuto = new skyAuto(robot);
        waitForStart();


        skyOdometryGlobalPositionThread globalPos = new skyOdometryGlobalPositionThread(robot,50,Math.PI/2,0,0);
        Thread positionThread = new Thread(globalPos);
        positionThread.start();
        timer.startTime();
        double ly,lx,rx;
        while(opModeIsActive()){

            ly = -gamepad1.left_stick_y;
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            rx*=4;
            rx/=5;
            rx = Math.cbrt(rx);
            mecDrive(ly,lx,rx);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPos.returnXCoordinate());
            telemetry.addData("Y Position", globalPos.returnYCoordinate());
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(globalPos.returnOrientation()));
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.addData("deltatheta",globalPos.deltatheta);
            telemetry.addData("sleepDelay",timer.seconds()/globalPos.count);
            telemetry.update();
        }

        //Stop the thread
        globalPos.stop();
    }
    public void mecDrive(double ly, double lx, double rx) {
        robot.fl.setPower(Range.clip(ly + lx + rx, -1, 1));
        robot.bl.setPower(Range.clip(ly - lx + rx, -1, 1));
        robot.fr.setPower(Range.clip(ly - lx - rx, -1, 1));
        robot.br.setPower(Range.clip(ly + lx - rx, -1, 1));

    }
}