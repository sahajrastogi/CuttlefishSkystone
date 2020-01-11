package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */

@TeleOp
public class skyOdometryLocalizationTest extends LinearOpMode {


    skyHMAP robot = new skyHMAP();


    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "br", verticalRightEncoderName = "fl", horizontalEncoderName = "fr";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,true);

        waitForStart();


        skyOdometryGlobalPositionThread globalPos = new skyOdometryGlobalPositionThread(robot,50,Math.PI/2,0,0);
        Thread positionThread = new Thread(globalPos);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPos.returnXCoordinate());
            telemetry.addData("Y Position", globalPos.returnYCoordinate());
            telemetry.addData("Orientation (Degrees)", globalPos.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPos.stop();
    }
}