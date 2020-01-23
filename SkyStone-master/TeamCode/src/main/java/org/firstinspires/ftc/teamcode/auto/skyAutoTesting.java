package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.CurvePoint;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.auto.odometry.skyOdometryGlobalPositionThread;
import org.firstinspires.ftc.teamcode.skyHMAP;

import java.util.ArrayList;

@TeleOp
public class skyAutoTesting extends LinearOpMode {

    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;
    private double ly,lx, rx;
    public DcMotorEx odo;


    public void runOpMode() {
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);

        ArrayList<CurvePoint> allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(0.0,0.0,1.0,1.0,5,1,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(40.0,40.0,1.0,1.0,5,1,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(40.0,110.0,1.0,1.0,5,1,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(80.0,110.0,1.0,1.0,5,1,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(80.0,20.0,1.0,1.0,5,1,Math.toRadians(50),1.0));

        waitForStart();

        skyOdometryGlobalPositionThread globalPos = new skyOdometryGlobalPositionThread(robot,50,Math.PI/2,0,0);
        Thread positionThread = new Thread(globalPos);
        positionThread.start();

        while(opModeIsActive()) {
            robotAuto.followCurve(allPoints, globalPos.returnPos(), Math.toRadians(90));
            telemetry.addData("pos",globalPos.returnPos().toString());
            telemetry.update();
        }

    }


}




