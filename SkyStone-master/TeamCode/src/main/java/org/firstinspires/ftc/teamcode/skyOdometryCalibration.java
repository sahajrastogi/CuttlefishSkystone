package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp
public class skyOdometryCalibration extends LinearOpMode {

    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;



    final double PIVOT_SPEED = 0.5;
    final double COUNTS_PER_INCH = 848.8265;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    File rotationRatioFile = AppUtil.getInstance().getSettingsFile("rotationRatio.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,true);
        robotAuto = new skyAuto(robot);
        timer.startTime();


        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(robotAuto.getZAngle() > -90 && opModeIsActive()){
            robot.fr.setPower(-PIVOT_SPEED);
            robot.br.setPower(-PIVOT_SPEED);
            robot.fl.setPower(PIVOT_SPEED);
            robot.bl.setPower(PIVOT_SPEED);

            telemetry.addData("IMU Angle", robotAuto.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        robotAuto.stopDriving();
        timer.reset();
        while(timer.milliseconds() < 2500 && opModeIsActive()){
            telemetry.addData("IMU Angle", robotAuto.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robotAuto.getZAngle();


     //do calcs
        double rotationRatio = Math.abs((double)robot.vl.getCurrentPosition()/(double)robot.vr.getCurrentPosition());
        double encoderDifference = Math.abs(robot.vl.getCurrentPosition()) + (Math.abs(robot.vr.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = Math.abs(encoderDifference/angle);

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = robot.h.getCurrentPosition()/(Math.toRadians(robotAuto.getZAngle()));
        horizontalTickOffset = Math.abs(horizontalTickOffset);
        //Write the constants to text files
        ReadWriteFile.writeFile(rotationRatioFile,String.valueOf(rotationRatio));
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("encoderDifference",encoderDifference);
            telemetry.addData("verticalEncoderTickOffsetPerDegree",verticalEncoderTickOffsetPerDegree);
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);
            telemetry.addData("rotationRatio",rotationRatio);


            //Display raw values
            telemetry.addData("IMU Angle", robotAuto.getZAngle());
            telemetry.addData("Vertical Left Position", robot.vl.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.vr.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.h.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }



}
