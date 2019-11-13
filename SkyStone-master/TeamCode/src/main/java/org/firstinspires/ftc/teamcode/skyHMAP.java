package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


public class  skyHMAP {

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    double P_DRIVE_COEFF = 0.02;     // Larger is more responsive, but also less stable
    public final double ticksPerInch = 72.1;

    /*Motors*/
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    public DcMotorEx iL;
    public DcMotorEx iR;

    public DcMotorEx lift;
    public DcMotorEx teleopRotate;

    public Servo autoBigFlip;
    public Servo autoSmallFlip;
    public Servo teleopClawLeft;
    public Servo teleopClawRight;
    public Servo fGrabber;
    public Servo fGrabber2;

    public Servo capstone;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap, boolean gyro) {
        hwMap = ahwMap;

        /*Motors*/
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");

        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

        iL = hwMap.get(DcMotorEx.class, "il");
        iR = hwMap.get(DcMotorEx.class, "ir");
        lift = hwMap.get(DcMotorEx.class, "lift");
        teleopRotate = hwMap.get(DcMotorEx.class,"tr");


        autoBigFlip = hwMap.get(Servo.class, "abf");
        autoSmallFlip = hwMap.get(Servo.class,"asf");
        teleopClawLeft = hwMap.get(Servo.class, "tcl");
        teleopClawRight = hwMap.get(Servo.class, "tcr");
        fGrabber = hwMap.get(Servo.class,"fg");
        fGrabber2 = hwMap.get(Servo.class,"fg2");
        capstone = hwMap.get(Servo.class,"c");

        if(gyro) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            //parameters.angleUnit = HardwareType.BNO055IMU.AngleUnit.DEGREES;
            //parameters.accelUnit = HardwareType.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
    }


}