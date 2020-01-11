package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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


    public DcMotor vl;
    public DcMotor vr;
    public DcMotor h;
    /*Servos*/
    public Servo aR;
    public Servo aL;
    public Servo sC;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap, boolean gyro) {
        hwMap = ahwMap;

        /*Motors*/
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");


        /*iL = hwMap.get(DcMotorEx.class, "iL");
        iR = hwMap.get(DcMotorEx.class, "iR");
        lift = hwMap.get(DcMotorEx.class,"lift");*/

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);


        vl = hwMap.get(DcMotor.class, "fl");
        vr = hwMap.get(DcMotor.class, "fr");
        h = hwMap.get(DcMotor.class, "bl");


        vl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        vl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        h.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //defaults:
        //forward is toward intake wheels
        //forward is positive ticks for both vertical encoders
        //right is positive ticks for horizontal encoder



        /*iL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        iR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        aR = hwMap.servo.get("axelRight");
        aL = hwMap.servo.get("axelLeft");
        sC = hwMap.servo.get("servoClaw");*/


        if(gyro) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
    }


}