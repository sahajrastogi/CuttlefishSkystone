package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    public DcMotorEx lift2;


    public DcMotor vl;
    public DcMotor vr;
    public DcMotor h;
    /*Servos*/

    public Servo aR;
    public Servo aL;

    public Servo fgl;
    public Servo fgr;

    public Servo gs;
    public Servo gf;

    public Servo cap;

    public DistanceSensor dist;

    public RevBlinkinLedDriver blinkin;
    HardwareMap hwMap;

    public void init(HardwareMap ahwMap, boolean gyro, boolean distance) {
        hwMap = ahwMap;

        /*Motors*/
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");


        iL = hwMap.get(DcMotorEx.class, "iL");
        iR = hwMap.get(DcMotorEx.class, "iR");
        lift = hwMap.get(DcMotorEx.class,"lift");
        lift2 = hwMap.get(DcMotorEx.class,"lift2");

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
        br.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.FORWARD);


        vl = hwMap.get(DcMotor.class, "iL");
        vr = hwMap.get(DcMotor.class, "br");
        h = hwMap.get(DcMotor.class, "iR");


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

        iL.setDirection(DcMotorEx.Direction.REVERSE);
        iR.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        h.setDirection(DcMotorEx.Direction.REVERSE);

        iL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        iR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        aR = hwMap.servo.get("ar");
        aL = hwMap.servo.get("al");
        /*sC = hwMap.servo.get("servoClaw");*/
        fgr = hwMap.servo.get("fgr");
        fgl = hwMap.servo.get("fgl");

        gs = hwMap.servo.get("gs");
        gf = hwMap.servo.get("gf");

        cap = hwMap.servo.get("cap");

        blinkin = hwMap.get(RevBlinkinLedDriver.class,"l");

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

        if(distance){
            dist = hwMap.get(DistanceSensor.class, "dist");
        }
    }
}

