package org.firstinspires.ftc.teamcode.auto.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.skyAuto;
import org.firstinspires.ftc.teamcode.skyHMAP;

import java.io.File;

@TeleOp
public class skyOdometryCalibrationFake extends LinearOpMode {

    public skyHMAP robot = new skyHMAP();
    public skyAuto robotAuto;



    final double PIVOT_SPEED = 0.2;
    final double COUNTS_PER_INCH = 848.8265;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    File rotationRatioFile = AppUtil.getInstance().getSettingsFile("rotationRatio.txt");

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        double wheelBaseSeparation = 15.35;
        double horizontalTickOffset = 4434;


        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

    }



}
