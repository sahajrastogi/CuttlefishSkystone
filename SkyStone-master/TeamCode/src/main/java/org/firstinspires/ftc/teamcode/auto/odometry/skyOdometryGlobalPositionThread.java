package org.firstinspires.ftc.teamcode.auto.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.skyHMAP;
import org.firstinspires.ftc.teamcode.auto.autoFunctions.Pos;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class skyOdometryGlobalPositionThread implements Runnable{

    skyHMAP robot = new skyHMAP();
    private boolean isRunning = true;

    final double COUNTS_PER_INCH = 848.8265;

    //Position variables used for storage and calculations
    public double count;
    public double vrpos = 0, vlpos = 0, hpos = 0,  deltatheta = 0;
    public double globalx = 0, globaly = 0, orientation = 0;//in radians
    public double prevVrpos = 0, prevVlpos = 0, prevHpos = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;
    private double rotationRatio;

    private int sleepTime;

    //Files to access the algorithm constants
    private File rotationRatioFile = AppUtil.getInstance().getSettingsFile("rotationRatio.txt");

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    public skyOdometryGlobalPositionThread(skyHMAP robot, int threadSleepDelay, double initialOrientation, double initialX, double initialY){
        this.robot = robot;
        sleepTime = threadSleepDelay;
        globalx  = initialX;
        globaly = initialY;
        orientation = initialOrientation;

        rotationRatio = Double.parseDouble(ReadWriteFile.readFile(rotationRatioFile).trim());
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    private void globalCoordinatePositionUpdate(){
        vrpos = robot.vr.getCurrentPosition();
        vlpos = robot.vl.getCurrentPosition();
        hpos = robot.h.getCurrentPosition();

        double changevr = vrpos - prevVrpos;
        double changevl = vlpos - prevVlpos;
        double changehpos = hpos - prevHpos;

        deltatheta = (changevr - changevl) / (robotEncoderWheelDistance);
        double realchangehpos = changehpos + deltatheta*horizontalEncoderTickPerDegreeOffset;

        double center = (changevl + changevr)/2;
        globalx += center*Math.cos(orientation) + realchangehpos*Math.sin(orientation + deltatheta/2);
        globaly += center*Math.sin(orientation) - realchangehpos*Math.cos(orientation + deltatheta/2);
        orientation += deltatheta;
        prevVrpos = vrpos;
        prevVlpos = vlpos;
        prevHpos = hpos;
        count++;
    }


    public double returnXCoordinate(){ return globalx/COUNTS_PER_INCH; }

    public double returnYCoordinate(){ return globaly/COUNTS_PER_INCH; }


    public double returnOrientation(){ return orientation;}

    public Pos returnPos(){
        return new Pos(returnXCoordinate(),returnYCoordinate(),returnOrientation());
    }

    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
        }
    }
}
