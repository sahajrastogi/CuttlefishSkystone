package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.regex.Pattern;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GRAY;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIME;

@TeleOp
public class blinkinledgodmode extends OpMode {
    public RevBlinkinLedDriver blinkin;
    public void init() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"b");

    }

    public void start() {
        telemetry.addData("Hello", 1);
    }

    public void loop() {
        blinkin.setPattern(DARK_GREEN);
    }


}
