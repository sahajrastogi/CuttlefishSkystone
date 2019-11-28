package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class skyTeleOp extends OpMode {

    public skyHMAP robot = new skyHMAP();
    private double ly,lx, rx;
    public Gamepad gamepad1;
    private int liftPos;

    public void init()
    {
        robot.init(hardwareMap,false);
    }

    public void start(){
        telemetry.addData("Hello",1);
    }

    public void loop() {
        if(gamepad1.left_stick_y>0){
            robot.iL.setPower(1);
            robot.iR.setPower(1);
        }
        if(gamepad1.left_stick_y<0){
          robot.iL.setPower(-1);
          robot.iR.setPower(-1);
        }


         if(gamepad1.dpad_up){
             robot.lift.setPower(1);
         }
         else if(gamepad1.dpad_down){
             robot.lift.setPower(-1);
         }
         else{
              robot.lift.getCurrentPosition();

             robot.lift.setTargetPosition(liftPos);
         }

        ly = -Math.pow(gamepad1.left_stick_y,3);
        lx = gamepad1.left_stick_x;
        rx = Math.pow(gamepad1.right_stick_x,3);
        rx/=3;
        rx*=2;


        if(gamepad1.right_trigger > 0.1){
            ly/=3;
            rx/=3;

        }

        mecDrive(ly, lx, rx);

        }
    public void mecDrive(double ly, double lx, double rx){
        robot.fl.setPower(Range.clip(ly + 1*lx + rx,-1,1));
        robot.bl.setPower(Range.clip(ly - 1*lx + rx,-1,1));
        robot.fr.setPower(Range.clip(ly - 1*lx - rx,-1,1));
        robot.br.setPower(Range.clip(ly + 1*lx - rx,-1,1));
    }

}




