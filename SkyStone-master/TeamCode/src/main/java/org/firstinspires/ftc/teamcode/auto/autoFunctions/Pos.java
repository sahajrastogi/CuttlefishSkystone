package org.firstinspires.ftc.teamcode.auto.autoFunctions;

public class Pos {
    public double x,y,theta;
    public Pos(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public String toString(){
        return "" + x + " " + y + " " + theta;
    }
}
