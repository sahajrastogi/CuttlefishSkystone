package org.firstinspires.ftc.teamcode.auto.autoFunctions;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double followAngle;
    public boolean isStopPoint;


    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double followAngle, boolean isStopPoint){
        this.x=x;
        this.y=y;
        this.moveSpeed= moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.followAngle = followAngle;
        this.isStopPoint = isStopPoint;

    }

    public CurvePoint(CurvePoint p){
        x = p.x;
        y = p.y;
        moveSpeed = p.moveSpeed;
        turnSpeed = p.turnSpeed;
        followDistance = p.followDistance;
        followAngle = p.followAngle;
        isStopPoint = p.isStopPoint;

    }

    public Point toPoint(){
        return new Point(x,y);
    }

    public void setPoint(Point point){
        x = point.x;
        y = point.y;
    }
}
