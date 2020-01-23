package org.firstinspires.ftc.teamcode.auto.autoFunctions;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount){
        this.x=x;
        this.y=y;
        this.moveSpeed= moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this. slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint p){
        x = p.x;
        y = p.y;
        moveSpeed = p.moveSpeed;
        turnSpeed = p.turnSpeed;
        followDistance = p.followDistance;
        pointLength = p.pointLength;
        slowDownTurnAmount = p. slowDownTurnAmount;
        slowDownTurnRadians = p.slowDownTurnRadians;
    }

    public Point toPoint(){
        return new Point(x,y);
    }

    public void setPoint(Point point){
        x = point.x;
        y = point.y;
    }
}
