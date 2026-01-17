package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

public class GoalPos {
    private double a;
    private double b;

    public GoalPos(double X, double Y){
        a = X;
        b = Y;
    }

    public void setX(double n){
        a = n;
    }

    public void setY(double n){
        b = n;
    }

    public double getX(){
        return a;
    }

    public double getY(){
        return b;
    }

    public GoalPos update(double x, double y, double heading){
        final double weight = 10; // lower weight -> faster estimate update
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double dot = (a - x) * cos + (b - y) * sin;
        double a2 = dot * cos + x;
        double b2 = dot * sin + y;
        double aAvg = (a * weight + a2)/(weight + 1);
        double bAvg = (b * weight + b2)/(weight + 1);
        return new GoalPos(aAvg, bAvg);
    }

    public double findAngle(double x, double y){
        double ang = Math.toDegrees(Math.atan((b - y)/(a - x)));
        if ((a - x) >= 0){
            return ang;
        }
        return ang + 180;
    }

    @NonNull
    public String toString(){
        return "(" + a + "," + b + ")";
    }
}
