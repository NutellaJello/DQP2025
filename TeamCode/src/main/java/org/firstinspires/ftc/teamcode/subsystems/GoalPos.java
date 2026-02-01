package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

public class GoalPos {
    static  final double weight = 10; // lower weight -> faster estimate update
    private double a;
    private double b;


    public GoalPos(double X, double Y) {
        a = X;
        b = Y;
    }

    public void setX(double n) {
        a = n;
    }

    public void setY(double n) {
        b = n;
    }

    public double getX() {
        return a;
    }

    public double getY() {
        return b;
    }

    public void update(double x, double y, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double dot = (a - x) * cos + (b - y) * sin;
        double a2 = dot * cos + x;
        double b2 = dot * sin + y;
        a = (a * weight + a2) / (weight + 1);
        b = (b * weight + b2) / (weight + 1);
    }

    public void update(double x, double y, double heading, double dist){
        double a2 = x + dist * Math.cos(heading);
        double b2 = y + dist * Math.sin(heading);
        a = (a * weight + a2) / (weight + 1);
        b = (b * weight + b2) / (weight + 1);
    }

    public double findRange(double x, double y){
        return Math.sqrt(
                Math.pow(a - x, 2) + Math.pow(b - y, 2)
        );
    }
    public double findAngle(double x, double y) {
        return Math.toDegrees(Math.atan2((b - y), (a - x)));
    }

    @NonNull
    public String toString() {
        return "(" + a + "," + b + ")";
    }
}
