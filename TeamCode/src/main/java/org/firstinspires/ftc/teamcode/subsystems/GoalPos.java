package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

public class GoalPos {
    private double a;
    private double b;
    private double c;
    private double camAngle = 20; //degrees

    public GoalPos(double X, double Y, double Z) {
        a = X;
        b = Y;
        c = Z;
    }

    public void setX(double n) {
        a = n;
    }

    public void setY(double n) {
        b = n;
    }

    public void setZ(double n) {
        c = n;
    }

    public double getX() {
        return a;
    }

    public double getY() {
        return b;
    }

    public double getZ(){
        return c;
    }

//    public void update(double weight, double x, double y, double heading, double elevation) {
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//        double dot = (a - x) * cos + (b - y) * sin;
//        double a2 = dot * cos + x;
//        double b2 = dot * sin + y;
//        a = (a * weight + a2) / (weight + 1);
//        b = (b * weight + b2) / (weight + 1);
//    }

    public void update(double alpha, double x, double y, double heading, double elevation, double dist){
        dist *= 1.1076;
        elevation += Math.toRadians(camAngle);
        double phi = Math.PI/2 - elevation;
        phi = Range.clip(phi, 0, Math.PI);
        double a2 = x + dist * Math.cos(heading) * Math.sin(phi);
        double b2 = y + dist * Math.sin(heading) * Math.sin(phi);
        double c2 = dist * Math.cos(phi);
        if(Double.isFinite(a2) && Double.isFinite(b2) && Double.isFinite(c2)) {
            a = a * (1 - alpha) + a2 * alpha;
            b = b * (1 - alpha) + b2 * alpha;
            c = c * (1 - alpha) + c2 * alpha;
        }
        if (c < 18){
            c = 18;
        }
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
        return "(" + Math.round(a * 100)/100.0 + "," + Math.round(b * 100)/100.0 + "," + Math.round(c * 100)/100.0 + ")";
    }
}
