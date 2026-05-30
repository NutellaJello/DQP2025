package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

public class GoalPos {
    private double goalX;
    private double goalY;
    private double goalZ;
    private double camElevationAngleDeg = 20; // degrees

    public GoalPos(double X, double Y, double Z) {
        goalX = X;
        goalY = Y;
        goalZ = Z;
    }

    public void setX(double n) { goalX = n; }
    public void setY(double n) { goalY = n; }
    public void setZ(double n) { goalZ = n; }

    public double getX() { return goalX; }
    public double getY() { return goalY; }
    public double getZ() { return goalZ; }

    public void update(double alpha, double x, double y, double heading, double elevation, double dist) {
        elevation += Math.toRadians(camElevationAngleDeg);
        double phi = Math.PI / 2 - elevation;
        phi = Range.clip(phi, 0, Math.PI);
        double x2 = x + dist * Math.cos(heading) * Math.sin(phi);
        double y2 = y + dist * Math.sin(heading) * Math.sin(phi);
        double z2 = dist * Math.cos(phi);
        goalX = goalX * (1 - alpha) + x2 * alpha;
        goalY = goalY * (1 - alpha) + y2 * alpha;
        goalZ = goalZ * (1 - alpha) + z2 * alpha;
        if (goalZ < 18) goalZ = 18;
    }

    public double findRange(double x, double y) {
        return Math.sqrt(Math.pow(goalX - x, 2) + Math.pow(goalY - y, 2));
    }

    public double findBearing(double x, double y) {
        return Math.toDegrees(Math.atan2((goalY - y), (goalX - x)));
    }

    @NonNull
    public String toString() {
        return "(" + Math.round(goalX * 100) / 100.0 + "," + Math.round(goalY * 100) / 100.0 + "," + Math.round(goalZ * 100) / 100.0 + ")";
    }
}
