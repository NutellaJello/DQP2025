package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

/** Blue-alliance configuration of the shared teleop controls. */
@TeleOp(name = "Blue Teleop", group = "TeleOp")
public class BlueTeleop extends RedTeleop {

    @Override
    protected GoalPos createGoal() {
        return new GoalPos(30, -50, 15.5);
    }

    @Override
    protected double gateAngle() {
        return Math.toRadians(-30);
    }

    @Override
    protected int targetAprilTagId() {
        return RobotConstants.BLUE_GOAL_TAG_ID;
    }

    @Override
    protected double horizontalCorrectionSign() {
        return 1;
    }

    @Override
    protected double[] hOffsetConstants(){
        return new double[]{1.0,3.0};
    }

    @Override
    protected boolean showVoltageTelemetry() {
        return false;
    }
}
