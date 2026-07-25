package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

/** Blue-side coordinates and timing for the shared far autonomous routine. */
@Autonomous(name = "BLUE Far", group = "Autos")
public class BlueFar extends RedFar {
    @Override
    protected GoalPos createGoal() { return new GoalPos(14, 135, 15.5); }

    @Override
    protected Pose[] createPoses() {
        return new Pose[] { new Pose(56, 0, Math.toRadians(90)), new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(135)),
                new Pose(26, 7.3, Math.toRadians(180)), new Pose(15, 7.3, Math.toRadians(180)),
                new Pose(56, 35, Math.toRadians(180)), new Pose(26, 35, Math.toRadians(180)),
                new Pose(15, 8, Math.toRadians(180)), new Pose(44, 10, Math.toRadians(180)) };
    }

    @Override
    protected int targetAprilTagId() { return RobotConstants.BLUE_GOAL_TAG_ID; }

    @Override
    protected double turretCorrectionSign() { return -1; }

    @Override
    protected double horizontalOffsetIntercept() { return 3; }

    @Override
    protected int firstIntakeWaitMs() { return 2000; }

    @Override
    protected boolean stopIntakeAfterMove() { return true; }

    @Override
    protected boolean stopIntakeOnIntakeTimeout() { return false; }
}
