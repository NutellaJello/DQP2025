package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

/** Blue-side coordinates for the shared close autonomous routine. */
@Disabled
@Autonomous(name = "BLUE Close", group = "Autos")
public class BlueClose extends RedClose {
    @Override
    protected GoalPos createGoal() { return new GoalPos(-147, 143, 15.5); }

    @Override
    protected Pose[] createPoses() {
        return new Pose[] {
                new Pose(-122, 133, Math.toRadians(180)),
                new Pose(-93, 90, Math.toRadians(180)),
                new Pose(-100, 90, Math.toRadians(180)),
                new Pose(-105, 67, Math.toRadians(180)),
                new Pose(-130, 65, Math.toRadians(180)),
                new Pose(-106, 65, Math.toRadians(180)),
                new Pose(-122, 63),
                new Pose(-138, 63, Math.toRadians(150)),
                new Pose(-138, 62.5, Math.toRadians(150)),
                new Pose(-127.5, 88, Math.toRadians(180)),
                new Pose(-108, 77, Math.toRadians(180)) };
    }

    @Override
    protected int targetAprilTagId() { return RobotConstants.BLUE_GOAL_TAG_ID; }

    @Override
    protected double turretCorrectionSign() { return -1; }
}
