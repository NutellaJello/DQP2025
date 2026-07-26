package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

/** Blue-side coordinates for the shared close autonomous routine. */
@Autonomous(name = "BLUE close but better", group = "Autos")
public class BlueClose2 extends RedClose2 {
    @Override
    protected GoalPos createGoal() { return new GoalPos(-147, 143, 15.5); }

    @Override
    protected Pose[] createPoses() {
        return new Pose[] {
                new Pose(-122, 133, Math.toRadians(180)), // start
                new Pose(-95, 92, Math.toRadians(180)), // outtakePre
                new Pose(-100, 90, Math.toRadians(180)), //outtake
                new Pose(-105, 67, Math.toRadians(180)), //intake1p1
                new Pose(-131, 65, Math.toRadians(180)), //intake1p2
                new Pose(-106, 65, Math.toRadians(180)), //outtake1Point
                new Pose(-122, 63), //gatePoint
                new Pose(-138.5, 63, Math.toRadians(150)), //gate -138，63， deg: 150
                new Pose(-128.5, 90, Math.toRadians(180)), //intake2
                new Pose(-108, 77, Math.toRadians(180)) }; //end
    }

    @Override
    protected int targetAprilTagId() { return RobotConstants.BLUE_GOAL_TAG_ID; }

    @Override
    protected double turretCorrectionSign() { return -1; }
}
