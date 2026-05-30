package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red 15", group = "Autos")
public class RedClose15 extends BaseAuto {

    private DcMotorEx flyWheel2;
    private final double lowLimit = -990;
    private final double highLimit = 850;

    private enum PathState {
        PRELOAD, SHOOTPRE,
        INTAKE11, INTAKE12, OUTTAKE1, SHOOT1,
        OPENGATE, BIGBACK, OUTTAKEB, SHOOTB,
        INTAKE2, OUTTAKE2, SHOOT2,
        INTAKE31, INTAKE32, OUTTAKE3, SHOOT3,
        END, STOP
    }

    private PathState pathState;

    // Poses
    private final Pose start        = new Pose(120, 133, Math.toRadians(0));
    private final Pose outtakePre   = new Pose(93, 90, Math.toRadians(0));
    private final Pose outtake      = new Pose(100, 90, Math.toRadians(0));
    private final Pose intake1p1    = new Pose(105, 67, Math.toRadians(0));
    private final Pose intake1p2    = new Pose(130, 65, Math.toRadians(0));
    private final Pose outtake1Point= new Pose(106, 65, Math.toRadians(0));
    private final Pose gatePoint    = new Pose(122, 46, Math.toRadians(46));
    private final Pose gate         = new Pose(133, 65.5, Math.toRadians(20));
    private final Pose bigBack      = new Pose(142, 53, Math.toRadians(46));
    private final Pose bigBackPoint = new Pose(133, 50, Math.toRadians(46));
    private final Pose outtakeBPoint= new Pose(100, 60, Math.toRadians(0));
    private final Pose intake2      = new Pose(127, 88, Math.toRadians(0));
    private final Pose intake3p1    = new Pose(100, 45, Math.toRadians(0));
    private final Pose intake3p2    = new Pose(130, 39, Math.toRadians(0));
    private final Pose end          = new Pose(108, 77, Math.toRadians(0));

    // Paths (bigBack/intake2/end shadow Pose fields, so PathChains use distinct names)
    private PathChain preload, intake11, intake12, outtake1;
    private PathChain openGate, bigBackPath, outtakeB;
    private PathChain intake2Path, outtake2;
    private PathChain intake31, intake32, outtake3;
    private PathChain endPath;

    @Override protected double getPIDFP()       { return 400; }
    @Override protected GoalPos createGoalPos() { return new GoalPos(147, 143, 15.5); }
    @Override protected Pose getStartPose()     { return start; }
    @Override protected double getFWVConstant() { return 1150; }

    @Override
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        intake11 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1p1))
                .setConstantHeadingInterpolation(0)
                .build();
        intake12 = follower.pathBuilder()
                .addPath(new BezierLine(intake1p1, intake1p2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(intake1p2, outtake1Point, outtake)))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(outtake, gatePoint, gate)))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate.getHeading())
                .setBrakingStrength(0.08)
                .build();
        bigBackPath = follower.pathBuilder()
                .addPath(new BezierLine(gate, bigBack))
                .setLinearHeadingInterpolation(gate.getHeading(), bigBack.getHeading())
                .build();
        outtakeB = follower.pathBuilder()
                .addPath(new BezierCurve(bigBack, outtakeBPoint, outtake))
                .setLinearHeadingInterpolation(gate.getHeading(), outtake.getHeading())
                .build();
        intake2Path = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        intake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(0)
                .build();
        intake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(0)
                .build();
        outtake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    protected void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        initHardware();
        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
    }

    @Override
    public void stop() {
        flyWheel2.setVelocity(0);
        super.stop();
    }

    @Override
    public void statePathUpdate() {
        telemetry.update();
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();

        switch (pathState) {
            case PRELOAD:
                flyWheel2.setVelocity(1100);
                move(preload, () -> setPathState(PathState.SHOOTPRE), true, false);
                break;
            case SHOOTPRE:
                shoot(PathState.INTAKE11);
                break;
            case INTAKE11:
                move(intake11, () -> setPathState(PathState.INTAKE12), false, false);
                break;
            case INTAKE12:
                moveIntake(intake12, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE1));
                break;
            case OUTTAKE1:
                flyWheel2.setVelocity(1100);
                move(outtake1, () -> setPathState(PathState.SHOOT1), true, false);
                break;
            case SHOOT1:
                shoot(PathState.OPENGATE);
                break;
            case OPENGATE:
                move(openGate, () -> setPathState(PathState.BIGBACK), false, false);
                break;
            case BIGBACK:
                moveIntake(bigBackPath, 0.65, false, 50, () -> setPathState(PathState.OUTTAKEB));
                break;
            case OUTTAKEB:
                flyWheel2.setVelocity(1100);
                move(outtakeB, () -> setPathState(PathState.SHOOTB), true, false);
                break;
            case SHOOTB:
                shoot(PathState.INTAKE2);
                break;
            case INTAKE2:
                moveIntake(intake2Path, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE2));
                break;
            case OUTTAKE2:
                flyWheel2.setVelocity(1100);
                move(outtake2, () -> setPathState(PathState.SHOOT2), true, false);
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31);
                break;
            case INTAKE31:
                move(intake31, () -> setPathState(PathState.INTAKE32), false, false);
                break;
            case INTAKE32:
                moveIntake(intake32, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE3));
                break;
            case OUTTAKE3:
                flyWheel2.setVelocity(1100);
                move(outtake3, () -> setPathState(PathState.SHOOT3), true, false);
                break;
            case SHOOT3:
                shoot(PathState.END);
                break;
            case END:
                move(endPath, () -> setPathState(PathState.STOP), false, false);
                break;
        }

        if (pathState != PathState.END && pathState != PathState.STOP
                && opmodeTimer.getElapsedTimeSeconds() < 29.5) {
            aiming(detectedTags);
        } else {
            intake.setPower(0);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
        }
    }

    private void shoot(PathState nextPath) {
        double targetV = toFWV(range);
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
        if (range < 40) {
            flapPos = 0;
        } else if (range < 95) {
            flapPos = 0.2;
        } else {
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);
        double fwv = flyWheel1.getVelocity();
        if (fwv >= targetV) {
            stopper.setPosition(0.973);
            intake.setPower(1);
        }
        if (actionTimer.getElapsedTime() > 1600) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    private void aiming(List<AprilTagDetection> detectedTags) {
        range = goalPos.findRange(xPos, yPos);
        if (gainSet) {
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) {
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(3.2 / range));
                    double elevation = detection.ftcPose.elevation;
                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;
                    bearing = Math.toRadians(bearing);
                    elevation = Math.toRadians(elevation);
                    goalPos.update(0.08, xPos, yPos, bearing, elevation, camRange);
                    break;
                }
            }
        }
        double turretTarget = goalPos.findBearing(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading);
        if (turretTarget > 180 + 30) turretTarget -= 360;
        else if (turretTarget < -180 - 30) turretTarget += 360;
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
}
