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
    private boolean shooting = false;
    private double minFWVSinceOpen = Double.MAX_VALUE;
    private long stopperOpenTime = 0;
    private double elevation;
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

    private final Pose start          = new Pose(120, 133, Math.toRadians(0));
    private final Pose outtakePre     = new Pose(93,  90,  Math.toRadians(0));
    private final Pose outtake        = new Pose(100, 90,  Math.toRadians(0));
    private final Pose intake1p1      = new Pose(105, 67,  Math.toRadians(0));
    private final Pose intake1p2      = new Pose(130, 67 - 2, Math.toRadians(0));
    private final Pose outtake1Point  = new Pose(106, 65,  Math.toRadians(0));
    private final Pose gatePoint      = new Pose(122, 46,  Math.toRadians(46));
    private final Pose gate           = new Pose(133, 65.5, Math.toRadians(20));
    private final Pose bigBack        = new Pose(142, 53,  Math.toRadians(46));
    private final Pose bigBackPoint   = new Pose(133, 50,  Math.toRadians(46));
    private final Pose outtakeBPoint  = new Pose(100, 60,  Math.toRadians(0));
    private final Pose intake2        = new Pose(127, 88,  Math.toRadians(0));
    private final Pose intake3p1      = new Pose(100, 45,  Math.toRadians(0));
    private final Pose intake3p2      = new Pose(130, 45 - 6, Math.toRadians(0));
    private final Pose end            = new Pose(108, 77,  Math.toRadians(0));

    private PathChain Preload, Intake11, Intake12, Outtake1;
    private PathChain Opengate, BigBack, OuttakeB;
    private PathChain Intake2, Outtake2;
    private PathChain Intake31, Intake32, Outtake3;
    private PathChain End;

    @Override protected double getPIDFP()        { return 400; }
    @Override protected GoalPos createGoalPos()  { return new GoalPos(147, 143, 15.5); }
    @Override protected Pose getStartPose()       { return start; }
    @Override protected double getFWVConstant()   { return 1150; }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        baseInit();
        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
    }

    @Override
    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .setGlobalDeceleration(0.9)
                .build();
        Intake11 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1p1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Intake12 = follower.pathBuilder()
                .addPath(new BezierLine(intake1p1, intake1p2))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(intake1p2, outtake1Point, outtake)))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .setGlobalDeceleration(0.9)
                .build();
        Opengate = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(outtake, gatePoint, gate)))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate.getHeading())
                .setBrakingStrength(0.08)
                .setGlobalDeceleration(0.9)
                .build();
        BigBack = follower.pathBuilder()
                .addPath(new BezierLine(gate, bigBack))
                .setLinearHeadingInterpolation(gate.getHeading(), bigBack.getHeading())
                .setGlobalDeceleration(0.9)
                .build();
        OuttakeB = follower.pathBuilder()
                .addPath(new BezierCurve(bigBack, outtakeBPoint, outtake))
                .setLinearHeadingInterpolation(gate.getHeading(), outtake.getHeading())
                .setGlobalDeceleration(0.9)
                .build();
        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Intake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Intake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
    }

    protected void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.resetTimer();
    }

    /** Move with idle flywheel spin-up (pre-spins to 1100 ticks/s before arriving at shoot position). */
    public void moveIdle(PathChain path, Runnable onComplete) {
        if (!moving) {
            flyWheel1.setVelocity(1100);
            flyWheel2.setVelocity(1100);
            follower.followPath(path, false);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            moving = false;
            onComplete.run();
        }
    }

    @Override
    public void statePathUpdate() {
        telemetry.update();
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();

        switch (pathState) {
            case PRELOAD:
                moveIdle(Preload, () -> setPathState(PathState.SHOOTPRE));
                break;
            case SHOOTPRE:
                if (!gainSet && opmodeTimer.getElapsedTimeSeconds() < 3.0) { break; }
                shoot(PathState.INTAKE11);
                break;
            case INTAKE11:
                move(Intake11, () -> setPathState(PathState.INTAKE12));
                break;
            case INTAKE12:
                moveIntake(Intake12, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE1));
                break;
            case OUTTAKE1:
                moveIdle(Outtake1, () -> setPathState(PathState.SHOOT1));
                break;
            case SHOOT1:
                shoot(PathState.OPENGATE);
                break;
            case OPENGATE:
                move(Opengate, () -> setPathState(PathState.BIGBACK));
                break;
            case BIGBACK:
                moveIntake(BigBack, 0.65, false, 50, () -> setPathState(PathState.OUTTAKEB));
                break;
            case OUTTAKEB:
                moveIdle(OuttakeB, () -> setPathState(PathState.SHOOTB));
                break;
            case SHOOTB:
                shoot(PathState.INTAKE2);
                break;
            case INTAKE2:
                moveIntake(Intake2, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE2));
                break;
            case OUTTAKE2:
                moveIdle(Outtake2, () -> setPathState(PathState.SHOOT2));
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31);
                break;
            case INTAKE31:
                move(Intake31, () -> setPathState(PathState.INTAKE32));
                break;
            case INTAKE32:
                moveIntake(Intake32, 0.7, false, 50, () -> setPathState(PathState.OUTTAKE3));
                break;
            case OUTTAKE3:
                moveIdle(Outtake3, () -> setPathState(PathState.SHOOT3));
                break;
            case SHOOT3:
                shoot(PathState.END);
                break;
            case END:
                move(End, () -> setPathState(PathState.STOP));
                break;
        }
        if (pathState != PathState.END && pathState != PathState.STOP && opmodeTimer.getElapsedTimeSeconds() < 29.5) {
            aiming(detectedTags);
        } else {
            intake.setPower(0);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            turret.setTargetPosition(0);
        }
    }

    public void shoot(PathState nextPath) {
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
        double FWV = flyWheel1.getVelocity();
        if (FWV >= targetV && !shooting) {
            stopper.setPosition(0.973);
            intake.setPower(1);
            shooting = true;
            stopperOpenTime = System.currentTimeMillis();
        }
        if (shooting) {
            minFWVSinceOpen = Math.min(minFWVSinceOpen, FWV);
        }
        boolean ballPassed = shooting
                && minFWVSinceOpen < targetV * 0.90
                && FWV > targetV * 0.97
                && System.currentTimeMillis() - stopperOpenTime > 300;
        if (ballPassed || actionTimer.getElapsedTime() > 1600) {
            shooting = false;
            minFWVSinceOpen = Double.MAX_VALUE;
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags) {
        range = goalPos.findRange(xPos, yPos);
        if (gainSet) {
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(3.2 / range)); // 3.2 in: camera is left of turret axis — re-measure if remounted
                    elevation = detection.ftcPose.elevation;

                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;   // in degrees
                    bearing = Math.toRadians(bearing);
                    elevation = Math.toRadians(elevation);
                    double alpha = hasEst ? 0.08 : 1.0;
                    goalPos.update(alpha, xPos, yPos, bearing, elevation, camRange);
                    hasEst = true;
                    break;
                }
            }
        }

        double turretTarget = goalPos.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT
        // Red turret homes at 0 ticks = forward center; range is ±180° → limits [-990, 850] ticks
        if (turretTarget > 180 + 30) {
            turretTarget -= 360;
        } else if (turretTarget < -180 - 30) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
}
