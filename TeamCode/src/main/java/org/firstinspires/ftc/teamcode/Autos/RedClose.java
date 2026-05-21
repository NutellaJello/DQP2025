package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Disabled
@Autonomous(name = "Red Close", group = "Autos")
public class RedClose extends BaseAuto {
    private DcMotorEx flyWheel2;
    private boolean shooting = false;
    private double minFWVSinceOpen = Double.MAX_VALUE;
    private long stopperOpenTime = 0;
    private final double lowLimit = 0;
    private final double highLimit = 1865;
    private final double braking = 0.9;

    private enum PathState {
        PRELOAD, SHOOTPRE,
        INTAKE1, OUTTAKE1, SHOOT1,
        INTAKE21, INTAKE22, OUTTAKE2, SHOOT2,
        INTAKE31, INTAKE32, OUTTAKE3, SHOOT3,
        END, STOP
    }

    private PathState pathState;

    private final Pose start      = new Pose(130, 142, Math.toRadians(-139));
    private final Pose outtakePre = new Pose(93,  90,  Math.toRadians(0));
    private final Pose outtake    = new Pose(100, 90,  Math.toRadians(0));
    private final Pose intake1    = new Pose(125, 88,  Math.toRadians(0));
    private final Pose intake2p1  = new Pose(100, 67,  Math.toRadians(0));
    private final Pose intake2p2  = new Pose(128, 67 - 2, Math.toRadians(0));
    private final Pose intake3p1  = new Pose(100, 39,  Math.toRadians(0));
    private final Pose intake3p2  = new Pose(128, 39,  Math.toRadians(0));
    private final Pose end        = new Pose(112, 77,  Math.toRadians(0));

    private PathChain Preload, Intake1, Outtake1;
    private PathChain Intake21, Intake22, Outtake2;
    private PathChain Intake31, Intake32, Outtake3;
    private PathChain End;

    @Override protected double getPIDFP()        { return 380; } // tuned lower than RedClose15/RedCloseGate (400) — verify intentional
    @Override protected GoalPos createGoalPos()  { return new GoalPos(147, 144, 15.5); }
    @Override protected Pose getStartPose()       { return start; }
    @Override protected double getFWVConstant()   { return 1162; }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        baseInit();
        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);
        shooting = false;
        minFWVSinceOpen = Double.MAX_VALUE;
        stopperOpenTime = 0;
    }

    @Override
    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .setBrakingStrength(braking)
                .setGlobalDeceleration(0.9)
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
                .setGlobalDeceleration(0.9)
                .build();
        Intake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Intake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1, intake2p2))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
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
                .setBrakingStrength(braking)
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

    @Override
    public void statePathUpdate() {
        telemetry.update();
        if (pathState != PathState.END && pathState != PathState.STOP && opmodeTimer.getElapsedTimeSeconds() < 29.3) {
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();
            aiming(detectedTags);
        } else {
            intake.setPower(0);
            turret.setTargetPosition(0);
        }
        switch (pathState) {
            case PRELOAD:
                move(Preload, () -> setPathState(PathState.SHOOTPRE));
                break;
            case SHOOTPRE:
                if (!gainSet && opmodeTimer.getElapsedTimeSeconds() < 3.0) { break; }
                shoot(PathState.INTAKE1);
                break;
            case INTAKE1:
                moveIntake(Intake1, 0.35, true, 50, () -> setPathState(PathState.OUTTAKE1));
                break;
            case OUTTAKE1:
                move(Outtake1, () -> setPathState(PathState.SHOOT1));
                break;
            case SHOOT1:
                shoot(PathState.INTAKE21);
                break;
            case INTAKE21:
                move(Intake21, () -> setPathState(PathState.INTAKE22));
                break;
            case INTAKE22:
                moveIntake(Intake22, 0.35, true, 50, () -> setPathState(PathState.OUTTAKE2));
                break;
            case OUTTAKE2:
                move(Outtake2, () -> setPathState(PathState.SHOOT2));
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31);
                break;
            case INTAKE31:
                move(Intake31, () -> setPathState(PathState.INTAKE32));
                break;
            case INTAKE32:
                moveIntake(Intake32, 0.35, true, 50, () -> setPathState(PathState.OUTTAKE3));
                break;
            case OUTTAKE3:
                move(Outtake3, () -> setPathState(PathState.SHOOT3));
                break;
            case SHOOT3:
                shoot(PathState.END);
                break;
            case END:
                move(End, () -> {
                    if (visionPortal != null) {
                        visionPortal.close();
                        visionPortal = null;
                    }
                    setPathState(PathState.STOP);
                });
                break;
        }
    }

    public void shoot(PathState nextPath) {
        // range also updated in aiming(); kept here as fallback when aiming() is skipped (t > 29.3 s)
        range = goalPos.findRange(xPos, yPos);
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
        double FWV = Math.min(flyWheel1.getVelocity(), flyWheel2.getVelocity());
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
        if (ballPassed || actionTimer.getElapsedTime() > 2800) {
            shooting = false;
            minFWVSinceOpen = Double.MAX_VALUE;
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            setPathState(nextPath);
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags) {
        range = goalPos.findRange(xPos, yPos);
        if (gainSet) {
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
                    camRange = detection.ftcPose.range + camOffsetX;

                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(2.5 / range)); // 2.5 in: camera is left of turret axis — re-measure if remounted
                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;   // in degrees
                    bearing = Math.toRadians(bearing);

                    double alpha = hasEst ? 0.08 : 1.0;
                    goalPos.update(alpha, xPos, yPos, bearing, Math.toRadians(detection.ftcPose.elevation), camRange);
                    hasEst = true;
                    break;
                }
            }
        }

        double turretTarget = goalPos.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT
        if (turretTarget > 360 + 30) {
            turretTarget -= 360;
        } else if (turretTarget < 0 - 30) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
}
