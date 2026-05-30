package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Blue Far Gate", group = "Autos")
public class BlueFarGate extends BaseAuto {
    private double targetV = 0;
    private final double fwv = 1885;
    private final double lowLimit = 0;
    private final double highLimit = 1865;
    private final double braking = 0.5;

    private enum PathState {
        PRELOAD, SHOOTPRE,
        ALIGNINTAKE1, INTAKE1, OUTTAKE1, SHOOT1,
        ALIGNINTAKE2, TOGATE, OUTTAKE2, SHOOT2,
        END, STOP
    }

    private PathState pathState;

    private final Pose start       = new Pose(56, 0,  Math.toRadians(90));
    private final Pose outtakePre  = new Pose(56, 10, Math.toRadians(90));
    private final Pose outtake     = new Pose(56, 10, Math.toRadians(90));
    private final Pose preintake1  = new Pose(26, 7,  Math.toRadians(180));
    private final Pose intake1     = new Pose(15, 7,  Math.toRadians(180));
    private final Pose preintake2  = new Pose(26, 7,  Math.toRadians(180));
    private final Pose togate      = new Pose(15, 7,  Math.toRadians(180));
    private final Pose end         = new Pose(42, 7,  Math.toRadians(180));

    private PathChain Preload, AlignIntake, Intake1, Outtake1;
    private PathChain AlignIntake2, Togate, Outtake2, End;

    @Override protected double getPIDFP()        { return 380; }
    @Override protected GoalPos createGoalPos()  { return new GoalPos(0, 144, 15.5); }
    @Override protected Pose getStartPose()       { return start; }
    @Override protected double getFWVConstant()   { return 1025; } // tuned lower — was shooting over when tested

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        initHardware();
    }

    @Override
    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setBrakingStrength(0.5)
                .setGlobalDeceleration(0.9)
                .build();
        AlignIntake = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, preintake1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setGlobalDeceleration(0.9)
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake1, intake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.9)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setGlobalDeceleration(0.9)
                .build();
        AlignIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, preintake2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setGlobalDeceleration(0.9)
                .build();
        Togate = follower.pathBuilder()
                .addPath(new BezierLine(preintake2, togate))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.5)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(togate, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .setGlobalDeceleration(0.9)
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setGlobalDeceleration(0.9)
                .build();
    }

    protected void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.resetTimer();
    }

    @Override
    public void statePathUpdate() {
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();
        if (pathState != PathState.END && pathState != PathState.STOP && opmodeTimer.getElapsedTimeSeconds() < 29.5) {
            aiming(detectedTags);
        } else {
            turret.setTargetPosition(0);
        }
        switch (pathState) {
            case PRELOAD:
                move(Preload, () -> setPathState(PathState.SHOOTPRE));
                break;
            case SHOOTPRE:
                if (!gainSet && opmodeTimer.getElapsedTimeSeconds() < 3.0) { break; }
                shoot(PathState.ALIGNINTAKE1);
                break;
            case ALIGNINTAKE1:
                move(AlignIntake, () -> setPathState(PathState.INTAKE1));
                break;
            case INTAKE1:
                moveIntake(Intake1, 0.35, true, 1500, () -> setPathState(PathState.OUTTAKE1));
                break;
            case OUTTAKE1:
                move(Outtake1, () -> setPathState(PathState.SHOOT1));
                break;
            case SHOOT1:
                fixedshoot(PathState.ALIGNINTAKE2);
                break;
            case ALIGNINTAKE2:
                move(AlignIntake2, () -> setPathState(PathState.TOGATE));
                break;
            case TOGATE:
                moveIntake(Togate, 0.35, true, 1500, () -> setPathState(PathState.OUTTAKE2));
                break;
            case OUTTAKE2:
                move(Outtake2, () -> setPathState(PathState.SHOOT2));
                break;
            case SHOOT2:
                fixedshoot(PathState.END);
                break;
            case END:
                move(End, () -> setPathState(PathState.STOP));
                turret.setTargetPosition(0);
                break;
        }
    }

    public void shoot(PathState nextPath) {
        targetV = toFWV(range);
        range = goalPos.findRange(xPos, yPos);
        flyWheel1.setVelocity(fwv);
        if (range < 40) {
            flapPos = 0;
        } else if (range < 95) {
            flapPos = 0.2;
        } else {
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);
        double FWV = flyWheel1.getVelocity();
        if (FWV >= fwv) {
            stopper.setPosition(0.973);
            intake.setPower(0.67);
        }
        if (actionTimer.getElapsedTime() > 5500) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void fixedshoot(PathState nextPath) {
        range = goalPos.findRange(xPos, yPos);
        flyWheel1.setVelocity(fwv);
        if (range < 40) {
            flapPos = 0;
        } else if (range < 95) {
            flapPos = 0.2;
        } else {
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);
        double FWV = flyWheel1.getVelocity();
        if (FWV >= fwv) {
            stopper.setPosition(0.973);
            intake.setPower(0.67);
        }
        if (actionTimer.getElapsedTime() > 5500) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 20) { // SIDE DEPENDENT
                camRange = detection.ftcPose.range + camOffsetX;

                bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(2.6 / range)); // 2.6 in: camera is left of turret axis — re-measure if remounted
                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;   // in degrees
                bearing = Math.toRadians(bearing);

                double alpha = hasEst ? 0.08 : 1.0;
                goalPos.update(alpha, xPos, yPos, bearing, Math.toRadians(detection.ftcPose.elevation), camRange);
                hasEst = true;
                break;
            }
        }

        double turretTarget = goalPos.findBearing(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT
        if (turretTarget > 200) {
            turretTarget -= 360;
        } else if (turretTarget < -200) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
}
