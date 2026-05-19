package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Disabled
@Autonomous(name = "Red Far", group = "Autos")
public class RedFar extends BaseAuto {
    private final double fwv = 1878;
    private final double lowLimit = 0;
    private final double highLimit = 1865;

    private enum PathState {
        PRELOAD, SHOOTPRE,
        ALIGNINTAKE1, INTAKE1, OUTTAKE1, SHOOT1,
        INTAKE21, INTAKE22, OUTTAKE2, SHOOT2,
        END, STOP
    }

    private PathState pathState;

    private final Pose start      = new Pose(90, 0,  Math.toRadians(0));
    private final Pose outtakePre = new Pose(90, 10, Math.toRadians(0));
    private final Pose outtake    = new Pose(90, 10, Math.toRadians(0));
    private final Pose preintake1 = new Pose(120, 5, Math.toRadians(0));
    private final Pose intake1    = new Pose(136, 0, Math.toRadians(0));
//    private final Pose preintake2 = new Pose(120, 5,  Math.toRadians(0));
//    private final Pose intake2    = new Pose(136, 0,  Math.toRadians(0));
//    private final Pose intake2p1  = new Pose(90,  52, Math.toRadians(0));
//    private final Pose intake2p2  = new Pose(120, 57, Math.toRadians(0));
    private final Pose end        = new Pose(108, 6, Math.toRadians(0));

    private PathChain Preload, AlignIntake, Intake1, Outtake1, End;
//    private PathChain Intake21, Intake22, Outtake2;

    @Override protected double getPIDFP()        { return 380; }
    @Override protected GoalPos createGoalPos()  { return new GoalPos(147, 144, 15.5); }
    @Override protected Pose getStartPose()       { return start; }
    @Override protected double getFWVConstant()   { return 1162; }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        baseInit();
    }

    @Override
    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .setGlobalDeceleration(0.9)
                .build();
        AlignIntake = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, preintake1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake1, intake1))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(0)
                .setGlobalDeceleration(0.9)
                .build();
//        Intake21 = follower.pathBuilder()
//                .addPath(new BezierLine(outtake, intake2p1))
//                .setConstantHeadingInterpolation(0)
//                .build();
//        Intake22 = follower.pathBuilder()
//                .addPath(new BezierLine(intake2p1, intake2p2))
//                .setConstantHeadingInterpolation(0)
//                .build();
//        Outtake2 = follower.pathBuilder()
//                .addPath(new BezierLine(intake2p2, outtake))
//                .setConstantHeadingInterpolation(0)
//                .build();
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
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();
        if (pathState != PathState.END && pathState != PathState.STOP) {
            aiming(detectedTags);
        } else {
            turret.setTargetPosition(0);
        }
        switch (pathState) {
            case PRELOAD:
                move(Preload, () -> setPathState(PathState.SHOOTPRE));
                break;
            case SHOOTPRE:
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
                shoot(PathState.END);
                break;
//            case INTAKE21:
//                move(Intake21, () -> setPathState(PathState.INTAKE22));
//                break;
//            case INTAKE22:
//                moveIntake(Intake22, 0.35, true, 1500, () -> setPathState(PathState.OUTTAKE2));
//                break;
//            case OUTTAKE2:
//                move(Outtake2, () -> setPathState(PathState.SHOOT2));
//                break;
//            case SHOOT2:
//                shoot(PathState.END);
//                break;
            case END:
                move(End, () -> setPathState(PathState.STOP));
                break;
        }
    }

    public void shoot(PathState nextPath) {
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
