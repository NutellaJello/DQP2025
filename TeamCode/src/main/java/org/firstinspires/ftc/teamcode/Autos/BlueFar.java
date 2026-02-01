package org.firstinspires.ftc.teamcode.Autos; // make sure this aligns with class location
import android.graphics.Paint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Blue Far", group = "Autos")
public class BlueFar extends OpMode {
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;
    private Timer actionTimer, opmodeTimer;
    private boolean moving = false;
    private boolean atFWV = false;
    GoalPos goalPos = new GoalPos(0,144);
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 180; // angle from straight forward (counterclockwise)
    private final double lowLimit = -450;
    private final double highLimit = 730;
    private enum PathState {
        PRELOAD,
        SHOOTPRE,
        ALIGNINTAKE1,
        INTAKE1,
        OUTTAKE1,
        SHOOT1,
        INTAKE21,
        INTAKE22,
        OUTTAKE2,
        SHOOT2,
        END,
        STOP
    }

    private PathState pathState;
    //positions
    private final Pose start = new Pose(52, 0, Math.toRadians(90)); // staring postition
    private final Pose outtakePre = new Pose(52, 10, Math.toRadians(90)); // moving out to shoot preload
    private final Pose outtake = new Pose(52, 10, Math.toRadians(0));  // general position to shoot after getting preload

    private final Pose preintake1 = new Pose(52, 30, Math.toRadians(0));// need to align because doesnt curve
    // test bezier curve later.

    private final Pose intake1 = new Pose(22, 35, Math.toRadians(0)); // intaking the first batch

    private final Pose intake2p1 = new Pose(52, 52, Math.toRadians(0)); // moving to get the second batch
    private final Pose intake2p2 = new Pose(22, 57, Math.toRadians(0)); // actually moving inward to get batch
    private final Pose end = new Pose(52, 10, Math.toRadians(0));

    //paths
    private PathChain Preload;
    private PathChain AlignIntake;
    private PathChain Intake1;
    private PathChain Outtake1;
    private PathChain Intake21;
    private PathChain Intake22;
    private PathChain Outtake2;
    private PathChain End;

    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        AlignIntake = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, preintake1)) // test bezier curve later
                .setConstantHeadingInterpolation(0)
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake1, intake1))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        Intake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setConstantHeadingInterpolation(0)
                .build();
        Intake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1, intake2p2))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(0)
                .build();
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        actionTimer.resetTimer();

    }

    @Override
    public void init() {
        pathState = PathState.PRELOAD;
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createAutoFollower(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

//        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
//        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
//        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
//        flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
//        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);


        buildPaths();
        follower.setPose(start);
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
        actionTimer.resetTimer();
    }

    public void statePathUpdate() {
        telemetry.addData("step", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();

        if(pathState != PathState.END && pathState != PathState.STOP){
            aiming();
        }
        switch (pathState) {
            case PRELOAD:
                move(Preload, PathState.SHOOTPRE);
                break;
            case SHOOTPRE:
                shoot(1000,1000, PathState.ALIGNINTAKE1);
                break;
            case ALIGNINTAKE1:
                move(AlignIntake, PathState.INTAKE1);
                break;
            case INTAKE1:
                moveIntake(Intake1, PathState.OUTTAKE1);
                break;
            case OUTTAKE1:
                move(Outtake1, PathState.SHOOT1);
                break;
            case SHOOT1:
                shoot(1000,1000,PathState.INTAKE21);
                break;
            case INTAKE21:
                move(Intake21, PathState.INTAKE22);
                break;
            case INTAKE22:
                moveIntake(Intake22, PathState.OUTTAKE2);
                break;
            case OUTTAKE2:
                move(Outtake2, PathState.SHOOT2);
                break;
            case SHOOT2:
                shoot(1000,1000,PathState.END);
                break;
            case END:
                move(End, PathState.STOP);
                turret.setTargetPosition(0);
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        xPos = follower.getPose().getX();
        yPos = follower.getPose().getY();
        heading = follower.getPose().getHeading();
        statePathUpdate();
    }

    public void move(PathChain path, PathState nextPath){
        if (!moving) {
            follower.followPath(path, true);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }

    public void moveIntake(PathChain path, PathState nextPath){
        if (!moving) {
            follower.followPath(path,0.3, true);
            intake.setPower(0.8);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            intake.setPower(0);
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }

    public void shoot(double FW1Target, double FW2Target, PathState nextPath){
        if (!follower.isBusy() && !atFWV) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            //  flyWheel1.setVelocity(FW1Target);
            // flyWheel2.setVelocity(FW2Target);
        }
        double FWV1 = 1200;//flyWheel1.getVelocity();
        double FWV2 = 1200;//flyWheel2.getVelocity();
        if(FWV1 >= FW1Target && FWV2 >= FW2Target){
            stopper.setPosition(0.965);
            intake.setPower(0.8);
            atFWV = true;
        }
        if (actionTimer.getElapsedTime() > 4000) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            //   flyWheel1.setVelocity(0);
            // flyWheel2.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
            atFWV = false;
        }
    }

    public void aiming(){//){}List<AprilTagDetection> detectedTags){
//        for (AprilTagDetection detection : detectedTags) {
//            if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
//                camRange = detection.ftcPose.range;
//                bearing = Math.toRadians(detection.ftcPose.bearing);
//                double xCam = camRange * Math.cos(bearing); //cartesian coordinates in cam frame of reference
//                double yCam = camRange * Math.sin(bearing) - camOffset;
//                range = Math.hypot(xCam, yCam); // corrected range
//                bearing = Math.toDegrees(Math.atan2(yCam, xCam)); // corrected bearing
//
//                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
//                bearing = Math.toRadians(bearing);
//                if(!gamepad1.x){
//                    goalPos.update(xPos, yPos, bearing, camRange);
//                }
//                xEst = xPos + camRange * Math.cos(bearing);
//                yEst = yPos + camRange * Math.sin(bearing);
//                if(!hasEst){
//                    goalPos.setX(xEst);
//                    goalPos.setY(yEst);
//                }
//                hasEst = true;
//                break;
//            }
//        }

        //required turret angle
        double turretTarget = goalPos.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading);
        if (turretTarget > 200) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < -200) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        //turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));

        turret.setTargetPosition((int) turretTarget);
    }
}
