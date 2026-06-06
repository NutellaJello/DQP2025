package org.firstinspires.ftc.teamcode.Autos; // make sure this aligns with class location
import android.util.Size;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red 15", group = "Autos")
public class RedClose15 extends OpMode {
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean gainSet = false;
    private Follower follower;
    private Timer actionTimer, opmodeTimer;
    private boolean moving = false;
    GoalPos goal = new GoalPos(147,143, 15.5);
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = -2167;
    private final double highLimit = 340;
    private double camRange;
    private double bearing;
    private double elevation;
    private double xEst;
    private double yEst;
    private final double camOffsetX = 2;
    private double turretPos;
    private double flapPos = 0.2;
    private boolean hasEst = false;
    double p = 400;
    double d = 0;
    double i = 0;
    double f = 13.5;
    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d,  f);
    private enum PathState {
        PRELOAD,
        SHOOTPRE,
        INTAKE11,
        INTAKE12,
        OUTTAKE1,
        SHOOT1,
        OPENGATE,
        BIGBACK,
        OUTTAKEB,
        SHOOTB,
        INTAKE2,
        OUTTAKE2,
        SHOOT2,
        INTAKE31,
        INTAKE32,
        OUTTAKE3,
        SHOOT3,
        END,
        STOP
    }

    private PathState pathState;
    //positions
    private final Pose start = new Pose(120, 133, Math.toRadians(0));
    private final Pose outtakePre = new Pose(93, 90, Math.toRadians(0));
    private final Pose outtake = new Pose(100, 90, Math.toRadians(0));
    private final Pose intake1p1 = new Pose(105, 67, Math.toRadians(0));
    private final Pose intake1p2 = new Pose(130, 67 - 2, Math.toRadians(0));
    private final Pose outtake1Point = new Pose(106, 65, Math.toRadians(0));
    private final Pose gatePoint = new Pose(122,46, Math.toRadians(46));
    private final Pose gate = new Pose (133, 65.5, Math.toRadians(20));
    private final Pose bigBack = new Pose(142, 53, Math.toRadians(46));
    private final Pose bigBackPoint = new Pose(133, 50, Math.toRadians(46));
    private final Pose outtakeBPoint = new Pose(100, 60, Math.toRadians(0));
    private final Pose intake2 = new Pose(127, 88, Math.toRadians(0));
    private final Pose intake3p1 = new Pose(100, 45, Math.toRadians(0));
    private final Pose intake3p2 = new Pose(130, 45 - 6, Math.toRadians(0));
    private final Pose end = new Pose(108, 77, Math.toRadians(0));

    //Paths
    private PathChain Preload;
    private PathChain Intake11;
    private PathChain Intake12;
    private PathChain Outtake1;
    private PathChain Opengate;
    private PathChain BigBack;
    private PathChain OuttakeB;
    private PathChain Intake2;
    private PathChain Outtake2;
    private PathChain Intake31;
    private PathChain Intake32;
    private PathChain Outtake3;
    private PathChain End;

    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        Intake11 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1p1))
                .setConstantHeadingInterpolation(0)
                .build();
        Intake12 = follower.pathBuilder()
                .addPath(new BezierLine(intake1p1, intake1p2))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(intake1p2, outtake1Point, outtake)))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        Opengate = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(outtake, gatePoint, gate)))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate.getHeading())
                .setBrakingStrength(0.08)
                .build();
        BigBack = follower.pathBuilder()
                .addPath(new BezierLine(gate, bigBack))
                .setLinearHeadingInterpolation(gate.getHeading(), bigBack.getHeading())
                .build();
        OuttakeB = follower.pathBuilder()
                .addPath(new BezierCurve(bigBack, outtakeBPoint, outtake))
                .setLinearHeadingInterpolation(gate.getHeading(), outtake.getHeading())
                .build();
        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(0)
                .build();
        Intake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(0)
                .build();
        Intake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
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
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);

        flap = hardwareMap.get(Servo.class, "flap");
        flap.setDirection(Servo.Direction.FORWARD);


        buildPaths();
        initWebcam();
        follower.setPose(start);
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
        actionTimer.resetTimer();
    }
    @Override
    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
            visionPortal = null;
        }
    }

    public void statePathUpdate() {
//        telemetry.addData("step", pathState);
//        telemetry.addData("busy", follower.isBusy());
//        telemetry.addData("heading", follower.getHeading());
//        telemetry.addData("velocity", follower.getVelocity());
//        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();

        switch (pathState) {
            case PRELOAD:
                move(Preload, PathState.SHOOTPRE, true);
                break;
            case SHOOTPRE:
                shoot(PathState.INTAKE11);
                break;
            case INTAKE11:
                move(Intake11, PathState.INTAKE12);
                break;
            case INTAKE12:
                moveIntake(Intake12, PathState.OUTTAKE1);
                break;
            case OUTTAKE1:
                move(Outtake1, PathState.SHOOT1, true);
                break;
            case SHOOT1:
                shoot(PathState.OPENGATE);
                break;
            case OPENGATE:
                move(Opengate, PathState.BIGBACK);
                break;
            case BIGBACK:
                moveIntake(BigBack, PathState.OUTTAKEB, 0.65);
                break;
            case OUTTAKEB:
                move(OuttakeB, PathState.SHOOTB, true);
                break;
            case SHOOTB:
                shoot(PathState.INTAKE2);
                break;
            case INTAKE2:
                moveIntake(Intake2, PathState.OUTTAKE2);
                break;
            case OUTTAKE2:
                move(Outtake2, PathState.SHOOT2);
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31);
                break;
            case INTAKE31:
                move(Intake31, PathState.INTAKE32);
                break;
            case INTAKE32:
                moveIntake(Intake32, PathState.OUTTAKE3);
                break;
            case OUTTAKE3:
                move(Outtake3, PathState.SHOOT3);
                break;
            case SHOOT3:
                shoot(PathState.END);
                break;
            case END:
                move(End, PathState.STOP);
                break;
        }
        if(pathState != PathState.END && pathState != PathState.STOP && opmodeTimer.getElapsedTimeSeconds() < 29.5){
            aiming(detectedTags);
        }
        else{
            intake.setPower(0);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
        }
    }

    @Override
    public void loop() {
        follower.update();
        xPos = follower.getPose().getX();
        yPos = follower.getPose().getY();
        heading = follower.getPose().getHeading();
        turretPos = turret.getCurrentPosition();
        statePathUpdate();
        if (opmodeTimer.getElapsedTime() > 500 && !gainSet) {
            cameraControls();
        }
    }

    public void move(PathChain path, PathState nextPath){
        move(path, nextPath, false);
    }
    public void move(PathChain path, PathState nextPath, boolean idle){
        if (!moving) {
            if(idle){
                flyWheel1.setVelocity(1100);
                flyWheel2.setVelocity(1100);
            }
            follower.followPath(path, false);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }
    public void moveIntake(PathChain path, PathState nextPath){
        moveIntake(path, nextPath, 0.7);
    }
    public void moveIntake(PathChain path, PathState nextPath, double power){
        if (!moving) {
            follower.followPath(path,power, false);
            intake.setPower(1);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            intake.setPower(0);
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }

    public void shoot(PathState nextPath){
        double targetV = toFWV(range);
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);
        if (range < 40) {
            flapPos = 0;
        }else if (range < 95){
            flapPos = 0.2;
        }else{
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);
        double FWV = flyWheel1.getVelocity();
        if(FWV >= targetV){
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

    public void aiming(List<AprilTagDetection> detectedTags){
        range = goal.findRange(xPos, yPos);
        if(gainSet){
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(3.2/range)); // SIDE +/-
                    elevation = detection.ftcPose.elevation;

                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
                    bearing = Math.toRadians(bearing);
                    elevation = Math.toRadians(elevation);
                    goal.update(0.08, xPos, yPos, bearing, elevation, camRange);
                    break;
                }
            }
        }

        //required turret angle
        double turretTarget = goal.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT
        if (turretTarget > highLimit * (90.0/495.0) + 30.0) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < lowLimit * (90.0/495.0) - 30.0) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }
    private void initWebcam() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.setCameraResolution(new Size(640, 480)); //640 480

        builder.enableLiveView(false);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }
    public void cameraControls(){
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // exposure and gain
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            gainControl.setGain(100);
            gainSet = true;
        }
    }

    public double toFWV(double r){
        return (0.00673 * r * r) + (5.54 * r) +  (1150);
    }
}
