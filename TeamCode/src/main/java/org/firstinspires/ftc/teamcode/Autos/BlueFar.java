package org.firstinspires.ftc.teamcode.Autos; // make sure this aligns with class location
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Far", group = "Autos") // SIDE Red/Blue
public class BlueFar extends OpMode { // SIDE Red/Blue
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean gainSet = false;
    private boolean streamStarted = false;
    private double camStreamingTime;
    private Follower follower;
    private Timer actionTimer, opModeTimer;
    private boolean moving = false;
    GoalPos goal = new GoalPos(14,135, 15.5); // SIDE +147/-147
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = -1506;
    private final double highLimit = 340;
    private double camRange;
    private double bearing;
    private double elevation;
    private double xEst;
    private double yEst;
    private final double camOffsetX = 2;
    private double turretPos;
    private double hOffset;
    private double flapPos = 0.2;
    private boolean hasEst = false;
    double p = 400;
    double d = 0;
    double i = 0;
    double f = 13.5;
    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d,  f);
    private int shotCounter = 0;
    private enum PathState {
        PRELOAD, SHOOTPRE,
        ALIGNINTAKE1, INTAKE1, OUTTAKE1, SHOOT1,
        INTAKE21, INTAKE22, OUTTAKE2, SHOOT2, INTAKEG, OUTTAKEG, SHOOTG,
        END, STOP
    }

    private PathState pathState;
    //positions SIDE +/- ALL X COORDINATES none/180- ALL ANGLES
    private final Pose start = new Pose(56, 0, Math.toRadians(90)); // staring postition
    private final Pose outtakePre = new Pose(56, 8, Math.toRadians(90)); // moving out to shoot preload
    private final Pose outtake = new Pose(56, 12, Math.toRadians(135));  // general position to shoot after getting preload

    private final Pose preintake1 = new Pose(26, 7.3, Math.toRadians(180));// need to align because doesnt curve
    // test bezier curve later.

    private final Pose intake1 = new Pose(15, 7.3, Math.toRadians(180)); // intaking the batch @ loading
    private final Pose intake2p1 = new Pose(56, 35, Math.toRadians(180)); // moving to get the second batch
    private final Pose intake2p2 = new Pose(26, 35, Math.toRadians(180)); // actually moving inward to get batch
    private final Pose gateCycle = new Pose(15, 8, Math.toRadians(180));
    private final Pose end = new Pose(44, 10, Math.toRadians(180));

    //Paths
    private PathChain Preload, AlignIntake, Intake1, Outtake1, Intake21, Intake22, Outtake2, IntakeG, OuttakeG, End;

    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();
        AlignIntake = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, preintake1)) // test bezier curve later
                .setLinearHeadingInterpolation(outtakePre.getHeading(), preintake1.getHeading())
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake1, intake1))
                .setConstantHeadingInterpolation(preintake1.getHeading())
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(intake1.getHeading(), outtake.getHeading())
                .build();
        Intake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setLinearHeadingInterpolation(outtake.getHeading(), intake2p1.getHeading())
                .build();
        Intake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1, intake2p2))
                .setConstantHeadingInterpolation(intake2p1.getHeading())
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setLinearHeadingInterpolation(intake2p2.getHeading(),outtake.getHeading())
                .build();
        IntakeG = follower.pathBuilder()
                .addPath(new BezierLine(outtake, gateCycle))
                .setLinearHeadingInterpolation(outtake.getHeading(),gateCycle.getHeading())
                .build();
        OuttakeG = follower.pathBuilder()
                .addPath(new BezierLine(gateCycle, outtake))
                .setLinearHeadingInterpolation(gateCycle.getHeading(),outtake.getHeading())
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setLinearHeadingInterpolation(outtake.getHeading(), end.getHeading())
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
        opModeTimer = new Timer();
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
        turret.setPositionPIDFCoefficients(20);

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);

        flap = hardwareMap.get(Servo.class, "flap");
        flap.setDirection(Servo.Direction.FORWARD);


        buildPaths();
        initWebcam();
        follower.setPose(start);
    }

    public void start() {
        opModeTimer.resetTimer();
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
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();

        switch (pathState) {
            case PRELOAD:
                move(Preload, PathState.SHOOTPRE, true);
                break;
            case SHOOTPRE:
                shoot(PathState.ALIGNINTAKE1);
                break;
            case ALIGNINTAKE1:
                move(AlignIntake, PathState.INTAKE1);
                break;
            case INTAKE1:
                moveIntake(Intake1, PathState.OUTTAKE1, 0.4, 2000);
                break;
            case OUTTAKE1:
                move(Outtake1, PathState.SHOOT1, true);
                break;
            case SHOOT1:
                shoot(PathState.INTAKE21);
                break;
            case INTAKE21:
                move(Intake21, PathState.INTAKE22);
                break;
            case INTAKE22:
                moveIntake(Intake22, PathState.OUTTAKE2);
                break;
            case OUTTAKE2:
                move(Outtake2, PathState.SHOOT2, true);
                break;
            case SHOOT2:
                shoot(PathState.INTAKEG);
                break;
            case INTAKEG:
                moveIntake(IntakeG, PathState.OUTTAKEG, 1);
                break;
            case OUTTAKEG:
                move(OuttakeG, PathState.SHOOTG, true);
                break;
            case SHOOTG:
                shoot(PathState.END);
                break;
            case END:
                move(End, PathState.STOP);
                break;
        }
        if(pathState != PathState.END && pathState != PathState.STOP && opModeTimer.getElapsedTimeSeconds() < 29.5){
            aiming(detectedTags);
        }
        else{
            turret.setTargetPosition(0);
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
        // configure webcam
        if(!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            camStreamingTime = opModeTimer.getElapsedTime();
            streamStarted = true;
        }
        if(!gainSet && streamStarted && opModeTimer.getElapsedTime() > camStreamingTime + 500){
            gainSet = cameraControls();
        }
    }

    public void move(PathChain path, PathState nextPath){
        move(path, nextPath, false, 50);
    }
    public void move(PathChain path, PathState nextPath, boolean idle){
        move(path, nextPath, idle, 50);
    }
    public void move(PathChain path, PathState nextPath, boolean idle, double wait){
        if (!moving) {
            if(idle){
                flyWheel1.setVelocity(1100);
                flyWheel2.setVelocity(1100);
            }
            follower.followPath(path, false);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > wait) {
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
        if(actionTimer.getElapsedTime() > 3000){
            follower.breakFollowing();
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }
    public void moveIntake(PathChain path, PathState nextPath){
        moveIntake(path, nextPath, 0.7, 50);
    }
    public void moveIntake(PathChain path, PathState nextPath, double power){
        moveIntake(path, nextPath, power, 50);
    }
    public void moveIntake(PathChain path, PathState nextPath, double power, double wait){
        if (!moving) {
            follower.followPath(path, power, false);
            intake.setPower(1);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > wait) {
            intake.setPower(0);
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
        if(actionTimer.getElapsedTime() > 3000){
            follower.breakFollowing();
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }

    public void shoot(PathState nextPath){
        double targetV = 0.903*range * 7.462 + 1000;  //FWTarget = range * 7.710 + 980
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);

        // no regression needed
        flap.setPosition(0.22);

        double FWV = Math.max(flyWheel1.getVelocity(), flyWheel2.getVelocity());
        if(FWV >= targetV){
            stopper.setPosition(0.973);
            intake.setPower(1);
        }
        if (actionTimer.getElapsedTime() > 2000) {
            shotCounter++;
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            if(shotCounter == 4){ //shotCounter >= 4 && shotCounter <= 5
                pathState = PathState.INTAKEG;
            }else{
                pathState = nextPath;
            }
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags){
        range = goal.findRange(xPos, yPos);
        if(gainSet){
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 20) { // SIDE 24/20
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing;
                    elevation = detection.ftcPose.elevation;

                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180.0/976.0;   // in degrees
                    bearing = Math.toRadians(bearing);
                    elevation = Math.toRadians(elevation);
                    goal.update(0.08, xPos, yPos, bearing, elevation, camRange);
                    break;
                }
            }
        }

        //required turret angle
        hOffset = range * 0.0309 - 3.0; //hOffset = range * 0.0309 - 5.367 // SIDE 4.0/3.0
        double turretTarget = goal.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading)
                - Math.toDegrees(Math.atan(hOffset/range)); // SIDE +/-
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

    public boolean cameraControls(){
        boolean exposureOK = false;
        boolean gainOK = true;
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if(exposureControl == null || gainControl == null){
            return false;
        }

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureOK = exposureControl.setExposure(2, TimeUnit.MILLISECONDS);

        gainOK = gainControl.setGain(100);

        return exposureOK && gainOK;
    }
}