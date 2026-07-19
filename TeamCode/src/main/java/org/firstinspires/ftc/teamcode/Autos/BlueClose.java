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

@Autonomous(name = "Blue Close", group = "Autos") // SIDE Red/Blue
public class BlueClose extends OpMode { // SIDE Red/Blue
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
    GoalPos goal = new GoalPos(-147,143, 15.5); // SIDE +147/-147
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = -1906;
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
        PRELOAD,
        SHOOTPRE,
        INTAKE11,
        INTAKE12,
        OUTTAKE1,
        SHOOT1,
        INTAKEG1,
        OUTTAKEG1,
        SHOOTG1,
        INTAKE2,
        OUTTAKE2,
        SHOOT2,
        INTAKEG2,
        OUTTAKEG2,
        SHOOTG2,
        END,
        STOP
    }

    private PathState pathState;
    //positions SIDE +/- ALL X COORDINATES none/180- ALL ANGLES
    private final Pose start = new Pose(-122, 133, Math.toRadians(180));
    private final Pose outtakePre = new Pose(-93, 90, Math.toRadians(180));
    private final Pose outtake = new Pose(-100, 90, Math.toRadians(180));
    private final Pose intake1p1 = new Pose(-105, 67, Math.toRadians(180));
    private final Pose intake1p2 = new Pose(-130, 67 - 2, Math.toRadians(180));
    private final Pose outtake1Point = new Pose(-106, 65, Math.toRadians(180));
    private final Pose gatePoint = new Pose(-122,63);
    private final Pose gate1 = new Pose (-138, 63, Math.toRadians(180-30));
    private final Pose gate2 = new Pose (-138, 63, Math.toRadians(180-30));
    private final Pose intake2 = new Pose(-127.5, 88, Math.toRadians(180));
    private final Pose end = new Pose(-108, 77, Math.toRadians(180));

    //Paths
    private PathChain Preload;
    private PathChain Intake11;
    private PathChain Intake12;
    private PathChain Outtake1;
    private PathChain IntakeG1;
    private PathChain IntakeG2;
    private PathChain OuttakeG1;
    private PathChain OuttakeG2;
    private PathChain Intake2;
    private PathChain Outtake2;
    private PathChain End;

    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        Intake11 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1p1))
                .setConstantHeadingInterpolation(outtakePre.getHeading())
                .build();
        Intake12 = follower.pathBuilder()
                .addPath(new BezierLine(intake1p1, intake1p2))
                .setConstantHeadingInterpolation(intake1p1.getHeading())
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(intake1p2, outtake1Point, outtake)))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        IntakeG1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(outtake, gatePoint, gate1)))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate1.getHeading())
                .setBrakingStrength(0.3)
                .build();
        IntakeG2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, gate2))
                .setLinearHeadingInterpolation(outtake.getHeading(), gate2.getHeading())
                .setBrakingStrength(0.3)
                .build();
        OuttakeG1 = follower.pathBuilder()
                .addPath(new BezierCurve(Arrays.asList(gate1, gatePoint, outtake)))
                .setLinearHeadingInterpolation(gate1.getHeading(), outtake.getHeading())
                .build();
        OuttakeG2 = follower.pathBuilder()
                .addPath(new BezierLine(gate2, outtake))
                .setLinearHeadingInterpolation(gate2.getHeading(), outtake.getHeading())
                .build();
        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2))
                .setConstantHeadingInterpolation(outtake.getHeading())
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, outtake))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(outtake.getHeading())
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
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        turret.setPositionPIDFCoefficients(15);

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
                shoot(PathState.INTAKEG1);
                break;
            case INTAKEG1:
                moveIntake(IntakeG1, PathState.OUTTAKEG1, 1, 4500);
                break;
            case OUTTAKEG1:
                move(OuttakeG1, PathState.SHOOTG1, true);
                break;
            case SHOOTG1:
                shoot(PathState.INTAKE2);
                break;
            case INTAKE2:
                moveIntake(Intake2, PathState.OUTTAKE2);
                break;
            case OUTTAKE2:
                move(Outtake2, PathState.SHOOT2, true);
                break;
            case SHOOT2:
                shoot(PathState.INTAKEG2);
                break;
            case INTAKEG2:
                moveIntake(IntakeG2, PathState.OUTTAKEG2, 1, 4000);
                break;
            case OUTTAKEG2:
                move(OuttakeG2, PathState.SHOOTG2, true);
                break;
            case SHOOTG2:
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
                flyWheel1.setVelocity(1300);
                flyWheel2.setVelocity(1300);
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
            intake.setPower(0);
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
            follower.followPath(path, power, true);
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
            intake.setPower(0);
            pathState = nextPath;
            actionTimer.resetTimer();
            moving = false;
        }
    }

    public void shoot(PathState nextPath){
        double targetV = 0.903*range * 7.710 + 970;  //FWTarget = range * 7.710 + 980
        flyWheel1.setVelocity(targetV);
        flyWheel2.setVelocity(targetV);

        if(range > 53){
            flapPos = range * 0.00080 + 0.1481; //flapPos = range * 0.00086 + 0.1481;
        } else{
            flapPos = range * 0.011 - 0.385;
        }
        flapPos = Range.clip(flapPos, 0, 0.22);
        flap.setPosition(flapPos);

        double FWV = Math.max(flyWheel1.getVelocity(), flyWheel2.getVelocity());
        if(FWV >= targetV){
            stopper.setPosition(0.973);
            intake.setPower(1);
        }
        if (actionTimer.getElapsedTime() > 900) {
            shotCounter++;
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            if(shotCounter == 5 /*&& shotCounter <= 6*/){
                pathState = PathState.INTAKEG2;
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
                    goal.update(0.15, xPos, yPos, bearing, elevation, camRange);
                    break;
                }
            }
        }

        //required turret angle
        hOffset = range * 0.0309 - 4.0; //hOffset = range * 0.0309 - 5.367 // SIDE 4.0/3.0
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