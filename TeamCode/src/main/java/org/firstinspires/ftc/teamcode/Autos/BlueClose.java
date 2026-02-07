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

@Autonomous(name = "Blue Close", group = "Autos")
public class BlueClose extends OpMode {
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;
    private Timer actionTimer, opmodeTimer;
    private boolean moving = false;
    GoalPos goalPos = new GoalPos(0,144);
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 0;
    private final double lowLimit = 0;
    private final double highLimit = 1865;
    private double camRange;
    private double bearing;
    private double xEst;
    private double yEst;
    private final double camOffsetX = 2;
    private double turretPos;
    private double flapPos = 0.2;
    private boolean hasEst = false;
    double p = 380;
    double d = 0;
    double i = 0;
    double f = 13.5;
    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d,  f);
    private final double outtakeBrake = 0.7;
    private final double intakeBreak = 0.7;
    private enum PathState {
        PRELOAD,
        SHOOTPRE,
        INTAKE1,
        OUTTAKE1,
        SHOOT1,
        INTAKE21,
        INTAKE22,
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
    private final Pose start = new Pose(14, 142, Math.toRadians(139));
    private final Pose outtakePre = new Pose(40, 105, Math.toRadians(135));
    private final Pose outtake = new Pose(40, 105, Math.toRadians(135));
    private final Pose intake1 = new Pose(4, 105, Math.toRadians(180));
    private final Pose intake2p1 = new Pose(40, 77, Math.toRadians(180));
    private final Pose intake2p2 = new Pose(1, 77 - 2, Math.toRadians(180));
    private final Pose intake3p1 = new Pose(40, 58, Math.toRadians(180));
    private final Pose intake3p2 = new Pose(1, 54-2, Math.toRadians(180));
    private final Pose end = new Pose(20, 77, Math.toRadians(180));

    //Paths
    private PathChain Preload;
    private PathChain Intake1;
    private PathChain Outtake1;
    private PathChain Intake21;
    private PathChain Intake22;
    private PathChain Outtake2;
    private PathChain Intake31;
    private PathChain Intake32;
    private PathChain Outtake3;
    private PathChain End;

    public void buildPaths() {
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .setBrakingStrength(outtakeBrake)
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setBrakingStrength(outtakeBrake)
                .build();
        Intake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(intakeBreak)
                .build();
        Intake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1, intake2p2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setBrakingStrength(outtakeBrake)
                .build();
        Intake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(intakeBreak)
                .build();
        Intake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Outtake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setBrakingStrength(outtakeBrake)
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake, end))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);

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
        new Thread(() -> {
            try {
                cameraControls();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    public void statePathUpdate() {
//        telemetry.addData("step", pathState);
//        telemetry.addData("busy", follower.isBusy());
//        telemetry.addData("heading", follower.getHeading());
//        telemetry.addData("velocity", follower.getVelocity());
//        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();
        List<AprilTagDetection> detectedTags = aprilTag.getDetections();
        if(pathState != PathState.END && pathState != PathState.STOP){
            aiming(detectedTags);
        }else{
            turret.setTargetPosition(0);
        }
        switch (pathState) {
            case PRELOAD:
                move(Preload, PathState.SHOOTPRE);
                break;
            case SHOOTPRE:
                shoot(PathState.INTAKE1);
                break;
            case INTAKE1:
                moveIntake(Intake1, PathState.OUTTAKE1);
                break;
            case OUTTAKE1:
                move(Outtake1, PathState.SHOOT1);
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
    }

    @Override
    public void loop() {
        follower.update();
        xPos = follower.getPose().getX();
        yPos = follower.getPose().getY();
        heading = follower.getPose().getHeading();
        turretPos = turret.getCurrentPosition();
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
            follower.followPath(path,0.45, true); // 0.35 -> 0.45
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
        range = goalPos.findRange(xPos, yPos);
        flyWheel1.setVelocity(targetV);
        if (range<40) {
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
        if (actionTimer.getElapsedTime() > 3000) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 20) { // SIDE DEPENDENT
                camRange = detection.ftcPose.range + camOffsetX;

                bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(2.5/range)); // SIDE DEPENDENT
                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
                bearing = Math.toRadians(bearing);

                if(!gamepad1.x){
                    goalPos.update(xPos, yPos, bearing, camRange);
                }
                break;
            }
        }

        //required turret angle
        double turretTarget = goalPos.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT\
        if (turretTarget > 200) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < -200) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));
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
    public void cameraControls() throws InterruptedException {
        Thread.sleep(3000);
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // exposure and gain
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            gainControl.setGain(100);
        }
    }

    public double toFWV(double r){
        return (0.00673 * range * range) + (5.54 * range) +  (1162);
    }
}