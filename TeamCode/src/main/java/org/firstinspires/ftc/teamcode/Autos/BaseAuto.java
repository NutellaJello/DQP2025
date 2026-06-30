package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public abstract class BaseAuto extends OpMode {

    // === Hardware ===
    protected DcMotorEx intake;
    protected DcMotorEx turret;
    protected DcMotorEx flyWheel1;
    protected Servo stopper;
    protected Servo flap;
    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal;
    protected Follower follower;
    protected Timer actionTimer, opmodeTimer;
    protected GoalPos goalPos;

    // === Pose tracking ===
    protected double xPos, yPos, heading;
    protected double turretPos;

    // === Shooting / aiming state ===
    protected double range;
    protected double flapPos = 0.2;
    protected double camRange;
    protected double bearing;
    protected boolean hasEst = false;

    // === Motion state ===
    protected boolean moving = false;
    protected volatile boolean gainSet = false;

    // === Fixed constants ===
    protected final double camOffsetX = 2.0; // inches: camera forward of turret rotation axis — re-measure if remounted
    protected final double startingAngle = 0; // angle from straight forward (counterclockwise)

    // === PIDF — p varies by file; d/i/f are universal ===
    protected double p = 400;
    protected double d = 0;
    protected double i = 0;
    protected double f = 13.5;
    protected PIDFCoefficients fwPID;

    // === Per-file configuration ===
    protected abstract GoalPos createGoalPos();
    protected abstract Pose getStartPose();
    protected abstract double getFWVConstant();

    // === Per-file behavior ===
    protected abstract void buildPaths();
    protected abstract void statePathUpdate();

    /**
     * Initializes all hardware shared across every auto.
     * Subclass init() must call this after setting up subclass-only hardware (e.g. flyWheel2).
     */
    protected void baseInit() {
        fwPID = new PIDFCoefficients(p, i, d, f);
        goalPos = createGoalPos();

        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createAutoFollower(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, fwPID);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        stopper.setPosition(0.9);
        flap.setPosition(0.2);

        buildPaths();
        initWebcam();
        follower.setPose(getStartPose());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
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

    @Override
    public void stop() {
        stopper.setPosition(0.9);
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    public void move(PathChain path, Runnable onComplete) {
        if (!moving) {
            follower.followPath(path, true);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            moving = false;
            onComplete.run();
        }
    }

    /**
     * Follows path at reduced speed while running intake, then calls onComplete when done.
     * @param holonomic  pass true for normal autos, false for RedCloseGate/RedClose15 intake paths
     * @param timeoutMs  50 for close autos, 1500 for far autos (extra dwell for loading zone)
     */
    public void moveIntake(PathChain path, double speed, boolean holonomic, int timeoutMs, Runnable onComplete) {
        if (!moving) {
            follower.followPath(path, speed, holonomic);
            intake.setPower(1);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > timeoutMs) {
            intake.setPower(0);
            moving = false;
            onComplete.run();
        }
    }

    public void waitMs(int milliseconds, Runnable onComplete) {
        if (actionTimer.getElapsedTime() > milliseconds) {
            onComplete.run();
        }
    }

    public double toFWV(double r) {
        return (0.00673 * r * r) + (5.54 * r) + getFWVConstant();
    }

    protected void initWebcam() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(3);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void cameraControls() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            gainControl.setGain(100);
            gainSet = true;
        }
    }
}
