package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Close Adaptive", group = "Autos")
public class RedCloseAdaptive extends OpMode {
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor greenLocator;
    private VisionPortal visionPortal;
    private boolean gainSet = false;

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private boolean moving = false;

    private GoalPos goalPos = new GoalPos(147, 144);

    private double xPos = 0;
    private double yPos = 0;
    private double heading = 0;
    private double range = 72;
    private final double startingAngle = 0;
    private final double lowLimit = 0;
    private final double highLimit = 1865;
    private final double camOffsetX = 2;
    private double turretPos;
    private double flapPos = 0.2;
    private double camRange;
    private double bearing;

    private static final double AUTO_LIMIT_SEC = 30.0;
    private static final double END_BUFFER_SEC = 2.0;
    private static final double MIN_EXTRA_CYCLE_SEC = 4.5;
    private static final long FIXED_SHOOT_MS = 2800;
    private static final long EXTRA_COLLECT_MS = 650;
    private static final long EXTRA_SHOOT_MS = 2100;

    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;
    private static final double TRACK_ALPHA = 0.35;
    private static final double TRACK_STALE_SEC = 0.6;

    // Collection area bounds for Red Close side (soft-clamped for safety).
    private static final double COLLECT_X_MIN = 108;
    private static final double COLLECT_X_MAX = 139;
    private static final double COLLECT_Y_MIN = 34;
    private static final double COLLECT_Y_MAX = 92;

    private Pose trackedArtifactPose = null;
    private double trackedArtifactScore = 0;
    private double lastArtifactSeenSec = -1;
    private ArtifactObservation lastObservation = null;
    private int extraCyclesCompleted = 0;

    private double p = 380;
    private double d = 0;
    private double i = 0;
    private double f = 13.5;
    private PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d, f);
    private final double braking = 0.9;

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
        EXTRA_SELECT,
        EXTRA_SCAN,
        EXTRA_TO_INTAKE,
        EXTRA_COLLECT,
        EXTRA_TO_OUTTAKE,
        EXTRA_SHOOT,
        END,
        STOP
    }

    private PathState pathState;

    // Fixed close-side intake lanes.
    private final Pose start = new Pose(130, 142, Math.toRadians(-139));
    private final Pose outtakePre = new Pose(93, 90, Math.toRadians(0));
    private final Pose outtake = new Pose(100, 90, Math.toRadians(0));
    private final Pose intake1 = new Pose(125, 88, Math.toRadians(0));
    private final Pose intake2p1 = new Pose(100, 67, Math.toRadians(0));
    private final Pose intake2p2 = new Pose(128, 65, Math.toRadians(0));
    private final Pose intake3p1 = new Pose(100, 39, Math.toRadians(0));
    private final Pose intake3p2 = new Pose(128, 39, Math.toRadians(0));
    private final Pose end = new Pose(112, 77, Math.toRadians(0));

    // Scan waypoints when tracker has no confident artifact target.
    private final Pose[] scanWaypoints = new Pose[] {
            new Pose(122, 82, Math.toRadians(0)),
            new Pose(122, 62, Math.toRadians(0)),
            new Pose(122, 44, Math.toRadians(0))
    };
    private int scanIndex = 0;

    private PathChain preload;
    private PathChain fixedIntake1;
    private PathChain fixedOuttake1;
    private PathChain fixedIntake21;
    private PathChain fixedIntake22;
    private PathChain fixedOuttake2;
    private PathChain fixedIntake31;
    private PathChain fixedIntake32;
    private PathChain fixedOuttake3;
    private PathChain endPath;

    private PathChain extraScanPath;
    private PathChain extraToIntake;
    private PathChain extraToOuttake;

    private static class ArtifactObservation {
        final Pose pose;
        final double score;
        final String color;
        final int px;
        final int py;

        ArtifactObservation(Pose pose, double score, String color, int px, int py) {
            this.pose = pose;
            this.score = score;
            this.color = color;
            this.px = px;
            this.py = py;
        }
    }

    public void buildFixedPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .setBrakingStrength(braking)
                .build();

        fixedIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setConstantHeadingInterpolation(0)
                .build();

        fixedOuttake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
                .build();

        fixedIntake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setConstantHeadingInterpolation(0)
                .build();

        fixedIntake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1, intake2p2))
                .setConstantHeadingInterpolation(0)
                .build();

        fixedOuttake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
                .build();

        fixedIntake31 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake3p1))
                .setConstantHeadingInterpolation(0)
                .build();

        fixedIntake32 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p1, intake3p2))
                .setConstantHeadingInterpolation(0)
                .build();

        fixedOuttake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3p2, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
                .build();

        endPath = follower.pathBuilder()
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
        stopper.setPosition(0.9);

        flap = hardwareMap.get(Servo.class, "flap");
        flap.setDirection(Servo.Direction.FORWARD);
        flap.setPosition(flapPos);

        buildFixedPaths();
        initVision();
        follower.setPose(start);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
        actionTimer.resetTimer();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    @Override
    public void loop() {
        follower.update();
        xPos = follower.getPose().getX();
        yPos = follower.getPose().getY();
        heading = follower.getPose().getHeading();
        turretPos = turret.getCurrentPosition();

        if (opmodeTimer.getElapsedTime() > 500 && !gainSet) {
            cameraControls();
        }

        updateArtifactTracker();
        statePathUpdate();

        telemetry.addData("state", pathState);
        telemetry.addData("extraCycles", extraCyclesCompleted);
        telemetry.addData("time", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("tracker", trackedArtifactPose == null ? "none" :
                String.format("(%.1f, %.1f) score=%.1f", trackedArtifactPose.getX(), trackedArtifactPose.getY(), trackedArtifactScore));
        if (lastObservation != null) {
            telemetry.addData("obs", "%s (%d,%d) s=%.1f", lastObservation.color, lastObservation.px, lastObservation.py, lastObservation.score);
        }
        telemetry.update();
    }

    public void statePathUpdate() {
        if (pathState != PathState.END && pathState != PathState.STOP && opmodeTimer.getElapsedTimeSeconds() < 29.5) {
            aiming(aprilTag.getDetections());
        } else {
            turret.setTargetPosition(0);
        }

        switch (pathState) {
            case PRELOAD:
                move(preload, PathState.SHOOTPRE);
                break;
            case SHOOTPRE:
                shoot(PathState.INTAKE1, FIXED_SHOOT_MS, 1.0);
                break;
            case INTAKE1:
                moveIntake(fixedIntake1, PathState.OUTTAKE1, 0.35, 50);
                break;
            case OUTTAKE1:
                move(fixedOuttake1, PathState.SHOOT1);
                break;
            case SHOOT1:
                shoot(PathState.INTAKE21, FIXED_SHOOT_MS, 1.0);
                break;
            case INTAKE21:
                move(fixedIntake21, PathState.INTAKE22);
                break;
            case INTAKE22:
                moveIntake(fixedIntake22, PathState.OUTTAKE2, 0.35, 50);
                break;
            case OUTTAKE2:
                move(fixedOuttake2, PathState.SHOOT2);
                break;
            case SHOOT2:
                shoot(PathState.INTAKE31, FIXED_SHOOT_MS, 1.0);
                break;
            case INTAKE31:
                move(fixedIntake31, PathState.INTAKE32);
                break;
            case INTAKE32:
                moveIntake(fixedIntake32, PathState.OUTTAKE3, 0.35, 50);
                break;
            case OUTTAKE3:
                move(fixedOuttake3, PathState.SHOOT3);
                break;
            case SHOOT3:
                shoot(PathState.EXTRA_SELECT, FIXED_SHOOT_MS, 1.0);
                break;

            case EXTRA_SELECT:
                if (!hasTimeForExtraCycle()) {
                    setPathState(PathState.END);
                    break;
                }
                if (trackedArtifactPose != null) {
                    buildExtraPaths(trackedArtifactPose);
                    setPathState(PathState.EXTRA_TO_INTAKE);
                    moving = false;
                } else {
                    buildScanPath();
                    setPathState(PathState.EXTRA_SCAN);
                    moving = false;
                }
                break;
            case EXTRA_SCAN:
                move(extraScanPath, PathState.EXTRA_SELECT);
                break;
            case EXTRA_TO_INTAKE:
                moveIntake(extraToIntake, PathState.EXTRA_COLLECT, 0.37, 70);
                break;
            case EXTRA_COLLECT:
                intake.setPower(1);
                if (actionTimer.getElapsedTime() > EXTRA_COLLECT_MS) {
                    intake.setPower(0);
                    setPathState(PathState.EXTRA_TO_OUTTAKE);
                    moving = false;
                }
                break;
            case EXTRA_TO_OUTTAKE:
                move(extraToOuttake, PathState.EXTRA_SHOOT);
                break;
            case EXTRA_SHOOT:
                shoot(PathState.EXTRA_SELECT, EXTRA_SHOOT_MS, 1.0);
                break;

            case END:
                move(endPath, PathState.STOP);
                break;
            case STOP:
                intake.setPower(0);
                stopper.setPosition(0.9);
                flyWheel1.setVelocity(0);
                break;
        }
    }

    private boolean hasTimeForExtraCycle() {
        return opmodeTimer.getElapsedTimeSeconds() < (AUTO_LIMIT_SEC - END_BUFFER_SEC - MIN_EXTRA_CYCLE_SEC);
    }

    private void buildScanPath() {
        Pose scanTarget = scanWaypoints[scanIndex];
        scanIndex = (scanIndex + 1) % scanWaypoints.length;
        extraScanPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), scanTarget))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    private void buildExtraPaths(Pose intakeTarget) {
        Pose current = follower.getPose();
        extraToIntake = follower.pathBuilder()
                .addPath(new BezierLine(current, intakeTarget))
                .setConstantHeadingInterpolation(0)
                .build();

        extraToOuttake = follower.pathBuilder()
                .addPath(new BezierLine(intakeTarget, outtake))
                .setConstantHeadingInterpolation(0)
                .setBrakingStrength(braking)
                .build();
    }

    public void move(PathChain path, PathState nextPath) {
        if (!moving) {
            follower.followPath(path, true);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > 50) {
            setPathState(nextPath);
            moving = false;
        }
    }

    public void moveIntake(PathChain path, PathState nextPath, double followPower, long settleMs) {
        if (!moving) {
            follower.followPath(path, followPower, true);
            intake.setPower(1);
            moving = true;
        }
        if (!follower.isBusy() && actionTimer.getElapsedTime() > settleMs) {
            intake.setPower(0);
            setPathState(nextPath);
            moving = false;
        }
    }

    public void shoot(PathState nextPath, long shootMs, double feedPower) {
        range = goalPos.findRange(xPos, yPos);
        double targetV = toFWV(range);
        flyWheel1.setVelocity(targetV);

        if (range < 40) {
            flapPos = 0;
        } else if (range < 95) {
            flapPos = 0.2;
        } else {
            flapPos = 0.24;
        }
        flap.setPosition(flapPos);

        if (flyWheel1.getVelocity() >= targetV) {
            stopper.setPosition(0.973);
            intake.setPower(feedPower);
        } else {
            stopper.setPosition(0.9);
            intake.setPower(0);
        }

        if (actionTimer.getElapsedTime() > shootMs) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            if (pathState == PathState.EXTRA_SHOOT && nextPath == PathState.EXTRA_SELECT) {
                extraCyclesCompleted++;
            }
            setPathState(nextPath);
        }
    }

    private void updateArtifactTracker() {
        if (purpleLocator == null || greenLocator == null) {
            return;
        }
        ArtifactObservation best = detectBestArtifact();
        double now = opmodeTimer.getElapsedTimeSeconds();
        if (best != null) {
            lastObservation = best;
            if (trackedArtifactPose == null) {
                trackedArtifactPose = best.pose;
            } else {
                double x = trackedArtifactPose.getX() * (1.0 - TRACK_ALPHA) + best.pose.getX() * TRACK_ALPHA;
                double y = trackedArtifactPose.getY() * (1.0 - TRACK_ALPHA) + best.pose.getY() * TRACK_ALPHA;
                trackedArtifactPose = new Pose(x, y, 0);
            }
            trackedArtifactScore = best.score;
            lastArtifactSeenSec = now;
        } else if (trackedArtifactPose != null && lastArtifactSeenSec > 0 && now - lastArtifactSeenSec > TRACK_STALE_SEC) {
            trackedArtifactPose = null;
            trackedArtifactScore = 0;
        }
    }

    private ArtifactObservation detectBestArtifact() {
        ArtifactObservation purple = detectBestFromLocator(purpleLocator, "purple");
        ArtifactObservation green = detectBestFromLocator(greenLocator, "green");

        if (purple == null) {
            return green;
        }
        if (green == null) {
            return purple;
        }
        return purple.score >= green.score ? purple : green;
    }

    private ArtifactObservation detectBestFromLocator(ColorBlobLocatorProcessor locator, String colorName) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        if (blobs == null || blobs.isEmpty()) {
            return null;
        }

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 120, 20000, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 4.0, blobs);

        ArtifactObservation best = null;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            RotatedRect fit = blob.getBoxFit();
            int px = (int) fit.center.x;
            int py = (int) fit.center.y;

            // Ignore tiny/high artifacts far from intake horizon.
            if (py < 130) {
                continue;
            }

            Pose estimate = imageToFieldPose(px, py);
            double centerPenalty = Math.abs(px - FRAME_WIDTH * 0.5) * 0.8;
            double score = blob.getContourArea() + py * 0.35 - centerPenalty;

            ArtifactObservation obs = new ArtifactObservation(estimate, score, colorName, px, py);
            if (best == null || obs.score > best.score) {
                best = obs;
            }
        }
        return best;
    }

    private Pose imageToFieldPose(int px, int py) {
        double nx = (px / (double) FRAME_WIDTH - 0.5) * 2.0;
        double ny = py / (double) FRAME_HEIGHT;

        // Coarse camera-plane to floor-plane approximation for intake targeting.
        double forward = Range.clip(34 - ny * 22, 8, 34);
        double lateral = Range.clip(-nx * (6.0 + 0.25 * forward), -14, 14);

        double fx = xPos + forward * Math.cos(heading) - lateral * Math.sin(heading);
        double fy = yPos + forward * Math.sin(heading) + lateral * Math.cos(heading);

        fx = Range.clip(fx, COLLECT_X_MIN, COLLECT_X_MAX);
        fy = Range.clip(fy, COLLECT_Y_MIN, COLLECT_Y_MAX);
        return new Pose(fx, fy, 0);
    }

    public void aiming(List<AprilTagDetection> detectedTags) {
        if (gainSet) {
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) {
                    camRange = detection.ftcPose.range + camOffsetX;
                    bearing = detection.ftcPose.bearing + Math.toDegrees(Math.atan(2.5 / range));
                    bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180 / 976;
                    bearing = Math.toRadians(bearing);
                    goalPos.update(xPos, yPos, bearing, camRange);
                    break;
                }
            }
        }

        double turretTarget = goalPos.findAngle(xPos, yPos) - startingAngle - Math.toDegrees(heading);
        if (turretTarget > 390) {
            turretTarget -= 360;
        } else if (turretTarget < -30) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget;
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit);
        turret.setTargetPosition((int) turretTarget);
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(3);

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asImageCoordinates(0, 120, FRAME_WIDTH - 1, FRAME_HEIGHT - 1))
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asImageCoordinates(0, 120, FRAME_WIDTH - 1, FRAME_HEIGHT - 1))
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT));
        builder.enableLiveView(false);
        builder.addProcessor(aprilTag);
        builder.addProcessor(purpleLocator);
        builder.addProcessor(greenLocator);
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

    public double toFWV(double r) {
        return (0.00673 * r * r) + (5.54 * r) + 1162;
    }
}
