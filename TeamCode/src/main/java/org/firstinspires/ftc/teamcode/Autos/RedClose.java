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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Close", group = "Autos")
public class RedClose extends OpMode {
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
    GoalPos goalPos = new GoalPos(147,144);
    private double xPos = 0, yPos = 0, heading = 0;
    private double range;
    private final double startingAngle = 180; // angle from straight forward (counterclockwise)
    private final double lowLimit = -450;
    private final double highLimit = 730;
    private double camRange;
    private double bearing;
    private double xEst;
    private double yEst;
    private final double camOffsetX = 2;
    private double turretPos;
    private boolean hasEst = false;
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
        END,
        STOP
    }

    private PathState pathState;
    //positions
    private final Pose start = new Pose(130, 142, Math.toRadians(221-360));
    private final Pose outtakePre = new Pose(95, 90, Math.toRadians(-90));
    private final Pose outtake = new Pose(100, 90, Math.toRadians(-90));
    private final Pose intake1 = new Pose(130, 88, Math.toRadians(0));
    private final Pose intake2p1 = new Pose(100, 65, Math.toRadians(0));
    private final Pose intake2p2 = new Pose(133, 65 - 4, Math.toRadians(0));
    private final Pose end = new Pose(122, 77, Math.toRadians(0));

    //Paths
    private PathChain Preload;
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
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setConstantHeadingInterpolation(0)
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setLinearHeadingInterpolation(0,Math.toRadians(-90))
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
                .setLinearHeadingInterpolation(0,Math.toRadians(-90))
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

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);


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
                shoot(900,900, PathState.INTAKE1);
                break;
            case INTAKE1:
                moveIntake(Intake1, PathState.OUTTAKE1);
                break;
            case OUTTAKE1:
                move(Outtake1, PathState.SHOOT1);
                break;
            case SHOOT1:
                shoot(900,900,PathState.INTAKE21);
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
                shoot(900,900,PathState.END);
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
            follower.followPath(path,0.35, true);
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
        range = goalPos.findRange(xPos, yPos);
        flyWheel1.setVelocity(2.937 * range + 690);
        flyWheel2.setVelocity(2.937 * range + 690);
        double FWV1 = flyWheel1.getVelocity();
        double FWV2 = flyWheel2.getVelocity();
        telemetry.addData("FW1",FWV1);
        telemetry.addData("FW2",FWV2);
        if(actionTimer.getElapsedTime() > 1000){
            stopper.setPosition(0.973);
            intake.setPower(0.6);
        }
        if (actionTimer.getElapsedTime() > 4500) {
            intake.setPower(0);
            stopper.setPosition(0.9);
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
            pathState = nextPath;
            actionTimer.resetTimer();
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
                camRange = detection.ftcPose.range + camOffsetX;
                bearing = detection.ftcPose.bearing;
//                bearing = Math.toRadians(detection.ftcPose.bearing);
//                double xCam = camRange * Math.cos(bearing); //cartesian coordinates in cam frame of reference
//                double yCam = camRange * Math.sin(bearing) - camOffset;
//                range = Math.hypot(xCam, yCam); // corrected range
//                bearing = Math.toDegrees(Math.atan2(yCam, xCam)); // corrected bearing

                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
                bearing = Math.toRadians(bearing);
                if(!gamepad1.x){
                    goalPos.update(xPos, yPos, bearing, camRange);
                }
                xEst = xPos + camRange * Math.cos(bearing);
                yEst = yPos + camRange * Math.sin(bearing);
                if(!hasEst){
                    goalPos.setX(xEst);
                    goalPos.setY(yEst);
                }
                hasEst = true;
                break;
            }
        }

        //required turret angle
        double turretTarget = goalPos.findAngle(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading); // SIDE DEPENDENT
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
}
