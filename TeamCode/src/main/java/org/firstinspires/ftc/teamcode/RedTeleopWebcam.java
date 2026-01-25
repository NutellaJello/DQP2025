package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Red Teleop", group = "TeleOp")

public class RedTeleopWebcam extends LinearOpMode {
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel;
    private Servo pusher;


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean fieldCentric = true;

    boolean useWebcam = true;
    double intakePower = 0;
    double flyWheelVTarget = 0;
    double pusherPos = 0.4;
    double turretPos;
    double flyWheelVel;
    double idlePower = 0;
    double camRange;
    double lastRange;
    double bearing;
    double elevation;
    GoalPos goalPos = new GoalPos(30,50);
    int fireState = 0;      // 0 = idle, 1 = spinup, 2 = pushUp, 3 = pushDown
    ElapsedTime fireTimer = new ElapsedTime();
    ElapsedTime turretPidTimer = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();

    private Follower follower;
    private boolean holding = false;
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double xEst;
    private double yEst;
    private double range1,range2,range3;
    private double range;
    private double xPos = 0, yPos = 0, heading = 0;

    @Override
    public void runOpMode() {
        if (visionPortal != null) {
            visionPortal.close();
            sleep(250);
        }
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap);
        follower = Constants.createTeleopFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(0)));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE

        flyWheel = hardwareMap.get(DcMotorEx.class, "FW");
        flyWheel.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.REVERSE);


        initWebcam();
        waitForStart();

        new Thread(() -> {
            try {
                cameraControls();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
        turretPidTimer.reset();
        releaseTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            xPos = follower.getPose().getX();
            yPos = follower.getPose().getY();
            heading = follower.getPose().getHeading();
            //telemetryWebcam();
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();

            // all the movement controls.
            if(!holding){
                drivetrain.Teleop(gamepad1, telemetry, fieldCentric);
            }

            turretPos = turret.getCurrentPosition();

            flyWheelVel = flyWheel.getVelocity();

            setIdlePower();

            setIntakePower();

            aiming(detectedTags);

            firing();

            brake();

            botTelemetry();

        }
        visionPortal.close();

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
        if (useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.setCameraResolution(new Size(640, 480)); //640 480

        builder.enableLiveView(true);
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





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idlePower = 1800;
        }else if(gamepad1.dpad_right){
            idlePower = 0;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1 && !gamepad1.x) {
            intakePower = 1;
        } else if(gamepad1.a){
            intakePower = -1;
        }else {
            intakePower = 0;
        }
        intake.setPower(intakePower);
    }





    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 24) { // SIDE DEPENDENT
                camRange = (detection.ftcPose.range + lastRange) / 2;
                lastRange = detection.ftcPose.range;
                bearing = detection.ftcPose.bearing + Math.toDegrees(heading) + turretPos * 90/489;   // in degrees
                //goalPos.update(xPos, yPos, Math.toRadians(bearing));
                if(!gamepad1.x){
                    goalPos.update(xPos, yPos, Math.toRadians(bearing), camRange);
                }
                xEst = xPos + camRange * Math.cos(Math.toRadians(bearing));
                yEst = yPos + camRange * Math.sin(Math.toRadians(bearing));
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
                - Math.toDegrees(heading)
                + (Math.toDegrees(Math.atan(3 / range))); // SIDE DEPENDENT
        if (turretTarget > 200) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < -200) {
            turretTarget += 360;
        }
        turretTarget = 489.0 / 90.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, -489, 489); //(Math.toDegrees(Math.atan(3.5 / range)));

        if (gamepad2.a){
            double turretPower = 0;
            if(!a2Press){
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                a2Press = true;
            }
            if(gamepad2.left_bumper){
                turretPower = 0.45;
            } else if (gamepad2.right_bumper){
                turretPower = -0.45;
            }else{
                turretPower = 0;
            }
            if ((turretPower > 0 && turretPos < 489) || (turretPower < 0 && turretPos > -489)) { //left, right limits
                turret.setPower(turretPower);
            } else {
                turret.setPower(0);   // stop at limits
            }
            if(hasEst){
                goalPos.setX(xEst);
                goalPos.setY(yEst);
            }
        }else{ // manual aiming
            if(a2Press){
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                a2Press = false;
            }
            turret.setTargetPosition((int) turretTarget);
        }
    }


    public void firing(){
        range = goalPos.findRange(xPos, yPos);
        if (gamepad1.x) {
            if (fireState == 0) {
                // Step 1: Start flywheel
                flyWheelVTarget = 10.27 * range + 1300;// 10.27 * range + 1300
                fireState = 1;   // go to SPINNING_UP
            } else if (fireState == 1) {
                // Step 2: Wait until flywheel reaches speed
                if (flyWheel.getVelocity() >= flyWheelVTarget ) {
                    pusherPos = 0.95;   // fire
                    fireTimer.reset();
                    fireState = 2;     // go to PUSH_UP
                }
            } else if (fireState == 2) {
                // Step 3: pusher up for 0.15s
                if (fireTimer.seconds() > 0.3) {
                    pusherPos = 0.4;   // retract
                    fireTimer.reset();
                    fireState = 3;    // go to PUSH_DOWN
                }
            } else if (fireState == 3) {
                intake.setPower(1);
                // Step 4: wait 0.15s then fire again
                if (fireTimer.seconds() > 0.5) {
                    fireState = 1;   // loop back→ SPINNING_UP → fire again
                }
            }
        } else {
            // X RELEASED → reset firing system
            pusherPos = 0.4;
            flyWheelVTarget = idlePower;
            fireState = 0;
        }
        pusher.setPosition(pusherPos);
        flyWheel.setVelocity(flyWheelVTarget);
    }


    public void brake(){
        if(gamepad1.x || (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8)){
            if(!holding){
                PathChain hold = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(xPos + 0.00000001, yPos, heading)))
                        .setConstantHeadingInterpolation(heading)
                        .build();
                follower.followPath(hold, true);
                holding = true;
            }
        }else{
            if(holding) {
                follower.breakFollowing();
                holding = false;
            }
        }
    }

    public void botTelemetry(){
        telemetry.addData("range", range);
//        telemetry.addData("turret position", turretPos);
        telemetry.addData("cam range", camRange);
//        telemetry.addData("robot angle",Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("turret target", (int) 489.0/90 * (goalPos.findAngle(
//                follower.getPose().getX(), follower.getPose().getY()
//        ) - Math.toDegrees(follower.getPose().getHeading())));
//        telemetry.addData("target bearing", (goalPos.findAngle(follower.getPose().getX(), follower.getPose().getY())));
        telemetry.update();

    }


}
