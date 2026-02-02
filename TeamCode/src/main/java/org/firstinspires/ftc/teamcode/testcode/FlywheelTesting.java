package org.firstinspires.ftc.teamcode.testcode;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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


@TeleOp(name = "FW Testing", group = "TeleOp")

public class FlywheelTesting extends LinearOpMode {
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;
    private Servo flap;


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean fieldCentric = true;

    boolean useWebcam = true;
    double intakePower = 0;
    double FW1Target = 0;
    double FW2Target = 0;
    double stopperPos = 0.9;

    double topFlapPos = 0;
    double flapPos = 0.2;

    double shotFreq = 0;
    double feedPower = 1;

    double p = 140;
    double d = 0;
    double i = 0;
    double f = 13.5;


    PIDFCoefficients fwPID = new PIDFCoefficients(p, 0, 0,  f);


    double turretPos;
    double FWV1;
    double FWV2;
    double idlePower = 0;
    double camRange = 0;
    double lastRange;
    double bearing = 0;
    double elevation = 0;
    GoalPos goalPos = new GoalPos(30,50);
    ElapsedTime clickTimer1 = new ElapsedTime();
    ElapsedTime clickTimer2 = new ElapsedTime();
    ElapsedTime shootDelay = new ElapsedTime();
    boolean atSpeed = false;
    private Follower follower;
    private boolean holding = false;
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double xEst;
    private double yEst;
    private double range;
    private double xPos = 0, yPos = 0, heading = 0;
    private final double camOffsetX = 2; //inches (not really inches) forward of center
    private final double camOffsetY = 0; //inches (not really inches) right of center
    private final double startingAngle = 180; // angle from straight forward (counterclockwise)
    private final double lowLimit = -450;
    private final double highLimit = 890;


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
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, fwPID);


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

        flap = hardwareMap.get(Servo.class, "flap");


        initWebcam();
        waitForStart();

        new Thread(() -> {
            try {
                cameraControls();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();

        while (opModeIsActive()) {
            follower.update();
            xPos = follower.getPose().getX();
            yPos = follower.getPose().getY();
            heading = follower.getPose().getHeading();

            List<AprilTagDetection> detectedTags = aprilTag.getDetections();

            // all the movement controls.
            if(!holding){
                drivetrain.Teleop(gamepad1, telemetry, fieldCentric);
            }

            turretPos = turret.getCurrentPosition();

            FWV1 = flyWheel1.getVelocity();
            FWV2 = flyWheel2.getVelocity();
            setIdlePower();

            if(!gamepad1.x){
                setIntakePower();
            }

            aiming(detectedTags);

            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){
                firing();
            }


            brake();

            intake.setPower(intakePower);

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





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idlePower = 1800;
        }else if(gamepad1.dpad_right){
            idlePower = 0;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            intakePower = 0.8;
        } else if(gamepad1.a){
            intakePower = -0.5;
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
                - Math.toDegrees(heading);
        if (turretTarget > 200) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < -200) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));

        if (false){
            double turretPower = 0;
            if(!a2Press){
                turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                a2Press = true;
            }
            if(gamepad2.left_bumper){
                turretPower = 0.4;
            } else if (gamepad2.right_bumper){
                turretPower = -0.4;
            }else{
                turretPower = 0;
            }
            if ((turretPower > 0 && turretPos < highLimit) || (turretPower < 0 && turretPos > lowLimit)) { //left, right limits
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
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                a2Press = false;
            }
            turret.setTargetPosition((int) turretTarget);
        }
    }


    public void firing(){
        range = goalPos.findRange(xPos, yPos);
        if(gamepad2.dpad_up && clickTimer1.seconds() > 0.05){
            FW1Target += 50;
            clickTimer1.reset();
        } else if (gamepad2.dpad_down && clickTimer1.seconds() > 0.05) {
            FW1Target -= 50;
            clickTimer1.reset();
        }

        //changing PID
        if(gamepad2.y && clickTimer2.seconds() > 0.1){
            flapPos += 0.02;
            clickTimer2.reset();
        } else if (gamepad2.a && clickTimer2.seconds() > 0.1) {
            flapPos -= 0.02;
            clickTimer2.reset();
        }

        flap.setPosition(flapPos);

        if (gamepad1.x) {
            flyWheel1.setVelocity(FW1Target);
            flyWheel2.setVelocity(FW2Target);
            if(FWV1 >= FW1Target && FWV2 >= FW2Target){
                if(!atSpeed) {
                    atSpeed = true;
                    shootDelay.reset();
                }
                if (shootDelay.seconds() > shotFreq && atSpeed) {
                    stopperPos = 0.97; // open
                    intakePower = feedPower;
                }
            }
        } else {
            atSpeed = false;
            intakePower = 0;
            stopperPos = 0.9; // closed
            flyWheel1.setVelocity(0);
            flyWheel2.setVelocity(0);
        }


        PIDFCoefficients fwPID = new PIDFCoefficients(p, 0, 0,  f);
        flyWheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, fwPID);
        stopper.setPosition(stopperPos);
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
        telemetry.addData("Range", range);
        telemetry.addData("FW1 target", FW1Target);

        telemetry.addData("\nFW1 vel", FWV1);
        telemetry.addData("flap pos", flapPos);
        telemetry.addData("coefficients", flyWheel1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER) );

//        telemetry.addData("robot angle",Math.toDegrees(follower.getPose().getHeading()));
//                follower.getPose().getX(), follower.getPose().getY()
//        ) - Math.toDegrees(follower.getPose().getHeading())));
//        telemetry.addData("target bearing", (goalPos.findAngle(follower.getPose().getX(), follower.getPose().getY())));
        telemetry.update();

    }


}
