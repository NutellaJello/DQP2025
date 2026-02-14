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


@TeleOp(name = "Blue Teleop", group = "TeleOp") // SIDE RED/BLUE

public class BlueTeleopWebcam extends LinearOpMode { // SIDE
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean gainSet = false;
    boolean fieldCentric = true;
    private ElapsedTime opModeTimer = new ElapsedTime();

    double hOffset = 0.25;
    double feedPower = 1;

    boolean useWebcam = true;
    double intakePower = 0;
    double FW1Target = 0;
    double flapPos = 0.2;
    double stopperPos = 0.9;
    double turretPos;
    double FWV1;
    double idlePower = 0;
    double camRange = 0;
    double lastRange;
    double bearing = 0;
    double elevation = 0;
    GoalPos goalPos = new GoalPos(30,-50); // SIDE 50/-50

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
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = 0; //495/90
    private final double highLimit = 1865;
    double p = 350;
    double d = 0;
    double i = 0;
    double f = 13.5;


    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d,  f);

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
        flyWheel1.setPIDFCoefficients( DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

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


        initWebcam();
        waitForStart();
        opModeTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            xPos = follower.getPose().getX();
            yPos = follower.getPose().getY();
            heading = follower.getPose().getHeading();

            if(!gainSet && opModeTimer.seconds() > 0.5){
                cameraControls();
            }
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();
            // all the movement controls.
            if(!holding){
                drivetrain.Teleop(gamepad1, heading, telemetry, fieldCentric);
            }

            turretPos = turret.getCurrentPosition();

            FWV1 = flyWheel1.getVelocity();
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

           // botTelemetry();

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





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idlePower = 1800;
        }else if(gamepad1.dpad_right){
            idlePower = 0;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            intakePower = 1;
        } else if(gamepad1.a){
            intakePower = -0.5;
        }
    }





    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 20) { // SIDE 24/20
                camRange = detection.ftcPose.range + camOffsetX;
                bearing = detection.ftcPose.bearing - Math.toDegrees(Math.atan(hOffset/range)); // SIDE +/-
//                bearing = Math.toRadians(detection.ftcPose.bearing);
//                double xCam = camRange * Math.cos(bearing); //cartesian coordinates in cam frame of reference
//                double yCam = camRange * Math.sin(bearing) - camOffset;
//                range = Math.hypot(xCam, yCam); // corrected range
//                bearing = Math.toDegrees(Math.atan2(yCam, xCam)); // corrected bearing

                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
                bearing = Math.toRadians(bearing);
                goalPos.update(xPos, yPos, bearing, camRange);
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
        if (turretTarget > 360 + 30) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < 0 - 30) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));

        if (gamepad2.a){
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
            if(gamepad2.y){
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turret.setPower(turretPower);
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

        //setting flap position
        //flapPos = Math.pow(range * 0.00158, 0.1) - 0.159;
        if (range<45) {
            flapPos = 0;
            feedPower = 1;
            hOffset = -3.5; // SIDE -1.5/-3.5
        }else if (range < 95){
            flapPos = 0.195;
            feedPower = 1;
            hOffset = -3.5; // SIDE -1.5/-3.5
        }else{
            flapPos = 0.24;
            feedPower = 0.67;
            hOffset = -0.5; // SIDE 2.5/-0.5
        }

        flap.setPosition(flapPos);


        if (gamepad1.x) {

            //setting target velocity
            FW1Target = (0.00673 * range * range) + (5.54 * range) +  (1162);  //10.27 * range + 1300;2.937 * range + 716.11;

            if (range< 45){
                FW1Target-=100;
            }

            if(FWV1 >= FW1Target){
                stopperPos = 0.973; // open
                intakePower = feedPower;
            }
        } else {
            intakePower = 0;
            stopperPos = 0.9; // closed
            FW1Target = idlePower;
        }
        flyWheel1.setVelocity(FW1Target);
        stopper.setPosition(stopperPos);
    }


    public void brake(){
        if(gamepad1.x || (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8)){
            if(!holding){
                PathChain hold = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(xPos + 0.00000001, yPos, heading)))
                        .setConstantHeadingInterpolation(heading)
                        .build();
                follower.followPath(hold,0.65, true);
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
        //telemetry.addData("goal est", goalPos);
        telemetry.addData("pidValues", flyWheel1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("flap", flapPos);
        telemetry.addData("targetVel", FW1Target);
        telemetry.addData("currVel", flyWheel1.getVelocity());
        telemetry.addData("range", range);
//        telemetry.addData("robot angle",Math.toDegrees(follower.getPose().getHeading()));
//                follower.getPose().getX(), follower.getPose().getY()
//        ) - Math.toDegrees(follower.getPose().getHeading())));
//        telemetry.addData("target bearing", (goalPos.findAngle(follower.getPose().getX(), follower.getPose().getY())));
        telemetry.update();

    }


}