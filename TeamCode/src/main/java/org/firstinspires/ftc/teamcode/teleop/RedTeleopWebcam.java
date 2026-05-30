package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import org.firstinspires.ftc.teamcode.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Red Teleop", group = "TeleOp") // SIDE RED/BLUE

public class RedTeleopWebcam extends LinearOpMode { // SIDE
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean gainSet = false;
    boolean fieldCentric = false;
    private ElapsedTime opModeTimer = new ElapsedTime();
    private boolean streamStarted = false;
    private double camStreamingTime;

    double hOffset = 0.25;
    double feedPower = 1;

    boolean useWebcam = true;
    double intakePower = 0;
    double flywheelTarget = 0;
    double flapPos = 0.2;
    double stopperPos = 0.9;
    double turretPos;
    double flywheelVelocity1;
    double flywheelVelocity2;
    double idlePower = 0;
    double camRange = 0;
    double lastRange;
    double bearing = 0;
    double elevation = 0;
    GoalPos goal = new GoalPos(30,50, 15.5); // SIDE 50/-50

    private Follower follower;
    private boolean auto = false;
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double xEst;
    private double yEst;
    private double range;
    private double height;
    private double xPos = 0, yPos = 0, heading = 0;
    private final double camOffsetX = 2; //inches (not really inches) forward of center
    private final double camOffsetY = 0; //inches (not really inches) right of center
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = -2167; //495/90
    private final double highLimit =  340 ;
    double pidP = 400;
    double pidD = 0;
    double pidI = 0;
    double pidF = 13.5;


    PIDFCoefficients fwPID = new PIDFCoefficients(pidP, pidI, pidD,  pidF);

    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap, gamepad1, telemetry, false, false);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(0)));

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

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);

        flap = hardwareMap.get(Servo.class, "flap");
        flap.setDirection(Servo.Direction.FORWARD);


        initWebcam();
        waitForStart();
        opModeTimer.reset();

        while (opModeIsActive()) {
            // update variables
            follower.update();
            xPos = follower.getPose().getX();
            yPos = follower.getPose().getY();
            heading = follower.getPose().getHeading();
            range = goal.findRange(xPos, yPos);
            turretPos = turret.getCurrentPosition();
            flywheelVelocity1 = flyWheel1.getVelocity();
            flywheelVelocity2 = flyWheel2.getVelocity();

            // set initial values
            intakePower = 0;

            // configure webcam
            if(!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }
            if(!gainSet && streamStarted){
                gainSet = cameraControls();
            }

            if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                List<AprilTagDetection> detectedTags = aprilTag.getDetections();
                aiming(detectedTags);
            }

            // all the movement controls.
            if(!auto){
                drivetrain.drive(heading);
            }

            setIdlePower();

            if(!auto){ // disable manual intake if shooting/gate intake
                setIntakePower();
            }
            gate();



            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){ // disable if manual intake
                firing();
            }


            brake();

            intake.setPower(intakePower);
            flyWheel1.setVelocity(flywheelTarget);
            flyWheel2.setVelocity(flywheelTarget);
            stopper.setPosition(stopperPos);

            botTelemetry();

        }
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }






    private void initWebcam() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
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




    public boolean cameraControls(){
        boolean exposureOK = false;
        boolean gainOK = true;
        if(opModeTimer.milliseconds() > camStreamingTime + 500){
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if(exposureControl == null || gainControl == null){
                return false;
            }

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureOK = exposureControl.setExposure(2, TimeUnit.MILLISECONDS);

            gainOK = gainControl.setGain(100);
        }
        return exposureOK && gainOK;
    }





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idlePower = 800;
        }else if(gamepad1.dpad_right){
            idlePower = 0;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            flywheelTarget = 0;
            stopperPos = 0.9;
            intakePower = 1;
        } else if(gamepad1.a){
            intakePower = -0.5;
        }
    }





    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 24) { // SIDE 24/20
                camRange = detection.ftcPose.range + camOffsetX;
                bearing = detection.ftcPose.bearing; // SIDE +/-
                elevation = detection.ftcPose.elevation;

                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180/976;   // in degrees
                bearing = Math.toRadians(bearing);
                elevation = Math.toRadians(elevation);
                if(hasEst){
                    goal.update(0.08, xPos, yPos, bearing, elevation, camRange);
                }else{
                    goal.update(1, xPos, yPos, bearing, elevation, camRange);
                    hasEst = true;
                }

                break;
            }
        }

        //required turret angle
        double turretTarget = goal.findBearing(xPos, yPos)
                - startingAngle
                - Math.toDegrees(heading)
                + Math.toDegrees(Math.atan(hOffset/range)); // SIDE +/-
        if (turretTarget > highLimit * (90.0/495.0) + 30.0) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < lowLimit * (90.0/495.0) - 30.0) {
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
                goal.setX(xEst);
                goal.setY(yEst);
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
        //setting flap position
        //flapPos = Math.pow(range * 0.00158, 0.1) - 0.159;
        if (range<45) {
            flapPos = 0;
            feedPower = 1;
            hOffset = -1.5; // SIDE -1.5/-3.5
        }else if (range < 95){
            flapPos = -1036323.63 * Math.pow(range, -3.67) + 0.252;
            feedPower = 1;
            hOffset = -1.5; // SIDE -1.5/-3.5
        }else{
            flapPos = 0.24;
            feedPower = 0.8;
            hOffset = 2.5; // SIDE 2.5/-0.5
        }
        flap.setPosition(flapPos);


        if (gamepad1.x) {

            //setting target velocity
            flywheelTarget = (0.00673 * range * range) + (5.54 * range) +  (1095);  //10.27 * range + 1300;2.937 * range + 716.11;



            if (range< 45){
                flywheelTarget-=90;
            }
            else if (range < 100){
                flywheelTarget-=46.7;
            }


            if(Math.abs(flywheelVelocity1) >= Math.abs(flywheelTarget)){
                stopperPos = 0.973; // open
                intakePower = feedPower;
            }
        } else {
            stopperPos = 0.9; // closed
            flywheelTarget = idlePower;
        }
    }


    public void brake(){
        if(gamepad1.x || (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8)){
            if(!auto){
                PathChain hold = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(xPos + 0.00000001, yPos, heading)))
                        .setConstantHeadingInterpolation(heading)
                        .build();
                follower.followPath(hold,0.65, true);
                auto = true;
            }
        }else{
            if(auto) {
                follower.breakFollowing();
                auto = false;
            }
        }
    }

    public void gate(){
        if(gamepad1.left_bumper){
            if(!auto){
                double moveX = -3;
                double moveY = -15;
                double sin = Math.sin(heading);
                double cos = Math.cos(heading);
                PathChain gate = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), new Pose(
                                xPos + moveX * cos - moveY * sin,
                                yPos + moveX * sin + moveY * cos,
                                heading + Math.PI/6)))
                        .setLinearHeadingInterpolation(heading, heading + Math.PI/6)
                        .build();
                intakePower = 1;
                follower.followPath(gate, false);
                auto = true;
            }
        } else{
            if(auto){
                follower.breakFollowing();
                auto = false;
            }
        }
    }

    public void botTelemetry(){
        telemetry.addData("gainSet", gainSet);
        telemetry.addData("goal est", goal);
        telemetry.addData("turret pos", turret.getCurrentPosition());
        telemetry.addData("Cam Status", visionPortal.getCameraState());
        telemetry.addData("range", range);
        telemetry.addData("FWV1", flywheelVelocity1);
        telemetry.addData("FWV2", flywheelVelocity2);
        telemetry.addData("targetVel", flywheelTarget);
        telemetry.addData("turretpos", turretPos);
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.update();


    }


}
