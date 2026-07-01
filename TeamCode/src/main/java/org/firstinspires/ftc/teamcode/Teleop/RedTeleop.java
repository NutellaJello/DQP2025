package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@TeleOp(name = "Red Teleop", group = "TeleOp") // SIDE RED/BLUE

public class RedTeleop extends LinearOpMode { // SIDE
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

    double hOffset = 0;
    double feedPower = 1;

    boolean useWebcam = true;
    double intakePower = 0;
    double FWTarget = 0;
    double flapPos = 0.2;
    double stopperPos = 0.9;
    double turretPos;
    double FWV1;
    double FWV2;
    double FWV;
    boolean idle = false;
    double camRange = 0;
    double bearing = 0;
    double elevation = 0;
    GoalPos goal = new GoalPos(30,50, 15.5); // SIDE 50/-50

    private Follower follower;
    private boolean auto = false;
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double range;
    private double height;
    private double xPos = 0, yPos = 0, heading = 0;
    private double leadK, shootXPos, shootYPos;
    private double headingOffset = 0;
    private final double camOffsetX = 2; //inches (not really inches) forward of center
    private final double camOffsetY = 0; //inches (not really inches) right of center
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = -1506; //495/90
    private final double highLimit =  340 ;
    double p = 400;
    double d = 0;
    double i = 0;
    double f = 13.5;


    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d,  f);

    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap, gamepad1, telemetry, false, fieldCentric);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(0)));

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
        turret.setPositionPIDFCoefficients(15);

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
            leadK = 0.0033 * range + 0.2667;
            shootXPos = follower.getVelocity().getXComponent() * leadK;
            shootYPos = follower.getVelocity().getYComponent() * leadK;
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                headingOffset = heading;
            }
            range = goal.findRange(shootXPos + xPos, shootYPos + yPos);
            turretPos = turret.getCurrentPosition();
            FWV1 = flyWheel1.getVelocity();
            FWV2 = flyWheel2.getVelocity();
            FWV = Math.max(FWV1, FWV2);

            // set initial values
            if(!auto){
                intakePower = 0;
            }


            // configure webcam
            if(!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }
            if(!gainSet && streamStarted && opModeTimer.milliseconds() > camStreamingTime + 500){
                gainSet = cameraControls();
            }


            // all the movement controls.
            if(!auto){
                drivetrain.Teleop(heading);
            }

            //aiming
            if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                List<AprilTagDetection> detectedTags = aprilTag.getDetections();
                aiming(detectedTags);
            }

            // intake controls
            if(!auto){
                setIntakePower();
            }

            boolean gateButton = gamepad1.left_bumper;
            if(gateButton){
                gate();
            }

            // outtake controls
            setIdlePower();
            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){ // disable if manual intake
                firing();
            }

            boolean brakeButton = /*gamepad1.x ||*/ (gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7);
            if(brakeButton){
                brake();
            }

            // stop autonomous pathing
            if((auto || follower.isBusy())&& !(gateButton || brakeButton) ){
                follower.breakFollowing();
                auto = false;
            }

            // apply final values
            intake.setPower(intakePower);
            flyWheel1.setVelocity(FWTarget);
            flyWheel2.setVelocity(FWTarget);
            stopper.setPosition(stopperPos);

            botTelemetry();

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





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idle = true;
        }else if(gamepad1.dpad_right){
            idle = false;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            stopperPos = 0.9;
            intakePower = 1;
        } else if(gamepad1.a){
            intakePower = -0.5;
        }
    }


    public void aiming(List<AprilTagDetection> detectedTags){
        telemetry.addData("xV", follower.getVelocity().getYComponent());
        telemetry.addData("yV", follower.getVelocity().getXComponent());
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == 24) { // SIDE 24/20
                camRange = detection.ftcPose.range + camOffsetX;
                bearing = detection.ftcPose.bearing;
                elevation = detection.ftcPose.elevation;

                bearing += startingAngle + Math.toDegrees(heading) + turretPos * 180.0/976.0;   // in degrees
                bearing = Math.toRadians(bearing);
                elevation = Math.toRadians(elevation);
                if(hasEst){
                    goal.update(0.15, xPos, yPos, bearing, elevation, camRange);
                }else{
                    goal.update(1, xPos, yPos, bearing, elevation, camRange);
                    hasEst = true;
                }

                break;
            }
        }

        //required turret angle
        if(range < 100){
            hOffset = range * 0.0309 - 4.0; //hOffset = range * 0.0309 - 5.367
        } else{
            hOffset = range * 0.0298 - 5.0; //hOffset = range * 0.0298 - 5.317 // SIDE 3.0/4.0
        }
        //hOffset = 0;

        double turretTarget = goal.findAngle(shootXPos + xPos, shootYPos + yPos)
                - startingAngle
                - Math.toDegrees(heading)
                + Math.toDegrees(Math.atan2(hOffset, range)); // SIDE +/-
        if (turretTarget > highLimit * (90.0/495.0) + 30.0) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < lowLimit * (90.0/495.0) - 30.0) {
            turretTarget += 360;
        }
        turretTarget = 976.0 / 180.0 * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));
        if (gamepad2.a){
            goal.update(1, xPos, yPos, bearing, elevation, camRange);
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
        if(range > 53){
            flapPos = range * 0.00080 + 0.1481; //flapPos = range * 0.00086 + 0.1481;
        } else{
            flapPos = range * 0.011 - 0.385;
        }
        flapPos = Range.clip(flapPos, 0, 0.22);
        flap.setPosition(flapPos);

        if(range < 100) {
            FWTarget = 0.903*range * 7.710 + 1000;  //FWTarget = range * 7.710 + 980
            feedPower = 1;
        } else {
            FWTarget = 0.903*range * 7.462 + 1020; //FWTarget = range * 7.462 + 1021
            feedPower = 0.65;
        }
        if (gamepad1.x) {

            //setting target velocity


            if(Math.abs(FWV) >= Math.abs(FWTarget)){
                stopperPos = 0.973; // open
                intakePower = feedPower;
            }
        } else {
            stopperPos = 0.9; // closed
            if(idle) {
                FWTarget *= 0.75;
            } else{
                FWTarget = 0;
            }
        }
    }


    public void brake(){
        if(!auto){
            PathChain hold = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), new Pose(xPos + 0.01, yPos, heading)))
                    .setConstantHeadingInterpolation(heading)
                    .build();
            follower.followPath(hold,1, true);
            auto = true;
        }
    }

    public void gate(){
        if(!auto){
            double moveX = 6; // forward 6in SIDE 6/5
            double moveY = -14; // left/right 3in SIDE -14/+14

            double sin = Math.sin(headingOffset);
            double cos = Math.cos(headingOffset);

            double targetX = xPos + moveX * cos - moveY * sin;
            double targetY = yPos + moveX * sin + moveY * cos;
            double targetH = Math.toRadians(35) + headingOffset; // SIDE +35/-35

            double controlX = xPos - moveY * sin;

            PathChain gate = follower.pathBuilder()
                    .addPath(new BezierCurve(follower.getPose(),
                            new Pose(
                                    controlX,
                                    targetY
                            ),
                            new Pose(
                                    targetX,
                                    targetY,
                                    targetH
                            )
                    ))
                    .setLinearHeadingInterpolation(heading, targetH)
                    .build();
            intakePower = 1;
            follower.followPath(gate, false);
            auto = true;
        }
    }

    public void botTelemetry(){
//        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
//            telemetry.addData(sensor.getDeviceName(), sensor.getVoltage());
//        }
        telemetry.addData("Cam Status", visionPortal.getCameraState());
        telemetry.addData("Cam Setup", gainSet);
        telemetry.addData("Goal Position", goal);
        telemetry.addData("Range", range);
        telemetry.addData("targetVel", FWTarget);
        telemetry.addData("FWV1", FWV1);
        telemetry.addData("FWV2", FWV2);
        telemetry.addData("Flap", flapPos);
        telemetry.addData("turretPos", turretPos);
        telemetry.addData("auto", auto);
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.update();




    }


}