package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Red DecodeTeleop webcam", group = "TeleOp")

public class BlueTeleopWebcam extends LinearOpMode{
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel;
    private Servo pusher;


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection tagHelper;
    boolean fieldCentric = false;

    boolean useWebcam = true;
    boolean autoAdjust = true;
    double intakePower = 0;
    int flyWheelMode = 0;
    double flyWheelPower = 0;
    double pusherPos = 0.3;
    double turretPower;
    double turretPos;

    double range;
    double bearing;
    double elevation;

    double turretKp = 0.005; //0.01
    double turretKi = 0.0;
    double turretKd = 0.00025; //0.0005

    // PID state
    double turretIntegral = 0;
    double turretLastError = 0;

    ElapsedTime turretPidTimer = new ElapsedTime();
    ElapsedTime spinUpDelay = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();
    boolean flywheelRunning = false;




    @Override
    public void runOpMode() {


        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap);

        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE

        flyWheel=hardwareMap.get(DcMotorEx.class, "FW");
        flyWheel.setDirection(DcMotorEx.Direction.REVERSE); // Change this to either FORWARD or REVERSE
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.REVERSE);




        initWebcam();

        waitForStart();
        turretPidTimer.reset();
        spinUpDelay.reset();
        releaseTimer.reset();

        while (opModeIsActive()) {
            // start camera
            telemetryWebcam();
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();


            // toggle for field centric

            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);

            turretPos = turret.getCurrentPosition();


            if(gamepad1.left_trigger > 0){
                intakePower = Range.clip(gamepad1.left_trigger, 0, 1);
                intake.setPower(intakePower);
            }
            else{
                intake.setPower(0);
            }

//            if(gamepad1.a) {
//                flyWheelMode = 1; //operating power
//            }else if(gamepad1.y){
//                flyWheelMode =2; //max power
//            }else if(gamepad1.b){
//                flyWheelMode =0;//off
//            }
//
//            if(flyWheelMode==1){
//                flyWheelPower = 2000;
//            }else if(flyWheelMode==2){
//                flyWheelPower = 3000;
//            }else{
//                flyWheelPower = 0;
//            }

//            flyWheel.setVelocity(flyWheelPower);

            // PID aiming

            if (autoAdjust){
                double dt = turretPidTimer.seconds();
                turretPidTimer.reset();

                double pidTurretPower = 0;
                boolean hasTarget = false;
                double error = 0;

                // Find our tag and compute error (target bearing = 0)
                for (AprilTagDetection detection : detectedTags) {
                    if (detection.metadata != null && detection.id == 20) { // your desired tag ID
                        range = detection.ftcPose.range;
                        bearing = detection.ftcPose.bearing;   // in degrees
                        elevation = detection.ftcPose.elevation;
                        error = bearing;                       // error = current bearing - desired (0)
                        hasTarget = true;
                        break; // we found our tag, no need to keep looping
                    }
                }

                if (hasTarget && dt > 0) {
                    // PID terms
                    double proportional = error;
                    turretIntegral += error * dt;

                    // Prevent integral windup (optional, tune limits)
                    turretIntegral = Range.clip(turretIntegral, -200, 200);

                    double derivative = (error - turretLastError) / dt;

                    pidTurretPower = turretKp * proportional
                            + turretKi * turretIntegral
                            + turretKd * derivative;

                    turretLastError = error;
                } else {
                    // No target: stop PID contributions & reset integral & use manual control
//                    pidTurretPower = 0;
                    turretIntegral = 0;
                    turretLastError = 0;
                    if(gamepad1.left_bumper){
                        pidTurretPower = 0.3;
                    }
                    else if(gamepad1.right_bumper){
                        pidTurretPower = -0.3;
                    }else{
                        pidTurretPower = 0;
                    }
                }


                // limit turret pos
                if ((pidTurretPower > 0 && turretPos < 525) || (pidTurretPower < 0 && turretPos > -525)) { //left, right limits
                    turret.setPower(pidTurretPower);
                } else {
                    turret.setPower(0);   // stop at limits
                }
            }


            if (gamepad1.x) {
                // Reset the release timer because X *is* held
                releaseTimer.reset();

                // CASE 1: Flywheel was OFF → start it, start timing the spin-up
                if (!flywheelRunning) {
                    flywheelRunning = true;
                    flyWheelPower = 0.000870231 * Math.pow(range, 3) + 1904;
                    spinUpDelay.reset();   // start spin-up timer
                }

                // CASE 2: Flywheel already running → check if delay has passed
                double delay = 0.6 + flyWheelPower * 0.0002;

                if (spinUpDelay.seconds() >= delay) {
                    pusherPos = 0.9;   // raise pusher
                }

            } else {
                // X NOT pressed

                pusherPos = 0.3;  // lower pusher immediately

                // Start or continue the "X not pressed" timer
                if (releaseTimer.seconds() > 3) {
                    // X has been unpressed for > 3 seconds → stop flywheel
                    flywheelRunning = false;
                    flyWheelPower = 0;
                }
            }
            pusher.setPosition(pusherPos);
            flyWheel.setVelocity(flyWheelPower);

            telemetry.addData("target turret power", flyWheelPower);
            telemetry.addData("stop flywheel delay", releaseTimer.seconds());
            telemetry.addData("spin up delay", spinUpDelay.seconds());
            telemetry.addData("Turret Position", turretPos);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("auto adjust camera", autoAdjust);
            telemetry.addData("pusher position", pusher.getPosition());
            telemetry.addData("flywheel power", flyWheelPower);
            telemetry.update();




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


        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }
    private void telemetryWebcam() {

        List<AprilTagDetection> detectedTags = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detectedTags.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

}