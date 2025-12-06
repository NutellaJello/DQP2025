package org.firstinspires.ftc.teamcode;

import android.util.Size;

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
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
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
    boolean fieldCentric = false;

    boolean useWebcam = true;
    boolean autoAdjust = true;
    double intakePower = 0;
    int flyWheelMode = 1;
    double flyWheelPower = 0;
    double pusherPos = 0.3;
    double turretPos;
    double flyWheelVel;
    double idlePower = 0;

    double range;
    double lastRange;
    double bearing;
    double elevation;

    double turretKp = 0.022;
    double turretKi = 0.0004; //0.0005
    double turretKd = 0.0008;  //0.0001

    // PID state
    double turretIntegral = 0;
    double turretLastError = 0;
    int fireState = 0;      // 0 = idle, 1 = spinup, 2 = pushUp, 3 = pushDown
    ElapsedTime fireTimer = new ElapsedTime();
    ElapsedTime turretPidTimer = new ElapsedTime();
    ElapsedTime spinUpDelay = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();
    ElapsedTime targetingTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        if (visionPortal != null) {
            visionPortal.close();
            sleep(250);
        }
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE

        flyWheel = hardwareMap.get(DcMotorEx.class, "FW");
        flyWheel.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            //telemetryWebcam();
            List<AprilTagDetection> detectedTags = aprilTag.getDetections();


            // toggle for field centric

            // all the movement controls.
            drivetrain.Teleop(gamepad1, telemetry, fieldCentric);

            turretPos = turret.getCurrentPosition();
            flyWheelVel = flyWheel.getVelocity();
            if(autoAdjust){
                if(gamepad1.dpad_up){
                 idlePower = 1800;
                }else if(gamepad1.dpad_down){
                  idlePower = 0;
                }
            }
            else{
                idlePower = 0;
            }

            if (gamepad1.left_trigger > 0.1 && !gamepad1.x) {
                intakePower = 1;
            } else if(gamepad1.a){
                intakePower = -1;
            }else {
                intakePower = 0;
            }
            intake.setPower(intakePower);
            if (gamepad2.b) { // auto
                flyWheelMode = 0;
            } else if (gamepad2.a) { // 2200
                flyWheelMode = 1;
            } else if (gamepad2.x) {
                flyWheelMode = 2;//off
            } else if (gamepad2.y) {
                flyWheelMode = 3;
            }

            if (flyWheelMode == 0) {
                flyWheelPower = 0;
            } else if (flyWheelMode == 2) {
                flyWheelPower = 2000;
            } else if (flyWheelMode == 3) {
                flyWheelPower = 2500;
            }

            // PID
            if (gamepad2.left_bumper) {
                autoAdjust = true;
            } else if (gamepad2.right_bumper) {
                autoAdjust = false;
            }
            //start pid control
            double dt = turretPidTimer.seconds();
            turretPidTimer.reset();
            dt = Range.clip(dt, 0.001, 0.1);

            double pidTurretPower = 0;
            boolean hasTarget = false;
            double error = 0;

            // Find our tag and compute error (target bearing = 0)
            for (AprilTagDetection detection : detectedTags) {
                if (detection.metadata != null && detection.id == 24) { // your desired tag ID
                    range = (detection.ftcPose.range + lastRange) / 2; //range smoothing
                    bearing = detection.ftcPose.bearing;   // in degrees
                    elevation = detection.ftcPose.elevation;
                    error = bearing + (Math.toDegrees(Math.atan(2 / range)));                       // error = current bearing - desired (0)
                    lastRange = detection.ftcPose.range;
                    targetingTimer.reset();
                    hasTarget = true;
                    break; // we found our tag, no need to keep looping

                }
            }

            if (hasTarget && dt > 0) {
                // PID terms
                turretIntegral += error * dt;

                // Prevent integral windup (optional, tune limits)

                double derivative = -(turretLastError - error) / dt;

                pidTurretPower = turretKp * error
                        + turretKi * turretIntegral
                        + turretKd * derivative;
                turretLastError = error;

            } else {
                // No target: stop PID contributions & reset integral & use manual control
                turretIntegral = 0;
                turretLastError = 0;
                if (gamepad2.left_trigger > 0) {
                    pidTurretPower = Range.clip(gamepad2.left_trigger / 2, 0, 0.5);
                } else if (gamepad2.right_trigger > 0) {
                    pidTurretPower = -Range.clip(gamepad2.right_trigger / 2., 0, 0.5);
                } else if (gamepad1.left_bumper) {
                    pidTurretPower = 0.4;
                } else if (gamepad1.right_bumper) {
                    pidTurretPower = -0.4;
                } else {
                    pidTurretPower = 0;
                }


            } //end pid control
            if (targetingTimer.seconds() < 1) {
                pidTurretPower += 0.38 * Range.clip(Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x, -1, 1);
            }
            pidTurretPower = Range.clip(pidTurretPower, -0.8, 0.8);

            // limit turret pos
            if ((pidTurretPower > 0 && turretPos < 600) || (pidTurretPower < 0 && turretPos > -600)) { //left, right limits
                turret.setPower(pidTurretPower);
            } else {
                turret.setPower(0);   // stop at limits
            }

            if (autoAdjust) {
                if (gamepad1.x) {

                    if (fireState == 0) {
                        // Step 1: Start flywheel
                        flyWheelPower = 11.5 * range + 1220;
                        fireState = 1;   // go to SPINNING_UP
                    } else if (fireState == 1) {
                        // Step 2: Wait until flywheel reaches speed
                        if (flyWheel.getVelocity() >= flyWheelPower || flyWheel.getVelocity() >= 2580) {
                            pusherPos = 0.67;   // fire
                            fireTimer.reset();
                            fireState = 2;     // go to PUSH_UP
                        }
                    } else if (fireState == 2) {
                        // Step 3: pusher up for 0.15s
                        if (fireTimer.seconds() > 0.3) {
                            pusherPos = 1;   // retract
                            fireTimer.reset();
                            fireState = 3;    // go to PUSH_DOWN
                        }
                    } else if (fireState == 3) {
                        intake.setPower(0.9);
                        // Step 4: wait 0.15s then fire again
                        if (fireTimer.seconds() > 0.5) {
                            fireState = 1;   // loop back→ SPINNING_UP → fire again
                        }
                    }
                } else {
                    // X RELEASED → reset firing system
                    pusherPos = 1;
                    flyWheelPower = idlePower;
                    fireState = 0;
                }
            } else {
                if (gamepad1.x) {
                    pusherPos = 0.67;
                } else {
                    pusherPos = 1;
                }
            }

            pusher.setPosition(pusherPos);
            flyWheel.setVelocity(flyWheelPower);

//            telemetry.addData("auto power", autoAdjust);
            telemetry.addData("range", range);
            telemetry.addData("target turret power", 11.5 * range + 1250);
            telemetry.addData("turret error", error);
//            telemetry.addData("turret turn power", pidTurretPower);
//            telemetry.addData("flywheel velocity", flyWheelVel);
//            telemetry.addData("dt",dt);
//            telemetryWebcam();
            //          telemetry.addData("fire timer", fireTimer);
//            telemetry.addData("stop flywheel delay", releaseTimer.seconds());
//            telemetry.addData("spin up delay", spinUpDelay.seconds());
//            telemetry.addData("bearing", bearing);
            //   telemetry.addData("Turret Position", turretPos);
//            telemetry.addData("Field Centric", fieldCentric);
//            telemetry.addData("auto adjust camera", autoAdjust);
//            telemetry.addData("pusher position", pusher.getPosition());
            telemetry.addData("flywheel velocity", flyWheel.getVelocity());
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
}
