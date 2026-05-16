package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;

import java.util.List;

@TeleOp(name = "Red Teleop Limelight", group = "TeleOp")
public class limelightRed extends LinearOpMode {

    private DecodeDriveTrain drivetrain;

    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;

    private Servo stopper;
    private Servo flap;

    private Limelight3A limelight;
    private final int APRILTAG_PIPELINE_INDEX = 0;

    boolean fieldCentric = true;
    private ElapsedTime opModeTimer = new ElapsedTime();

    double hOffset = 0.25;
    double feedPower = 1;

    double intakePower = 0;
    double FW1Target = 0;
    double flapPos = 0.2;
    double stopperPos = 0.9;

    double turretPos;
    double FWV1;
    double idlePower = 0;

    // Tag-relative pose from Limelight (meters)
    private double tagRelXM = 0, tagRelYM = 0, tagRelZM = 0;
    private double camRangeM = 0;

    GoalPos goalPos = new GoalPos(30, 50);

    private Follower follower;
    private boolean holding = false;
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double xEst;
    private double yEst;

    // Robot pose (field units are whatever your Pedro config uses, likely inches)
    private double robotXIn = 0, robotYIn = 0, robotHeadingRad = 0;

    private double range;

    private final double camOffsetX = 2;      // inches forward of center
    private final double startingAngle = 0;   // degrees
    private final double lowLimit = -990;
    private final double highLimit = 840;

    double p = 350;
    double d = 0;
    double i = 0;
    double f = 13.5;

    PIDFCoefficients fwPID = new PIDFCoefficients(p, i, d, f);

    @Override
    public void runOpMode() {

        initLimelight();

        drivetrain = new DecodeDriveTrain(hardwareMap);

        follower = Constants.createTeleopFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

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

        waitForStart();
        opModeTimer.reset();

        while (opModeIsActive()) {

            // Update Pedro pose first (robot pose)
            follower.update();
            robotXIn = follower.getPose().getX();
            robotYIn = follower.getPose().getY();
            robotHeadingRad = follower.getPose().getHeading();

            // Read Limelight safely
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = null;

            if (result != null && result.isValid()) {
                fiducials = result.getFiducialResults();
            }

            // Reset tag info each frame
            camRangeM = 0;
            tagRelXM = 0;
            tagRelYM = 0;
            tagRelZM = 0;

            // Optional: show what Limelight sees and capture pose for tag 24
            if (result != null && result.isValid() && fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fid : fiducials) {

                    Pose3D robotPoseTargetSpace = fid.getRobotPoseTargetSpace();
                    if (robotPoseTargetSpace != null) {
                        Position pos = robotPoseTargetSpace.getPosition();

                        // Store the pose only for the tag you care about
                        if (fid.getFiducialId() == 24) {
                            tagRelXM = pos.x;
                            tagRelYM = pos.y;
                            tagRelZM = pos.z;
                            camRangeM = Math.sqrt(tagRelXM * tagRelXM + tagRelYM * tagRelYM + tagRelZM * tagRelZM);
                        }

                        telemetry.addData("Fiducial ID", fid.getFiducialId());
                    }
                }

                telemetry.addData("tx", result.getTx());
                telemetry.addData("Data age (ms)", result.getStaleness());
            } else {
                telemetry.addData("Limelight", "no valid fiducials");
                if (result != null) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("ta", result.getTa());
                    telemetry.addData("Data age (ms)", result.getStaleness());
                }
            }

            // Movement
            if (!holding) {
                drivetrain.Teleop(gamepad1, robotHeadingRad, telemetry, fieldCentric);
            }

            turretPos = turret.getCurrentPosition();

            FWV1 = flyWheel1.getVelocity();
            setIdlePower();

            if (!gamepad1.x) {
                setIntakePower();
            }

            aiming(result, fiducials);

            if (!(gamepad1.left_trigger > 0.1 || gamepad1.a)) {
                firing();
            }

            brake();

            intake.setPower(intakePower);

            botTelemetry();

            telemetry.update();
        }
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);
        limelight.start();

        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
    }

    public void setIdlePower() {
        if (gamepad1.dpad_left) {
            idlePower = 1800;
        } else if (gamepad1.dpad_right) {
            idlePower = 0;
        }
    }

    public void setIntakePower() {
        if (gamepad1.left_trigger > 0.1) {
            intakePower = 1;
        } else if (gamepad1.a) {
            intakePower = -0.5;
        } else {
            intakePower = 0;
        }
    }

    public void aiming(LLResult result, List<LLResultTypes.FiducialResult> fiducials) {

        boolean haveVision = (result != null && result.isValid() && fiducials != null && !fiducials.isEmpty());

        if (haveVision) {
            LLResultTypes.FiducialResult chosen = null;
            for (LLResultTypes.FiducialResult fid : fiducials) {
                if (fid.getFiducialId() == 24) {
                    chosen = fid;
                    break;
                }
            }

            if (chosen != null && camRangeM > 0) {
                double llBearingDeg = result.getTx();

                double camRangeIn = camRangeM * 39.3700787;
                camRangeIn += camOffsetX;

                double bearingDeg = llBearingDeg + Math.toDegrees(Math.atan(hOffset / camRangeIn));

                bearingDeg += startingAngle + Math.toDegrees(robotHeadingRad) + turretPos * 180.0 / 976.0;

                double bearingRad = Math.toRadians(bearingDeg);

                goalPos.update(robotXIn, robotYIn, bearingRad, camRangeIn);

                xEst = robotXIn + camRangeIn * Math.cos(bearingRad);
                yEst = robotYIn + camRangeIn * Math.sin(bearingRad);

                if (!hasEst) {
                    goalPos.setX(xEst);
                    goalPos.setY(yEst);
                }
                hasEst = true;
            }
        }

        double turretTargetDeg = goalPos.findAngle(robotXIn, robotYIn)
                - startingAngle
                - Math.toDegrees(robotHeadingRad);

        if (turretTargetDeg > 360 + 30) {
            turretTargetDeg -= 360;
        } else if (turretTargetDeg < 0 - 30) {
            turretTargetDeg += 360;
        }

        double turretTargetTicks = 976.0 / 180.0 * turretTargetDeg;
        turretTargetTicks = Range.clip(turretTargetTicks, lowLimit, highLimit);

        if (gamepad2.a) {
            double turretPower;

            if (!a2Press) {
                turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                a2Press = true;
            }

            if (gamepad2.left_bumper) {
                turretPower = 0.4;
            } else if (gamepad2.right_bumper) {
                turretPower = -0.4;
            } else {
                turretPower = 0;
            }

            if (gamepad2.y) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            turret.setPower(turretPower);

            if (hasEst) {
                goalPos.setX(xEst);
                goalPos.setY(yEst);
            }

        } else {
            if (a2Press) {
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                a2Press = false;
            }
            turret.setTargetPosition((int) turretTargetTicks);
        }
    }

    public void firing() {
        range = goalPos.findRange(robotXIn, robotYIn);

        if (range < 45) {
            flapPos = 0;
            feedPower = 0.9;
            hOffset = -1.5;
        } else if (range < 95) {
            flapPos = 0.195;
            feedPower = 0.75;
            hOffset = -1.5;
        } else {
            flapPos = 0.24;
            feedPower = 0.6;
            hOffset = 2.5;
        }

        flap.setPosition(flapPos);

        if (gamepad1.x) {
            FW1Target = (0.00673 * range * range) + (5.54 * range) + (1162);

            if (range < 45) {
                FW1Target -= 100;
            }

            if (FWV1 >= FW1Target) {
                stopperPos = 0.973;
                intakePower = feedPower;
            }
        } else {
            intakePower = 0;
            stopperPos = 0.9;
            FW1Target = idlePower;
        }

        flyWheel1.setVelocity(FW1Target);
        stopper.setPosition(stopperPos);
    }

    public void brake() {
        if (gamepad1.x || (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8)) {
            if (!holding) {
                PathChain hold = follower.pathBuilder()
                        .addPath(new BezierLine(
                                follower.getPose(),
                                new Pose(robotXIn + 0.00000001, robotYIn, robotHeadingRad)
                        ))
                        .setConstantHeadingInterpolation(robotHeadingRad)
                        .build();

                follower.followPath(hold, 0.65, true);
                holding = true;
            }
        } else {
            if (holding) {
                follower.breakFollowing();
                holding = false;
            }
        }
    }

    public void botTelemetry() {
        telemetry.addData("flap", flapPos);
        telemetry.addData("targetVel", FW1Target);
        telemetry.addData("currVel", flyWheel1.getVelocity());
        telemetry.addData("range", range);
        telemetry.addData("turretpos", turretPos);
        telemetry.addData("targetPos", turret.getTargetPosition());

        telemetry.addData("robotX robotY", "%.2f %.2f", robotXIn, robotYIn);
        telemetry.addData("heading(deg)", "%.1f", Math.toDegrees(robotHeadingRad));

        if (camRangeM > 0) {
            telemetry.addData("tagRange(in)", "%.1f", camRangeM * 39.3700787);
        }
    }
}