package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DecodeDriveTrain {

    // Instantiate the drivetrain motor variables
    private DcMotorEx frontLeft; //Front left motor of drivetrain
    private DcMotorEx frontRight; //Front right motor of drivetrain
    private DcMotorEx backLeft; //Back left motor of drivetrain
    private DcMotorEx backRight; //Back right motor of drivetrain
    GoBildaPinpointDriver pinpoint;
    private double dampSpeedRatio = 1;
    private double dampTurnRatio  = -0.6;
    private Pose2D pose2D;
    private double headingOffset = 0;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private boolean showTelemetry;
    private boolean fieldCentric;



    public DecodeDriveTrain(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry, boolean showTelemetry, boolean fieldCentric){
        // Motor Mapping
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.showTelemetry = showTelemetry;
        this.fieldCentric = fieldCentric;


        // Set motor direction based on which side of the robot the motors are on
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
//
//        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        configurePinpoint();
    }
    public void configurePinpoint(){
        /*
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-2.36, -0.94, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void drive(double heading){ //Code to be run in Teleop Mode void Loop at top level
        heading = -heading;

        double powerFl = 0;
        double powerFr = 0;
        double powerBl = 0;
        double powerBr = 0;

        double y = Range.clip(-gamepad.left_stick_y, -1, 1);
        //left stick x value
        double x = Range.clip(-gamepad.left_stick_x, -1, 1);
        //right stick x value
        double rx = Range.clip(-gamepad.right_stick_x, -1, 1);
        if(gamepad.right_bumper){
            dampSpeedRatio = 1 - 0.6;
            dampTurnRatio = -0.6 + 0.3;
        }else{
            dampSpeedRatio = 1;
            dampTurnRatio = -0.6;
        }
        if(gamepad.dpad_up){
            headingOffset = heading;
        }
        if (fieldCentric){
            heading -= headingOffset;
            double max;
            double axial   = y * Math.cos(heading) - x * Math.sin(heading);
            double lateral = 1.2 * y * Math.sin(heading) + x * Math.cos(heading);

            double turn     =  0.8 * -gamepad.right_stick_x;

            powerFl = dampSpeedRatio*(axial - lateral) + turn*dampTurnRatio;
            powerFr = dampSpeedRatio*(axial + lateral) - turn*dampTurnRatio;
            powerBl = dampSpeedRatio*(axial + lateral) + turn*dampTurnRatio;
            powerBr = dampSpeedRatio*(axial - lateral) - turn*dampTurnRatio;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(powerFl), Math.abs(powerFr));
            max = Math.max(max, Math.abs(powerBl));
            max = Math.max(max, Math.abs(powerBr));

            if (max > 1.0) {
                powerFl  /= max;
                powerFr /= max;
                powerBl   /= max;
                powerBr  /= max;
            }
            //telemetry.addData("heading",Math.toDegrees(heading));
            frontLeft.setPower(powerFl);
            frontRight.setPower(powerFr);
            backLeft.setPower(powerBl);
            backRight.setPower(powerBr);
        }
        else{

            powerFl = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            powerFr = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
            powerBl = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            powerBr = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxFront = Math.max(powerFl, powerFr);
            double maxBack = Math.max(powerBl, powerBr);
            double maxPower = Math.max(maxFront, maxBack);

            if (maxPower > 1.0) {
                powerFl /= maxPower;
                powerFr /= maxPower;
                powerBl /= maxPower;
                powerBr /= maxPower;
            }
            //finally moving the motors
            frontLeft.setPower(powerFl);
            backLeft.setPower(powerBl);
            frontRight.setPower(powerFr);
            backRight.setPower(powerBr);
        }
        if(showTelemetry) {
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEG)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.addData("FL Power", powerFl);
            telemetry.addData("BL Power", powerBl);
            telemetry.addData("FR Power", powerFr);
            telemetry.addData("BR Power", powerBr);
        }

    }

}
