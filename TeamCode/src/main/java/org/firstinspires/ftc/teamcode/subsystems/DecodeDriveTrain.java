package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private DcMotorEx FL; //Front left motor of drivetrain
    private DcMotorEx FR; //Front right motor of drivetrain
    private DcMotorEx BL; //Back left motor of drivetrain
    private DcMotorEx BR; //Back right motor of drivetrain
    GoBildaPinpointDriver pinpoint;
    private double dampSpeedRatio = 1;
    private double dampTurnRatio  = -1;
    private Pose2D pose2D;
    private double headingOffset = 0;


    public DecodeDriveTrain(HardwareMap hardwareMap){                 // Motor Mapping
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");


        // Set motor direction based on which side of the robot the motors are on
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.FORWARD);

//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
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

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean fieldCentric) {
        Teleop(gamepad, telemetry, false, fieldCentric);
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean showTelemetry, boolean field ){ //Code to be run in Teleop Mode void Loop at top level
        pinpoint.update();
        pose2D = pinpoint.getPosition();
        double heading   = -pose2D.getHeading(AngleUnit.RADIANS);

        double PowerFL;
        double PowerFR;
        double PowerBL;
        double PowerBR;

        double y = Range.clip(-gamepad.left_stick_y, -1, 1);
        //left stick x value
        double x = Range.clip(-gamepad.left_stick_x, -1, 1);
        //right stick x value
        double rx = Range.clip(-gamepad.right_stick_x, -1, 1);
        if(gamepad.right_bumper){
            dampSpeedRatio = 1 - 0.65;
            dampTurnRatio = -0.6 + 0.3;
        }else{
            dampSpeedRatio = 1;
            dampTurnRatio = -0.6;
        }
        if(gamepad.dpad_up){
            headingOffset = heading;
        }
        if (field){
            heading -= headingOffset;
            double max;
            double axial   = y * Math.cos(heading) - x * Math.sin(heading);
            double lateral = 1.2 * y * Math.sin(heading) + x * Math.cos(heading);

            double turn     =  0.8 * -gamepad.right_stick_x;

            PowerFL = dampSpeedRatio*(axial - lateral) + turn*dampTurnRatio;
            PowerFR = dampSpeedRatio*(axial + lateral) - turn*dampTurnRatio;
            PowerBL = dampSpeedRatio*(axial + lateral) + turn*dampTurnRatio;
            PowerBR = dampSpeedRatio*(axial - lateral) - turn*dampTurnRatio;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(PowerFL), Math.abs(PowerFR));
            max = Math.max(max, Math.abs(PowerBL));
            max = Math.max(max, Math.abs(PowerBR));

            if (max > 1.0) {
                PowerFL  /= max;
                PowerFR /= max;
                PowerBL   /= max;
                PowerBR  /= max;
            }
            //telemetry.addData("heading",Math.toDegrees(heading));
            FL.setPower(PowerFL);
            FR.setPower(PowerFR);
            BL.setPower(PowerBL);
            BR.setPower(PowerBR);
        }
        else{

            PowerFL = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            PowerFR = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
            PowerBL = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            PowerBR = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxFront = Math.max(PowerFL, PowerFR);
            double maxBack = Math.max(PowerBL, PowerBR);
            double maxPower = Math.max(maxFront, maxBack);

            if (maxPower > 1.0) {
                PowerFL /= maxPower;
                PowerFR /= maxPower;
                PowerBL /= maxPower;
                PowerBR /= maxPower;
            }
            //finally moving the motors
            FL.setPower(PowerFL);
            BL.setPower(PowerBL);
            FR.setPower(PowerFR);
            BR.setPower(PowerBR);
        }
        if(showTelemetry) {
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEG)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.addData("FL Power", PowerFL);
            telemetry.addData("BL Power", PowerBL);
            telemetry.addData("FR Power", PowerFR);
            telemetry.addData("BR Power", PowerBR);
        }

    }

}
