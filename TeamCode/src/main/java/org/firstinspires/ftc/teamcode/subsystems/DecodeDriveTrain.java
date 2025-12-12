package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private DcMotorEx fl; //Front left motor of drivetrain
    private DcMotorEx fr; //Front right motor of drivetrain
    private DcMotorEx bl; //Back left motor of drivetrain
    private DcMotorEx br; //Back right motor of drivetrain
    private IMU imu;
    GoBildaPinpointDriver pinpoint;
    private double dampSpeedRatio = 1;
    private double dampTurnRatio  = -1;
    private Pose2D pose2D;


    public DecodeDriveTrain(HardwareMap hardwareMap){                 // Motor Mapping
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");


        // Set motor direction based on which side of the robot the motors are on
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

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
        pinpoint.setOffsets(9.25, -0.2, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

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
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    public void Teleop(Gamepad gamepad, Telemetry telemetry, boolean fieldCentric) {
        Teleop(gamepad, telemetry, true, fieldCentric);
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
        if(gamepad.right_trigger > 0.1){
            dampSpeedRatio = 1 - 0.8 * gamepad.right_trigger;
            dampTurnRatio = -0.6 + 0.4 * gamepad.right_trigger;
        }else{
            dampSpeedRatio = 1;
            dampTurnRatio = -0.6;
        }
        if (field){
            double max;
//            YawPitchRollAngles robotOrientation;
//            robotOrientation = imu.getRobotYawPitchRollAngles();
            if(gamepad.dpad_right){
                pinpoint.resetPosAndIMU();
            }


            double axial   = y * Math.cos(heading) - x * Math.sin(heading);
            double lateral = 1.1 * y * Math.sin(heading) + x * Math.cos(heading);

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
            fl.setPower(PowerFL);
            fr.setPower(PowerFR);
            bl.setPower(PowerBL);
            br.setPower(PowerBR);
        }
        else{
            if(gamepad.right_trigger > 0.1){
                dampSpeedRatio = 0.4;
                dampTurnRatio = -0.3; // -0.15
            }else{
                dampSpeedRatio = 1;
                dampTurnRatio = -0.6;
            }

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
            fl.setPower(PowerFL);
            bl.setPower(PowerBL);
            fr.setPower(PowerFR);
            br.setPower(PowerBR);
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
