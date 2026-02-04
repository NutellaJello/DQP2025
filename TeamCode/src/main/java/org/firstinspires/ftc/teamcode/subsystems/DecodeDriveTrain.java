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
    // Drivetrain motors
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

    private IMU imu;

    // Damping (kept exactly in your style)
    private double dampSpeedRatio = 1;
    private double dampTurnRatio  = -1;

    // Field centric
    private double headingOffset = 0;

    public DecodeDriveTrain(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        // Motor directions (keep yours, adjust if needed for your wiring)
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(params);
        imu.resetYaw();


        // Optional braking
        // FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void stopDrive() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void Teleop(Gamepad gamepad, double heading, Telemetry telemetry, boolean fieldCentric) {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    Teleop(gamepad, heading, telemetry, true, fieldCentric);
    }

    /*
     * Control scheme requested:
     * - Translation (x,y) on ONE joystick (left stick)
     * - Turn on the OTHER joystick (right stick x)
     *
     * Field centric expects heading in radians.
     */
    public void Teleop(Gamepad gamepad,
                       double heading,
                       Telemetry telemetry,
                       boolean showTelemetry,
                       boolean field) {

        // Keep your damping logic exactly
        if (gamepad.right_bumper) {
            dampSpeedRatio = 1 - 0.65;
            dampTurnRatio = -0.6 + 0.3;
        } else {
            dampSpeedRatio = 1;
            dampTurnRatio = -0.6;
        }

        // Keep your heading sign flip behavior
        heading = -heading;

        if (gamepad.dpad_up) {
            headingOffset = heading;
        }

        double PowerFL;
        double PowerFR;
        double PowerBL;
        double PowerBR;

        // Translation on left stick, turn on right stick
        double y  = Range.clip(-gamepad.left_stick_y, -1, 1);     // forward/back
        double x  = Range.clip(-gamepad.left_stick_x, -1, 1);     // strafe
        double rx = Range.clip(-gamepad.right_stick_x, -1, 1);    // turn

        if (field) {
            heading -= headingOffset;

            // Rotate the translation vector into robot frame
            // (rotation by -heading, consistent with heading sign handling above)
            double xr = x * Math.cos(heading) + y * Math.sin(heading);
            double yr = -x * Math.sin(heading) + y * Math.cos(heading);

            double axial   = yr;
            double lateral = 1.2 * xr;

            double turn = 0.8 * rx;

            // Apply your damping multipliers in the same place you were doing it
            PowerFL = dampSpeedRatio * (axial - lateral) + turn * dampTurnRatio;
            PowerFR = dampSpeedRatio * (axial + lateral) - turn * dampTurnRatio;
            PowerBL = dampSpeedRatio * (axial + lateral) + turn * dampTurnRatio;
            PowerBR = dampSpeedRatio * (axial - lateral) - turn * dampTurnRatio;

            // Normalize using absolute values (safer than max of signed values)
            double max = Math.max(
                    Math.max(Math.abs(PowerFL), Math.abs(PowerFR)),
                    Math.max(Math.abs(PowerBL), Math.abs(PowerBR))
            );

            if (max > 1.0) {
                PowerFL /= max;
                PowerFR /= max;
                PowerBL /= max;
                PowerBR /= max;
            }

            FL.setPower(PowerFL);
            FR.setPower(PowerFR);
            BL.setPower(PowerBL);
            BR.setPower(PowerBR);

        } else {
            // Robot centric, same control scheme
            double turn = rx;

            PowerFL = (y - x) * dampSpeedRatio + dampTurnRatio * turn;
            PowerFR = (y + x) * dampSpeedRatio - dampTurnRatio * turn;
            PowerBL = (y + x) * dampSpeedRatio + dampTurnRatio * turn;
            PowerBR = (y - x) * dampSpeedRatio - dampTurnRatio * turn;

            // Normalize using absolute values
            double max = Math.max(
                    Math.max(Math.abs(PowerFL), Math.abs(PowerFR)),
                    Math.max(Math.abs(PowerBL), Math.abs(PowerBR))
            );

            if (max > 1.0) {
                PowerFL /= max;
                PowerFR /= max;
                PowerBL /= max;
                PowerBR /= max;
            }


            FL.setPower(-PowerFL);
            BL.setPower(-PowerBL);
            FR.setPower(-PowerFR);
            BR.setPower(-PowerBR);
        }

        if (showTelemetry) {
            telemetry.addData("field", field);
            telemetry.addData("heading(rad)", heading);
            telemetry.addData("headingOffset(rad)", headingOffset);
            telemetry.addData("sticks y x rx", "%.2f %.2f %.2f", y, x, rx);
            telemetry.addData("dampSpeedRatio", dampSpeedRatio);
            telemetry.addData("dampTurnRatio", dampTurnRatio);
            telemetry.addData("LSY", gamepad.left_stick_y);
            telemetry.addData("y used", -gamepad.left_stick_y);
            telemetry.update();
        }
    }
}