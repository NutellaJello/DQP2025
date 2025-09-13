package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSlide;

@Config
@TeleOp(name = "DecodeTeleop", group = "TeleOp")

public class DecodeTeleop extends LinearOpMode{
    private DecodeDriveTrain drivetrain;
//    private OuttakeSlide outtakeSlide;
    /**
     * outtake slide
     */
    private DcMotorEx intake;
    boolean fieldCentric = true;



    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            // toggle for field centric
            if(gamepad1.dpad_left){
                fieldCentric = !fieldCentric;
            }
            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);



            // delete the other once we know which direction intake is
            if(gamepad1.left_trigger > 0){
                intake.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger > 0){
                intake.setPower(-gamepad1.left_trigger);
            }



        }


    }

}