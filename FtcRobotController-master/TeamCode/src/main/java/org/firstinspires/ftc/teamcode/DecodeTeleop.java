package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;


@TeleOp(name = "DecodeTeleop", group = "TeleOp")
@Disabled
public class DecodeTeleop extends LinearOpMode{
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx outtake;
    boolean fieldCentric = true;
    double intakePower = 0;
    double outtakePower = 0;



    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new DecodeDriveTrain(hardwareMap);

        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE); // Change this to either FORWARD or REVERSE

        outtake=hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE


        waitForStart();
        while (opModeIsActive()) {
            // toggle for field centric
            if(gamepad1.dpad_left){
                fieldCentric = !fieldCentric;
            }
            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);


            if(gamepad2.left_trigger > 0){
                intakePower = Range.clip(gamepad2.left_trigger, 0, 0.67);
                intake.setPower(intakePower);
            }
            else{
                intake.setPower(0);
            }

            if(gamepad2.right_trigger > 0){
                outtakePower = Range.clip(gamepad2.right_trigger, 0, 0.9);
                outtake.setPower(outtakePower);
            }
            else if(gamepad2.right_bumper){
                outtake.setPower(0.56);
            }else{
                outtake.setPower(0);
            }

            telemetry.addData("outtakePower", outtakePower);
            telemetry.update();




        }


    }

}
