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

public class DecodeTeleop extends LinearOpMode{
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx outtake;
    boolean fieldCentric = false;
    double intakePower = 0;
    int outtakeMode = 0;
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

            // all the movement controls.
            drivetrain.Teleop(gamepad1,telemetry, fieldCentric);


            if(gamepad1.left_trigger > 0){
                intakePower = Range.clip(gamepad1.left_trigger, 0, 0.67);
                intake.setPower(intakePower);
            }
            else{
                intake.setPower(0);
            }

            if(gamepad1.a) {
                outtakeMode = 1; //max power
            }else if(gamepad1.y){
                outtakeMode =2; //operating power
            }else if(gamepad1.b){
                outtakeMode =0;//off
            }

            if(outtakeMode==0){
                outtakePower =0;
            }else if(outtakeMode==1){
                outtakePower = 0.56;
            }else if(outtakeMode==2){
                outtakePower = 1;
            }else{
                outtakePower = 0;
            }


            outtake.setPower(outtakePower);
            /*
            if(gamepad1.right_trigger > 0){
                outtakePower = Range.clip(gamepad1.right_trigger, 0, 0.9);
                outtake.setPower(outtakePower);
            }
            else if(gamepad1.right_bumper){
                outtake.setPower(0.56);
            }else{
                outtake.setPower(0);
            }*/


            telemetry.addData("Field Centric", fieldCentric);
            telemetry.update();




        }


    }

}
