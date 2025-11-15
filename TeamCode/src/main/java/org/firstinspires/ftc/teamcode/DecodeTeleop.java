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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;


@TeleOp(name = "DecodeTeleop", group = "TeleOp")

public class DecodeTeleop extends LinearOpMode{
        private DecodeDriveTrain drivetrain;
        private DcMotorEx intake;
        private DcMotorEx turret;
        private DcMotorEx flyWheel;
        private Servo pusher;

        boolean fieldCentric = false;
        double intakePower = 0;
        int flyWheelMode = 0;
        double flyWheelPower = 0;
        double pusherPosition = 0.3;




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
            turret.setDirection(DcMotorEx.Direction.REVERSE);

            pusher = hardwareMap.get(Servo.class, "pusher");
            pusher.setDirection(Servo.Direction.REVERSE);




            waitForStart();
            while (opModeIsActive()) {
                // toggle for field centric

                // all the movement controls.
                drivetrain.Teleop(gamepad1,telemetry, fieldCentric);


                if(gamepad1.left_trigger > 0){
                    intakePower = Range.clip(gamepad1.left_trigger, 0, 1);
                    intake.setPower(intakePower);
                }
                else{
                    intake.setPower(0);
                }

                if(gamepad1.a) {
                    flyWheelMode = 1; //operating power
                }else if(gamepad1.y){
                    flyWheelMode =2; //max power
                }else if(gamepad1.b){
                    flyWheelMode =0;//off
                }

                if(flyWheelMode==1){
                    flyWheelPower = 0.56;
                }else if(flyWheelMode==2){
                    flyWheelPower = 0.8;
                }else{
                    flyWheelPower = 0;
                }


                flyWheel.setVelocity(2000);

                if(gamepad1.left_bumper){
                    turret.setPower(0.1);
                }
                else if(gamepad1.right_bumper){
                    turret.setPower(-0.1);
                }else{
                    turret.setPower(0);
                }


                if(gamepad1.x){
                    pusherPosition = 0.9; // up
                }
                else{
                    pusherPosition = 0.3; // down
                }
                pusher.setPosition(pusherPosition);

                telemetry.addData("Field Centric", fieldCentric);
                telemetry.addData("pusher position", pusher.getPosition());
                telemetry.update();




            }


        }

}
