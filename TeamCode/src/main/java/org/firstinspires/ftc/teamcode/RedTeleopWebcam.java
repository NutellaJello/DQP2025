package org.firstinspires.ftc.teamcode;

import android.util.Size;

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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    boolean fieldCentric = true;
    double intakePower = 0;


    private double heading = 0;



    @Override
    public void runOpMode() {
        drivetrain = new DecodeDriveTrain(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {


            drivetrain.Teleop(gamepad1, heading, telemetry, fieldCentric);



            setIntakePower();


            intake.setPower(intakePower);



        }


    }

    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            intakePower = -1;
        } else if(gamepad1.a){
            intakePower = 0.67;
        }
        else {
            intakePower=0;
        }
    }
}

