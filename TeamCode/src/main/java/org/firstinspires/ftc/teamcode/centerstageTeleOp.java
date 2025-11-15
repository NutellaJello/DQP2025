//ftc package yay!
package org.firstinspires.ftc.teamcode;

//importing needed things
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
//@Disabled
//beginning of class
public class centerstageTeleOp extends LinearOpMode {
    private Limelight3A limelight;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        //motors
        double dampSpeedRatio = 0.58;
        double dampTurnRatio = 0.4;

        double flopSpeed = 0.000003;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft"); //0
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft"); //1
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight"); //3

        Servo flopper = hardwareMap.servo.get("flopper"); //0
        Servo claw = hardwareMap.servo.get("claw"); //1
        Servo airplane = hardwareMap.servo.get("airplane");//expansion 2
//        //expansion
        Servo leftarm = hardwareMap.servo.get("leftarm"); //port 0 lswing
        Servo rightarm = hardwareMap.servo.get("rightarm"); //port 1 rswing
        //imma make a double that updates the servo posi
        rightarm.setDirection(Servo.Direction.REVERSE);
        double sPosiL = 0.8;

        //DcMotor slides = hardwareMap.dcMotor.get("slides"); //0
        // vision detection
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */
        limelight.start();

        boolean dropped = false;

        double floposi = 0.305;

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        //slides.setDirection(DcMotorSimple.Direction.REVERSE);
        /*slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/


        airplane.setPosition (0.7);
//        slides.setTargetPosition(0);
//        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wait for button click
        waitForStart();
        // stop the program if button click
        if (isStopRequested()) return;
        boolean slowMode = false;

        while (opModeIsActive()) {
            telemetry.addData("leftarm", Double.toString(leftarm.getPosition()));
            telemetry.addData("rightarm", Double.toString(rightarm.getPosition()));
            telemetry.addData("claw", Double.toString(claw.getPosition()));
            telemetry.addData("flopper", Double.toString(flopper.getPosition()));
            //telemetry.addData("slides", Double.toString(slides.getCurrentPosition()));
            telemetry.addData("airplane", Double.toString(airplane.getPosition()));
            telemetry.addData("drift", Double.toString(gamepad2.left_stick_y));

            centerRobot();


            ////////////////arm
            leftarm.setPosition(sPosiL);
            rightarm.setPosition(sPosiL);


            double upPosi = 0.98;

            if (sPosiL == upPosi && dropped){
                sPosiL = 0.6;
            }

            if(sPosiL > upPosi){
                sPosiL = upPosi;//setting minimmum so it dont go below the ground
            }

            if (gamepad2.dpad_up){
                sPosiL = upPosi;//up

            }
            if (gamepad2.dpad_down){
                sPosiL = 0.04;//down

            }
            if (gamepad2.dpad_right){
                sPosiL = 0.6;// middle
            }

//            if (gamepad2.dpad_right){
//                sPosiL = 0.12;//tweaker
//            }
//            if (gamepad2.dpad_left){
//                sPosiL = 0.6;//tweaker
//            }
            if(gamepad2.left_bumper){
                sPosiL = 0.08;
            }

            ////////////////claww
            if(gamepad2.right_bumper){
                claw.setPosition( 0.83);//open
                sleep(100);
                dropped = true;
            }else {
                claw.setPosition(0.62);//close
            }



            ///////////////////////flopper
            if (gamepad2.right_trigger > 0 && sPosiL < upPosi -0.05){
                //floposi = 0.72;//dump
                floposi+= 0.0022 ;
            }
            else if (gamepad2.a) {
                floposi = 0.305;//reset position
                dropped = false;
            }
            flopper.setPosition(floposi);
            if (floposi > 0.70){
                floposi = 0.70;
            }


            /// airplane
            if (gamepad1.dpad_left){
                airplane.setPosition (1);
            }
            else {
                airplane.setPosition(0.8);
            }

//            if (gamepad2.left_stick_y < 0 ){
//
//                    slides.setTargetPosition((int) (slides.getCurrentPosition() - (70 * Math.abs(gamepad2.left_stick_y))));
//                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    //slides.setPower(0.5);
//            }
//            if (gamepad2.left_stick_y > 0){
//                    slides.setTargetPosition((int)(slides.getCurrentPosition() + (70 * Math.abs(gamepad2.left_stick_y))));
//                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    //slides.setPower(0.5);
//            }
//
//            slides.setPower(0.7);



            double y = Range.clip(gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            //    double arct = 0;

            double flPower = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            double frPower = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
            double blPower = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            double brPower = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxFront = Math.max(flPower, frPower);
            double maxBack = Math.max(blPower, brPower);
            double maxPower = Math.max(maxFront, maxBack);

            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }
            //finally moving the motors
            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);

            //sprint
//

            if(gamepad1.right_bumper || gamepad1.right_trigger > 0){
                dampSpeedRatio = 0.3;
                dampTurnRatio = 0.2;
            }else if(gamepad1.left_bumper || gamepad1.left_trigger > 0){
                dampSpeedRatio = 1.0;
            }else{
                dampSpeedRatio = 0.6;
                dampTurnRatio = 0.4;
            }

            telemetry.update();

        }
    }
    private void centerRobot(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId(); // take the primary/closest tag

                    // Map tag ID -> order vector
//                    if (id == 20) {
//
//                    } else if (id == 24) {
//
//
//                    } else {
//
//
//                    }

                    telemetry.addData("Tag ID", id);
                } else {
                    telemetry.addData("Tags", "none");
                }


                Pose3D botPose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("botPose", botPose.toString());
            }
        }
        telemetry.update();
    }
}