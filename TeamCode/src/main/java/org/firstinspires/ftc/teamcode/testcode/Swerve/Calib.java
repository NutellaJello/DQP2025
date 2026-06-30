package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.testcode.Swerve.Constants;
import org.firstinspires.ftc.teamcode.testcode.Swerve.AbsEncoder;

@TeleOp(name = "Swerve Calibration", group = "calibration")
public class Calib extends LinearOpMode {

    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    private static final String[] MOTOR_NAMES = {
            Constants.FLM,
            Constants.FRM,
            Constants.BLM,
            Constants.BRM
    };
    private static final String[] SERVO_NAMES = {
            Constants.FLS,
            Constants.FRS,
            Constants.BLS,
            Constants.BRS
    };

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx[] motors = new DcMotorEx[4];
        Servo[] servos = new Servo[4];

        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            servos[i] = hardwareMap.get(Servo.class, SERVO_NAMES[i]);
        }

        int selected = 0;

        telemetry.addLine("=== SWERVE CALIBRATION ===");
        telemetry.addLine("Press START, then use R-trigger to find servo zeros.");
        telemetry.update();
        waitForStart();
        double[] servopos = {0.0,0.0,0.0,0.0};
        double mult = 0.01;
        for (Servo s : servos) s.setPosition(0);
        boolean rb = false;
        boolean lb = false;
        boolean x = false;
        boolean b = false;
        boolean y = false;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up)    selected = 0;
            if (gamepad1.dpad_right) selected = 1;
            if (gamepad1.dpad_left)  selected = 2;
            if (gamepad1.dpad_down)  selected = 3;

            if(gamepad1.right_bumper && !rb){
                servopos[selected]+=mult;
                rb = true;
            }else if (!gamepad1.right_bumper){
                rb = false;
            }
            if(gamepad1.left_bumper && !lb){
                servopos[selected]-=mult;
                lb = true;
            }else if (!gamepad1.left_bumper){
                lb = false;
            }
            if(gamepad1.x && !x){
                mult *= 10;
                x = true;
            }else if (!gamepad1.x){
                x = false;
            }
            if(gamepad1.b && !b){
                mult /= 10;
                b = true;
            }else if (!gamepad1.b){
                b = false;
            }
            if(gamepad1.y && !y){
                servopos[selected] = flipHeading(servopos[selected]);
                y = true;
            }else if (!gamepad1.y){
                y = false;
            }
            if (gamepad1.a) {
                servos[0].setPosition(Constants.FLoffset);
                servos[1].setPosition(Constants.FRoffset);
                servos[2].setPosition(Constants.BLoffset);
                servos[3].setPosition(Constants.BRoffset);

                servopos[0]= Constants.FLoffset;
                servopos[1]= Constants.FRoffset;
                servopos[2]= Constants.BLoffset;
                servopos[3]= Constants.BLoffset;

            } else {

                servos[selected].setPosition(servopos[selected]);
                //servos[selected].setPosition(gamepad1.right_trigger);
            }

            telemetry.addData("Selected", MODULE_NAMES[selected]);
            telemetry.addLine();
            telemetry.addLine("--- SERVO ZEROS (copy to SwerveConstants) ---");

            for (int i = 0; i < 4; i++) {
                String marker = (i == selected) ? "  <<" : "";
                telemetry.addData(MODULE_NAMES[i], "%.4f%s",
                        servos[i].getPosition(), marker);
            }
            telemetry.addData("servopos = ", servopos);
            telemetry.addData("mult = ", mult);
            telemetry.addLine();
            telemetry.addLine("D-pad=select | RB=motor | R-trigger=servo | A=reset");
            telemetry.update();
        }

        for (DcMotorEx m : motors) m.setPower(0);
    }
    private double flipHeading(double heading) {
        double plus  = heading + 0.5;
        double minus = heading - 0.5;


        if (plus <= 1) {
            return plus;
        } else {
            return minus;
        }


    }
    public static double normAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle > Math.PI)  angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    public static double normPosAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }
}