package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Swerve TeleOp", group = "drive")
public class SwerveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrivetrain swerve = new SwerveDrivetrain(hardwareMap);
        boolean fieldCentric = true;




        waitForStart();

        swerve.resetHeading();
        swerve.startServos();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                swerve.resetHeading();
            }
            swerve.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, fieldCentric);
            swerve.telemetry(telemetry);
            swerve.update();
            telemetry.update();
        }

        swerve.stop();
    }
}
