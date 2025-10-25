package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "MonolithDetect", group = "Testing")

@Disabled
public class MonolithDetect extends LinearOpMode{

    private Limelight3A limelight;
    IMU imu;
    // Green is represented as 1, Purple is represented as 2
    private final List<Integer> order = new ArrayList<Integer>();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        /*
         * Starts polling for data.
         */
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Running:", "True");
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> tags =
                            result.getFiducialResults();

                    if (tags != null && !tags.isEmpty()) {
                        int id = tags.get(0).getFiducialId(); // take the primary/closest tag

                        // Map tag ID -> order vector
                        if (id == 21) {
                            setOrder(1, 0, 0);
                        } else if (id == 22) {
                            setOrder(0, 1, 0);
                        } else if (id == 23) {
                            setOrder(0, 0, 1);
                        } else {
                            // Unknown tag: clear or keep previous
                            setOrder(0, 0, 0);
                        }

                        telemetry.addData("Tag ID", id);
                        telemetry.addData("Order", order);
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
    private void setOrder(int a, int b, int c) {
        order.clear();
        order.add(a); order.add(b); order.add(c);
    }

}
