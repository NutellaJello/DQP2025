package org.firstinspires.ftc.teamcode.Autos; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DecodeDriveTrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Red Side Close", group = "Autos")
public class RedSideClose extends OpMode {
    private DecodeDriveTrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flyWheel;
    private Servo pusher;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int fireState = 0;
    private Timer fireTimer = new Timer();
    private double flyWheelPower = 0;
    private double pusherPos = 1.0;
    private int shotCount = 0;
    private enum PathState{
        CYCLE1,
        OUTTAKE1,
        INTAKE1,
        END
    }

    PathState pathState;

    private final Pose start = new Pose(130,130,Math.toRadians(45));
    private final Pose outtakePre = new Pose(95,90,Math.toRadians(45));
    private final Pose intake1 = new Pose(135,90,Math.toRadians(0));

    private PathChain Cycle1;
    private PathChain Intake1;

    public void buildPaths(){
        Cycle1 = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setTangentHeadingInterpolation()
                .setVelocityConstraint(1)
                .build();
    }



    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init(){
        pathState = PathState.CYCLE1;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE

        flyWheel = hardwareMap.get(DcMotorEx.class, "FW");
        flyWheel.setDirection(DcMotorEx.Direction.FORWARD); // Change this to either FORWARD or REVERSE
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.REVERSE);


        buildPaths();
        follower.setPose(start);
    }

    public void start(){
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }
    public void statePathUpdate(){
        switch(pathState){
            case CYCLE1:
                follower.followPath(Cycle1, true);
                pathState = PathState.OUTTAKE1;
                break;
            case OUTTAKE1:
                if(!follower.isBusy()){
                    loopOuttake(62);
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                }
                if (shotCount >= 3) {
                    flyWheelPower = 0;
                    pusherPos = 1;
                    fireState = 0;
                    shotCount = 0;
                    pathState = PathState.INTAKE1;
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                }
                break;
            case INTAKE1:
                intake.setPower(1);
                follower.followPath(Intake1, true);
                pathState = PathState.END;
                break;
            case END:
                if(!follower.isBusy()){
                    intake.setPower(0);
                }
                break;
        }
    }
    @Override
    public void loop(){
        follower.update();
        statePathUpdate();
    }
    private void loopOuttake(double range) {

        if (fireState == 0) {
            flyWheelPower = 11.5 * range + 1250;
            fireState = 1;
        }

        else if (fireState == 1) {
            if (flyWheel.getVelocity() >= flyWheelPower || flyWheel.getVelocity() >= 2580) {
                pusherPos = 0.67;
                fireTimer.resetTimer();
                fireState = 2;
            }
        }

        else if (fireState == 2) {
            if (fireTimer.getElapsedTime() > 300) {
                pusherPos = 1;
                fireTimer.resetTimer();
                fireState = 3;
            }
        }

        else if (fireState == 3) {
            intake.setPower(0.9);
            if (fireTimer.getElapsedTime() > 500) {
                shotCount++;
                fireState = 1;
            }
        }
    }
}
