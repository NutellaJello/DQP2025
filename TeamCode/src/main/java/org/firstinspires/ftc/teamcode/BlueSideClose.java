package org.firstinspires.ftc.teamcode; // make sure this aligns with class location
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

@Autonomous(name = "Blue Side Close", group = "Autos")
public class BlueSideClose extends OpMode {
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
    private double pusherPos = 0.4;
    private int shotCount = 0;
    private boolean moving = false;
    private enum PathState{
        PRELOAD,
        SHOOTPRE,
        INTAKE1,
        OUTTAKE1,
        SHOOT1,
        INTAKE21,
        INTAKE22,
        OUTTAKE2,
        SHOOT2,
        END,
        STOP
    }

    PathState pathState;
    //positions
    private final Pose start = new Pose(144-130,130,Math.toRadians(180-41));
    private final Pose outtakePre = new Pose(144-95,90,Math.toRadians(180-41));
    private final Pose outtake = new Pose(144-100,90, Math.toRadians(180));
    private final Pose intake1 = new Pose(144-133,88,Math.toRadians(180));
    private final Pose intake2p1 = new Pose(144-100,68, Math.toRadians(180));
    private final Pose intake2p2 = new Pose(144-127,68-5,Math.toRadians(180));
    private final Pose end = new Pose(144-130,90,Math.toRadians(180));

    //paths
    private PathChain Preload;
    private PathChain Intake1;
    private PathChain Outtake1;
    private PathChain Intake21;
    private PathChain Intake22;
    private PathChain Outtake2;
    private PathChain End;

    public void buildPaths(){
        Preload = follower.pathBuilder()
                .addPath(new BezierLine(start, outtakePre))
                .setLinearHeadingInterpolation(start.getHeading(), outtakePre.getHeading())
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(outtakePre, intake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Outtake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, outtake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Intake21 = follower.pathBuilder()
                .addPath(new BezierLine(outtake, intake2p1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Intake22 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p1,intake2p2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Outtake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2p2, outtake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        End = follower.pathBuilder()
                .addPath(new BezierLine(outtake,end))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }



    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init(){
        pathState = PathState.PRELOAD;
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
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(0.3);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.REVERSE);


        buildPaths();
        follower.setPose(start);
    }

    public void start(){
        opmodeTimer.resetTimer();
        setPathState(pathState);
        pathTimer.resetTimer();
    }
    public void statePathUpdate(){
        telemetry.addData("step",pathState);
        telemetry.addData("busy",follower.isBusy());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();


        switch(pathState){
            case PRELOAD:
                if(!moving) {
                    turret.setTargetPosition(-60);
                    follower.followPath(Preload, true);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50) {
                    pathState = PathState.SHOOTPRE;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case SHOOTPRE:
                if(!follower.isBusy()){
                    loopOuttake(67);
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                }
                if (shotCount >= 3) {
                    flyWheelPower = 0;
                    pusherPos = 0.4;
                    fireState = 0;
                    shotCount = 0;
                    pathState = PathState.INTAKE1;
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                    pathTimer.resetTimer();
                }
                break;
            case INTAKE1:
                if(!moving){
                    turret.setTargetPosition(-280);
                    intake.setPower(1);
                    follower.followPath(Intake1, 0.3,false);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    intake.setPower(0);
                    pathState = PathState.OUTTAKE1;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case OUTTAKE1:
                if(!moving){
                    follower.followPath(Outtake1, true);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    pathState = PathState.SHOOT1;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case SHOOT1:
                if(!follower.isBusy()){
                    loopOuttake(67);
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                }
                if (shotCount >= 3) {
                    flyWheelPower = 0;
                    pusherPos = 0.4;
                    fireState = 0;
                    shotCount = 0;
                    intake.setPower(0);
                    pathState = PathState.INTAKE21;
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                    pathTimer.resetTimer();
                }
                break;
            case INTAKE21:
                if(!moving){
                    follower.followPath(Intake21, false);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    pathState = PathState.INTAKE22;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case INTAKE22:
                if(!moving){
                    intake.setPower(1);
                    follower.followPath(Intake22,0.3, false);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    intake.setPower(0);
                    pathState = PathState.OUTTAKE2;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case OUTTAKE2:
                if(!moving){
                    follower.followPath(Outtake2, true);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    pathState = PathState.SHOOT2;
                    pathTimer.resetTimer();
                    moving = false;
                }
                break;
            case SHOOT2:
                if(!follower.isBusy()){
                    loopOuttake(66);
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                }
                if (shotCount >= 3) {
                    flyWheelPower = 0;
                    pusherPos = 0.4;
                    fireState = 0;
                    shotCount = 0;
                    pathState = PathState.END;
                    flyWheel.setVelocity(flyWheelPower);
                    pusher.setPosition(pusherPos);
                    pathTimer.resetTimer();
                }
                break;
            case END:
                if(!moving) {
                    intake.setPower(0);
                    turret.setTargetPosition(0);
                    follower.followPath(End,false);
                    moving = true;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 50){
                    moving = false;
                    pathState = PathState.STOP;
                }
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
                pusherPos = 0.95;
                fireTimer.resetTimer();
                fireState = 2;
            }
        }

        else if (fireState == 2) {
            if (fireTimer.getElapsedTime() > 400) {
                pusherPos = 0.4;
                fireTimer.resetTimer();
                fireState = 3;
            }
        }

        else if (fireState == 3) {
            intake.setPower(1);
            if (fireTimer.getElapsedTime() > 700) {
                shotCount++;
                fireState = 1;
            }
        }
    }
}
