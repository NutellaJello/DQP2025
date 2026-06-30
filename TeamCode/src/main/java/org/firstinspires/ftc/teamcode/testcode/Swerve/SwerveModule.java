package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class SwerveModule {

    public double servorange = 355.0;
    public double servoforward;        // MUST UPDATE


    public static double servomin = 0.05; //maybe remove
    public static double servomax = 0.95; //maybe remove
    public static double turnbuffer = 0.00; //maybe remove

    private final DcMotorEx motor;
    private final Servo servo;
    private final AbsEncoder encoder;

    private double targetHeading = 0;
    private double lastDrivePower = 0;

    private final String name;

    private boolean dead = false;
    private String type = "mk2";
    public SwerveModule(DcMotorEx motor, Servo servo, AbsEncoder encoder, double offset, String name) {
        this.name = name;
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
        servoforward = offset;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String encoderName, boolean invertMotor, boolean invertServo, boolean invertEncoder, double offset, String name, String type) {

        this(hardwareMap.get(DcMotorEx.class, motorName), hardwareMap.get(Servo.class, servoName), new AbsEncoder(hardwareMap.get(AnalogInput.class, encoderName), offset), offset, name);
        this.type = type;
        if(type.equals("mini+")){
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(Constants.MINI_PWM_MIN, Constants.MINI_PWM_MAX));
            servorange = Constants.MINI_ROTATE;
        }else{
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(Constants.MK2_PWM_MIN, Constants.MK2_PWM_MAX));
            servorange = Constants.MK2_ROTATE;
        }
        servo.setDirection(Servo.Direction.FORWARD);
        if (invertMotor)   motor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (invertServo)   servo.setDirection(Servo.Direction.REVERSE);
        if (invertEncoder) encoder.setInverted(true);
    }
    public static double[] optimize(double targetSpeed, double targetHeading, double currentHeading) {
        double error = normAngle(targetHeading - currentHeading);

        if (Math.abs(error) > Math.PI / 2) {
            targetHeading = normPosAngle(targetHeading + Math.PI);
            targetSpeed   = -targetSpeed;
        }

        return new double[]{ targetSpeed, targetHeading };
    }

    private boolean isDZ(double heading) {
        double headingDeg = Math.toDegrees(normAngle(heading));
        double position = servoforward + (headingDeg / servorange);
        return position < servomin || position > servomax;
    }

    public void setTargetState(double speed, double heading) {
        if (Math.abs(speed) < turnbuffer) {
            setDrivePower(speed);
            return;
        }

        double[] optimized = optimize(speed, heading, getModuleRotation());
        double optSpeed   = optimized[0];
        double optHeading = optimized[1];


        if (isDZ(optHeading)) {
            optHeading = flipHeading(optHeading);
            optSpeed   = -optSpeed;
            dead = true;
        }else{
            dead = false;
        }


        targetHeading  = optHeading;
        lastDrivePower = optSpeed;
        motor.setPower(optSpeed);
    }
    private double flipHeading(double heading) {
        double plus  = normAngle(heading + Math.PI);
        double minus = normAngle(heading - Math.PI);


        if (plus <= 1) {
            return plus;
        } else {
            return minus;
        }


    }

    public void move(double speedFraction, double angleFraction) {
        setTargetState(speedFraction, angleFraction * 2 * Math.PI);
    }

    public void update() {
        servo.setPosition(headingToServoPosition(targetHeading));
    }
    public void setPos(){
        servo.setPosition(servoforward);
    }
    private double headingToServoPosition(double headingRad) {
        double headingDeg = Math.toDegrees(normAngle(headingRad));
        double position = servoforward + (headingDeg / servorange);
        return Math.max(servomin, Math.min(servomax, position));
    }
    public void setDrivePower(double power) {
        lastDrivePower = power;
        motor.setPower(power);
    }
    public double getModuleRotation()  { return encoder.getPos(); }
    public double getEncoderVoltage()  { return encoder.getVoltage(); }
    public double getTargetHeading()   { return targetHeading; }
    public int    getWheelPosition() {return motor.getCurrentPosition(); }
    public double getWheelVelocity() {return motor.getVelocity(); }
    public String getName() {return name; }
    public double getLastDrivePower() {return lastDrivePower; }
    public double getServoPosition() {return servo.getPosition(); }
    public boolean isDZ() {return dead; }


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
