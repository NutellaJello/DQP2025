package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Disabled
public class SwerveDrivetrain {

    private final SwerveModule fl;
    private final SwerveModule fr;
    private final SwerveModule bl;
    private final SwerveModule br;
    private final SwerveModule[] mods;

    private final GoBildaPinpointDriver pinpoint; //may not be configured right

    public SwerveDrivetrain(HardwareMap hardwareMap) {

        fl = new SwerveModule(hardwareMap, Constants.FLM, Constants.FLS, Constants.FLE, Constants.FLMinv, Constants.FLSinv, Constants.FLEinv, Constants.FLoffset, "FL", Constants.FL_SERVO_T);

        fr = new SwerveModule(hardwareMap, Constants.FRM, Constants.FRS, Constants.FRE, Constants.FRMinv, Constants.FRSinv, Constants.FREinv, Constants.FRoffset, "FR", Constants.FR_SERVO_T);

        bl = new SwerveModule(hardwareMap, Constants.BLM, Constants.BLS, Constants.BLE, Constants.BLMinv, Constants.BLSinv, Constants.BLEinv, Constants.BLoffset, "BL", Constants.BL_SERVO_T);

        br = new SwerveModule(hardwareMap, Constants.BRM, Constants.BRS, Constants.BRE, Constants.BRMinv, Constants.BRSinv, Constants.BREinv, Constants.BRoffset, "BR", Constants.BR_SERVO_T);

        mods = new SwerveModule[]{fl, fr, bl, br};


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
    }

    public void drive(double strafeX, double forwardY, double rotation,
                      boolean fieldCentric) {
        strafeX  = Math.abs(strafeX) < Constants.inputbuffer ? 0 : strafeX;
        forwardY = Math.abs(forwardY) < Constants.inputbuffer ? 0 : forwardY;
        rotation = Math.abs(rotation) < Constants.inputbuffer ? 0 : rotation;


        if (fieldCentric) {
            double heading = getHeading();
            double temp = forwardY * Math.cos(heading) + strafeX * Math.sin(heading);
            strafeX    = -forwardY * Math.sin(heading) + strafeX * Math.cos(heading);
            forwardY   = temp;
        }

        strafeX  *= Constants.maxspeed;
        forwardY *= Constants.maxspeed;
        rotation *= Constants.maxturnspeed;

        double[] vx = new double[4];
        double[] vy = new double[4];

        vx[0] = forwardY - rotation * Constants.WheelWidth / 2.0;
        vy[0] = strafeX  + rotation * Constants.WheelLen  / 2.0;

        vx[1] = forwardY + rotation * Constants.WheelWidth / 2.0;
        vy[1] = strafeX  + rotation * Constants.WheelLen  / 2.0;

        vx[2] = forwardY - rotation * Constants.WheelWidth / 2.0;
        vy[2] = strafeX  - rotation * Constants.WheelLen  / 2.0;

        vx[3] = forwardY + rotation * Constants.WheelWidth / 2.0;
        vy[3] = strafeX  - rotation * Constants.WheelLen  / 2.0;

        double[] speeds = new double[4];
        double[] angles = new double[4];
        double maxSpeed = 0;

        for (int i = 0; i < 4; i++) {
            speeds[i] = Math.hypot(vx[i], vy[i]);
            angles[i] = Math.atan2(vy[i], vx[i]);
            maxSpeed   = Math.max(maxSpeed, speeds[i]);
        }

        if (maxSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        for (int i = 0; i < 4; i++) {
            mods[i].setTargetState(speeds[i], angles[i]);
        }
    }
    public void update() {
        for (SwerveModule m : mods) {
            m.update();
        }
    }

    public void stop() {
        for (SwerveModule m : mods) {
            m.setDrivePower(0);
        }
    }

    public double getHeading() {
        pinpoint.update();
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }
    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }
    public void startServos(){
        for(SwerveModule m : mods){
            m.setPos();
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Heading", "%.5f",getHeading());
        telemetry.addLine();

        for (SwerveModule m : mods) {
            telemetry.addData(m.getName() + " actual", "%.5f rad", m.getModuleRotation());
            telemetry.addData(m.getName() + " target", "%.5f rad", m.getTargetHeading());
            telemetry.addData(m.getName() + " encoder V", "%.3f V", m.getEncoderVoltage());
            telemetry.addData(m.getName() + " servo", "%.3f", m.getServoPosition());
            telemetry.addData(m.getName() + " power", "%.2f", m.getLastDrivePower());
            telemetry.addData(m.getName() + " deadzone", m.isDZ());
            telemetry.addLine();
        }
    }

}
