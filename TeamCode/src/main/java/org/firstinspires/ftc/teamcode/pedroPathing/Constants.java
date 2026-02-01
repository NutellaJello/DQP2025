package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerAutoConstants = new FollowerConstants()
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .mass(11.52125)
            .forwardZeroPowerAcceleration(-36.20)
            .lateralZeroPowerAcceleration(-66.38)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.01, 0.01, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.09,0,0.01,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.2,0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.05,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.01,0.000005,0.6,0.01))
            .centripetalScaling(0.0005);

    public static FollowerConstants followerTeleopConstants = new FollowerConstants()
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .mass(11.52125)
            .forwardZeroPowerAcceleration(-36.20)
            .lateralZeroPowerAcceleration(-66.38)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.01, 0.01, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.015,0.035))
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.2,0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5,0,0.08,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.01,0.000005,0.6,0.01))
            .centripetalScaling(0.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(85.3)
            .yVelocity(60.42);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.36)
            .strafePodX(-0.94)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createAutoFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerAutoConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static Follower createTeleopFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerTeleopConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
