package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
public class Constants {
    public static final double MK2_PWM_MIN = 500;
    public static final double MK2_PWM_MAX = 2500;
    public static final double MINI_PWM_MIN = 500;  // check datasheet
    public static final double MINI_PWM_MAX = 2500;  // check datasheet

    public static final double MINI_ROTATE = 355;
    public static final double MK2_ROTATE = 355;
    public static final String FL_SERVO_T = "mini+";
    public static final String FR_SERVO_T = "mk2";
    public static final String BL_SERVO_T = "mk2";
    public static final String BR_SERVO_T = "mini+";
    //Distance between wheels
    public static final double WheelWidth = 5.35433;
    public static final double WheelLen  = 9.984252;

    public static final double FLoffset = 0.64;
    public static final double FRoffset = 0.65;
    public static final double BLoffset = 0.52;
    public static final double BRoffset = 0.36;
    //Inverts
    public static final boolean FLMinv = false;
    public static final boolean FRMinv = true;
    public static final boolean BLMinv = false;
    public static final boolean BRMinv = true;

    public static final boolean FLSinv = false;
    public static final boolean FRSinv = true;
    public static final boolean BLSinv = true;
    public static final boolean BRSinv = false;

    public static final boolean FLEinv = false;
    public static final boolean FREinv = false;
    public static final boolean BLEinv = false;
    public static final boolean BREinv = false;

    // Names
    public static final String FLM = "FL";
    public static final String FLS = "flservo";
    public static final String FLE = "flEncoder";

    public static final String FRM = "FR";
    public static final String FRS = "frservo";
    public static final String FRE = "frEncoder";

    public static final String BLM = "BL";
    public static final String BLS = "blservo";
    public static final String BLE = "blEncoder";

    public static final String BRM = "BR";
    public static final String BRS = "brservo";
    public static final String BRE = "brEncoder";
    public static final String IMU = "pinpoint";
    public static final double maxspeed = 0.6;
    public static final double maxturnspeed = 0.05;
    public static final double inputbuffer = 0.05;
}
