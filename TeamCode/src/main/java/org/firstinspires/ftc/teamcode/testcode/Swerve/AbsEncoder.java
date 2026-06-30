package org.firstinspires.ftc.teamcode.testcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
@Disabled
public class AbsEncoder {
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }
    public static final double MAX_VOLT = 3.3;

    private final AnalogInput analogInput;
    private boolean inverted = false;
    private double offset = 0; // radians
    public AbsEncoder(AnalogInput analogInput) {
        this.analogInput = analogInput;
    }
    public AbsEncoder(AnalogInput analogInput, double offset) {
        this.analogInput = analogInput;
        this.offset = offset;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void setInverted(Direction direction) {
        this.inverted = (direction == Direction.REVERSE);
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getPos() {
        double voltage = analogInput.getVoltage();
        double angle = (voltage / MAX_VOLT) * 2 * Math.PI;

        if (inverted) {
            angle = 2 * Math.PI - angle;
        }

        angle -= offset;

        // Normalize
        angle %= (2 * Math.PI);
        if (angle < 0){
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }
}
