package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {

    private FieldConstants() {
    }

    public static final double FIELD_LENGTH = Units.inchesToMeters(651.22);

    public static final double TARGET_INSET = Units.inchesToMeters(10.0); // avoid wall

    // X positions
    public static final double BLUE_TARGET_X = TARGET_INSET;
    public static final double RED_TARGET_X = FIELD_LENGTH - TARGET_INSET;

    // Y positions (from drawing)
    public static final double LOWER_TARGET_Y = Units.inchesToMeters(132.63);

    public static final double UPPER_TARGET_Y = Units.inchesToMeters(75.93);

    // 🔵 Blue alliance targets
    public static final Translation2d BLUE_LOWER_TARGET = new Translation2d(BLUE_TARGET_X, LOWER_TARGET_Y);

    public static final Translation2d BLUE_UPPER_TARGET = new Translation2d(BLUE_TARGET_X, UPPER_TARGET_Y);

    // 🔴 Red alliance targets (mirrored)
    public static final Translation2d RED_LOWER_TARGET = new Translation2d(RED_TARGET_X, LOWER_TARGET_Y);

    public static final Translation2d RED_UPPER_TARGET = new Translation2d(RED_TARGET_X, UPPER_TARGET_Y);

    public static final Translation2d[] BLUE_TARGETS = {
            BLUE_LOWER_TARGET,
            BLUE_UPPER_TARGET
    };

    public static final Translation2d[] RED_TARGETS = {
            RED_LOWER_TARGET,
            RED_UPPER_TARGET
    };
}