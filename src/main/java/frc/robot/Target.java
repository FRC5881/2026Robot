package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum Target {
    /**
     * Score in the hub
     */
    kHub,
    /**
     * Pass to the left side of our alliance zone (from the perspective of our
     * drivers)
     */
    kLeftAllianceZone,
    /**
     * Pass to the right side of our alliance zone (from the perspective of our
     * drivers)
     */
    kRightAllianceZone;

    /**
     * getPosition converts a generic target like 'kHub' into the Hub's true
     * position
     * on the field (in the field's reference frame). Since we target different hubs
     * based on our alliance color we pass the alliance color to this method.
     * 
     * @param alliance `DriverStation.getAlliance()`
     * @return Field relative position of the target
     */
    public Translation2d getPosition(Alliance alliance) {
        // TODO: implement
        switch (alliance) {
            case Blue:
                switch (this) {
                    case kHub:
                        return Translation2d.kZero;
                    case kLeftAllianceZone:
                        return Translation2d.kZero;
                    case kRightAllianceZone:
                        return Translation2d.kZero;
                }
            case Red:
                switch (this) {
                    case kHub:
                        return Translation2d.kZero;
                    case kLeftAllianceZone:
                        return Translation2d.kZero;
                    case kRightAllianceZone:
                        return Translation2d.kZero;
                }
        }

        // unreachable
        return Translation2d.kZero;
    }

    /**
     * Returns the acceptable accuracy for hitting the target (in meters)
     */
    public double maxAcceptableError() {
        // TODO: tune
        if (this.equals(kHub)) {
            // The hub is 42 inch diamter, divide by 2 for the radius, divide by 2 again to
            // require higher accuracy
            return Units.inchesToMeters(42.0 / 4.0);
        }

        // for hitting the ground +/- 1 meter should be good enough
        return 1;
    }

}