package frc.robot.util;

public class LookUpTablePass {

    /**
     * Represents a single entry in the lookup table with distance and corresponding values
     */
    private static class LookUpTableEntryPass {
        final double distance;
        final double hoodpiv;
        final double rpm;

        public LookUpTableEntryPass(double distance, double hoodpiv, double rpm) {
            this.distance = distance;
            this.hoodpiv = hoodpiv;
            this.rpm = rpm;
        }
    }

    /**
     * Lookup table entries - adjust these values based on your robot's calibration
     * Distance is in meters (or your preferred unit)
     */
    private static final LookUpTableEntryPass[] LOOKUP_TABLE = {
        // Format: distance, armPivot (0..90, larger when closer), shooterVelocity (e.g., RPM)
        new LookUpTableEntryPass(1.448, 2.6, 2623),
        new LookUpTableEntryPass(1.543,   3.3, 2662),
        new LookUpTableEntryPass(1.642,    3.6, 2600),
        new LookUpTableEntryPass(1.889,   3.8, 2663),
        new LookUpTableEntryPass(2.146,    4.1, 2663),
        new LookUpTableEntryPass(2.46,   4.2, 2663),
        new LookUpTableEntryPass(2.722,    4.35, 2750),
        new LookUpTableEntryPass(3.032,    4.4, 2900),
        new LookUpTableEntryPass(3.297,    4.4, 3001),
        new LookUpTableEntryPass(4.0,     4.3, 3000)
    };

    public LookUpTablePass() {
    }

    public static class LookUpTableTest {
        double distance = 0;
        double armpiv = 0;
        double shooterRPS = 0;

        public LookUpTableTest(double hoodpiv, double shooterRPS) {
            this.armpiv = hoodpiv;
            this.shooterRPS = shooterRPS;
        }

        public double getAngle() {
            return armpiv;
        }

        public double getRPM() {
            return shooterRPS;
        }
    }

    /**
     * Gets the lookup table output for a given distance using linear interpolation
     * 
     * @param distance The distance to look up (in meters or your preferred unit)
     * @return LookUpTableTest containing interpolated values for arm pivot and shooter speeds
     */
    public static LookUpTableTest LookUpTableOutput(double distance) {
        // Handle edge cases: distance below minimum or above maximum
        if (distance <= LOOKUP_TABLE[0].distance) {
            LookUpTableEntryPass entry = LOOKUP_TABLE[0];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        if (distance >= LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance) {
            LookUpTableEntryPass entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < LOOKUP_TABLE.length - 1; i++) {
            LookUpTableEntryPass lower = LOOKUP_TABLE[i];
            LookUpTableEntryPass upper = LOOKUP_TABLE[i + 1];

            if (distance >= lower.distance && distance <= upper.distance) {
                // Linear interpolation
                double t = (distance - lower.distance) / (upper.distance - lower.distance);

                double interpolatedArmPiv = lerp(lower.hoodpiv, upper.hoodpiv, t);
                double interpolatedShooterRight = lerp(lower.rpm, upper.rpm, t);

                return new LookUpTableTest(interpolatedArmPiv, interpolatedShooterRight);
            }
        }

        // Fallback (should never reach here)
        LookUpTableEntryPass entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
        return new LookUpTableTest(entry.hoodpiv, entry.rpm);
    }

    /**
     * Linear interpolation helper function
     * 
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated value
     */
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
