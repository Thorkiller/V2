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
     * Lookup table entries for passing.
     *
     * These are spaced at 0.32 m increments and biased a little higher than the
     * normal shot table so the pass arc carries more clearance through midfield.
     * Hood values are stored directly in degrees for readability.
     */
    private static final LookUpTableEntryPass[] LOOKUP_TABLE = {
        // Format: distance meters, hoodPivotDegrees, shooter RPM
        new LookUpTableEntryPass(1.448, 20.5701754386, 2460),
        new LookUpTableEntryPass(1.768, 22.1052631579, 2670),
        new LookUpTableEntryPass(2.088, 23.6403508772, 2595),
        new LookUpTableEntryPass(2.408, 25.2982456140, 2845),
        new LookUpTableEntryPass(2.728, 26.8947368421, 2765),
        new LookUpTableEntryPass(3.048, 28.3684210526, 3040),
        new LookUpTableEntryPass(3.368, 29.8421052632, 3165),
        new LookUpTableEntryPass(3.688, 31.0701754386, 3285),
        new LookUpTableEntryPass(4.008, 32.1754385965, 3415),
        new LookUpTableEntryPass(4.328, 33.0350877193, 3540),
        new LookUpTableEntryPass(4.648, 33.7719298246, 3665),
        new LookUpTableEntryPass(4.968, 34.2631578947, 3795),
        new LookUpTableEntryPass(5.288, 34.6315789474, 3925),
        new LookUpTableEntryPass(5.608, 34.8771929825, 4055),
        new LookUpTableEntryPass(5.928, 35.0000000000, 4380),
        new LookUpTableEntryPass(6.248, 35.0000000000, 4510),
        new LookUpTableEntryPass(6.568, 35.0000000000, 4640),
        new LookUpTableEntryPass(6.888, 35.0000000000, 4770),
        new LookUpTableEntryPass(7.208, 35.0000000000, 4900),
        new LookUpTableEntryPass(7.528, 35.0000000000, 5030),
        new LookUpTableEntryPass(7.848, 35.0000000000, 5160),
        new LookUpTableEntryPass(8.168, 35.0000000000, 5290),
        new LookUpTableEntryPass(8.488, 35.0000000000, 5420),
        new LookUpTableEntryPass(8.808, 35.0000000000, 5550),
        new LookUpTableEntryPass(9.128, 35.0000000000, 5680),
        new LookUpTableEntryPass(9.448, 35.0000000000, 5810),
        new LookUpTableEntryPass(9.768, 35.0000000000, 5940),
        new LookUpTableEntryPass(10.000, 35.0000000000, 6050)
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
