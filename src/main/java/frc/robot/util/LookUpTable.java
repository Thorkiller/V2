package frc.robot.util;

public class LookUpTable {

    /**
     * Represents a single entry in the lookup table with distance and corresponding values
     */
    private static class LookUpTableEntry {
        final double distance;
        final double hoodpiv;
        final double rpm;

        public LookUpTableEntry(double distance, double hoodpiv, double rpm) {
            this.distance = distance;
            this.hoodpiv = hoodpiv;
            this.rpm = rpm;
        }
    }

    /**
     * Lookup table entries - adjust these values based on your robot's calibration
     * Distance is in meters (or your preferred unit)
     */
    private static final LookUpTableEntry[] LOOKUP_TABLE = {
        // Format: distance, hoodPivotDegrees, shooterVelocity (RPM)
        new LookUpTableEntry(1.448, 15.9649122807, 2480),
        new LookUpTableEntry(1.543, 20.2631578947, 2760),
        new LookUpTableEntry(1.642, 22.1052631579, 2465),
        new LookUpTableEntry(1.889, 23.3333333333, 2835),
        new LookUpTableEntry(2.146, 25.1754385965, 2540),
        new LookUpTableEntry(2.460, 25.7894736842, 2865),
        new LookUpTableEntry(2.722, 26.7105263158, 2980),
        new LookUpTableEntry(3.032, 27.0175438596, 3140),
        new LookUpTableEntry(3.297, 27.0175438596, 3290),
        new LookUpTableEntry(3.617, 27.3245614035, 3445),
        new LookUpTableEntry(3.937, 27.5087719298, 3575),
        new LookUpTableEntry(4.257, 27.6315789474, 3705),
        new LookUpTableEntry(4.577, 27.7543859649, 3830),
        new LookUpTableEntry(4.897, 27.9385964912, 3960),
        new LookUpTableEntry(5.217, 28.1228070175, 4095),
        new LookUpTableEntry(5.537, 28.2456140351, 4235),
        new LookUpTableEntry(5.857, 28.3684210526, 4380),
        
        
    };

    public LookUpTable() {
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

        public double getRPS() {
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
            LookUpTableEntry entry = LOOKUP_TABLE[0];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        if (distance >= LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance) {
            LookUpTableEntry entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < LOOKUP_TABLE.length - 1; i++) {
            LookUpTableEntry lower = LOOKUP_TABLE[i];
            LookUpTableEntry upper = LOOKUP_TABLE[i + 1];

            if (distance >= lower.distance && distance <= upper.distance) {
                // Linear interpolation
                double t = (distance - lower.distance) / (upper.distance - lower.distance);

                double interpolatedArmPiv = lerp(lower.hoodpiv, upper.hoodpiv, t);
                double interpolatedShooterRight = lerp(lower.rpm, upper.rpm, t);

                return new LookUpTableTest(interpolatedArmPiv, interpolatedShooterRight);
            }
        }

        // Fallback (should never reach here)
        LookUpTableEntry entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
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
