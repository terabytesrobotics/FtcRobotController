package org.firstinspires.ftc.teamcode.control;

/** Named limits and completion criteria for a drive-to-pose behavior. */
public final class DriveProfile {
    private final String name;
    private final double maxTranslationPower;
    private final double maxRotationPower;
    private final double positionToleranceMm;
    private final double headingToleranceRadians;
    private final double settleTimeSeconds;
    private final double timeoutSeconds;

    private DriveProfile(Builder builder) {
        name = builder.name;
        maxTranslationPower = builder.maxTranslationPower;
        maxRotationPower = builder.maxRotationPower;
        positionToleranceMm = builder.positionToleranceMm;
        headingToleranceRadians = Math.toRadians(builder.headingToleranceDeg);
        settleTimeSeconds = builder.settleTimeMs / 1000.0;
        timeoutSeconds = builder.timeoutMs / 1000.0;
    }

    public static Builder named(String name) {
        return new Builder(name);
    }

    public String getName() {
        return name;
    }

    public double getMaxTranslationPower() {
        return maxTranslationPower;
    }

    public double getMaxRotationPower() {
        return maxRotationPower;
    }

    public double getPositionToleranceMm() {
        return positionToleranceMm;
    }

    public double getHeadingToleranceRadians() {
        return headingToleranceRadians;
    }

    public double getSettleTimeSeconds() {
        return settleTimeSeconds;
    }

    public double getTimeoutSeconds() {
        return timeoutSeconds;
    }

    public static final class Builder {
        private final String name;
        private double maxTranslationPower = 0.75;
        private double maxRotationPower = 0.60;
        private double positionToleranceMm = 20.0;
        private double headingToleranceDeg = 5.0;
        private double settleTimeMs = 200.0;
        private double timeoutMs = 5000.0;

        private Builder(String name) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("Drive profile name must not be empty");
            }
            this.name = name;
        }

        public Builder maxTranslationPower(double value) {
            maxTranslationPower = requireUnitPower("maxTranslationPower", value);
            return this;
        }

        public Builder maxRotationPower(double value) {
            maxRotationPower = requireUnitPower("maxRotationPower", value);
            return this;
        }

        public Builder positionToleranceMm(double value) {
            positionToleranceMm = requireNonNegative("positionToleranceMm", value);
            return this;
        }

        public Builder headingToleranceDeg(double value) {
            headingToleranceDeg = requireNonNegative("headingToleranceDeg", value);
            return this;
        }

        public Builder settleTimeMs(double value) {
            settleTimeMs = requireNonNegative("settleTimeMs", value);
            return this;
        }

        public Builder timeoutMs(double value) {
            timeoutMs = requireNonNegative("timeoutMs", value);
            return this;
        }

        public DriveProfile build() {
            return new DriveProfile(this);
        }

        private static double requireUnitPower(String label, double value) {
            if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
                throw new IllegalArgumentException(label + " must be between 0 and 1");
            }
            return value;
        }

        private static double requireNonNegative(String label, double value) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(label + " must be non-negative");
            }
            return value;
        }
    }
}
