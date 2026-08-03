package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.model.MotionModel;

// drivetrain physics — PRIMARY path-following tuning (kS/kV/kA, mass, accel limits).
// Pedro PID in RobotConfiguration is correction-only; leave it alone for normal autos.
public class RobotModel implements MotionModel {
    public double mass = 12.0;
    public double wheelRadius = 1.8898;
    public double motorFreeSpeed = 32.67;
    public double gearRatio = 1.0;
    public double nominalVoltage = 12.0;
    public double batteryVoltage = 12.0;
    public double frictionCoefficient = 0.7;
    public double gravity = 386.09;
    public double maxAcceleration = 80.0;
    public double maxDeceleration = 100.0;
    public double maxLateralAcceleration = 0.7 * 386.09;
    public double maxVelocityOverride = 0.0;
    public double maxAngularVelocity = 6.0;
    public double maxAngularAcceleration = 20.0;
    public double kS = 0.05;
    public double kV = 0.012;
    public double kA = 0.002;
    public double motorEfficiency = 0.85;

    /**
     * Per-tick path velocity ceiling (in/s) set by PathFollower adapter from
     * {@link org.firstinspires.ftc.teamcode.brainstem.follower.VelocityConstraint}.
     * Not a tunable constant — transient. {@link Double#POSITIVE_INFINITY} = no extra cap.
     */
    private double pathVelocityCeiling = Double.POSITIVE_INFINITY;

    public enum MotionMode {
        CRUISE(1.0, 1.0),
        LOADED(0.72, 0.85),
        PRECISION(0.42, 0.7);

        public final double velocityScale;
        public final double accelScale;

        MotionMode(double velocityScale, double accelScale) {
            this.velocityScale = velocityScale;
            this.accelScale = accelScale;
        }
    }

    public MotionMode motionMode = MotionMode.CRUISE;
    public double localizationConfidence = 1.0;

    public RobotModel() {
        refreshDerivedLimits();
    }

    public void refreshDerivedLimits() {
        maxLateralAcceleration = frictionCoefficient * gravity;
    }

    /**
     * Read-only view of motion limits — single source of truth is this model’s fields.
     * Does not copy values; always reflects live state.
     */
    public MotionConstraints constraints() {
        return new MotionConstraints(this);
    }

    /** Called by PathFollower adapters each tick; not for manual tuning. */
    public void setPathVelocityCeiling(double inchesPerSecond) {
        if (Double.isNaN(inchesPerSecond) || inchesPerSecond <= 0) {
            pathVelocityCeiling = Double.POSITIVE_INFINITY;
        } else {
            pathVelocityCeiling = inchesPerSecond;
        }
    }

    public void clearPathVelocityCeiling() {
        pathVelocityCeiling = Double.POSITIVE_INFINITY;
    }

    public double getPathVelocityCeiling() {
        return pathVelocityCeiling;
    }

    @Override
    public void cruise() {
        motionMode = MotionMode.CRUISE;
    }

    @Override
    public void loaded() {
        motionMode = MotionMode.LOADED;
    }

    @Override
    public void precision() {
        motionMode = MotionMode.PRECISION;
    }

    private double contextScale() {
        double confidence = clamp(localizationConfidence, 0.4, 1.0);
        return motionMode.velocityScale * confidence;
    }

    /** Robot capability without the transient path ceiling (for VelocityConstraint). */
    public double motorLimitedVelocityIgnoringPathCeiling() {
        double base;
        if (maxVelocityOverride > 0) {
            base = maxVelocityOverride;
        } else {
            double wheelOmega = (motorFreeSpeed / Math.max(gearRatio, 1e-6)) * motorEfficiency;
            double velocityAtNominal = wheelOmega * wheelRadius;
            base = velocityAtNominal * (batteryVoltage / Math.max(nominalVoltage, 1e-3));
        }
        return base * contextScale();
    }

    @Override
    public double motorLimitedVelocity() {
        return Math.min(motorLimitedVelocityIgnoringPathCeiling(), pathVelocityCeiling);
    }

    @Override
    public double getMaxLateralAcceleration() {
        return maxLateralAcceleration;
    }

    @Override
    public double profileMaxAcceleration() {
        return maxAcceleration * motionMode.accelScale;
    }

    @Override
    public double profileMaxDeceleration() {
        return maxDeceleration * motionMode.accelScale;
    }

    @Override
    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    @Override
    public double getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }

    @Override
    public double feedforwardPower(double velocity, double acceleration) {
        double rawVolts = kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
        double compensated = rawVolts * (nominalVoltage / Math.max(batteryVoltage, 1.0));
        return clamp(compensated / nominalVoltage, -1.0, 1.0);
    }

    @Override
    public double velocityAtVoltageSaturation(double acceleration) {
        double available = batteryVoltage - kS - Math.abs(kA * acceleration);
        return available <= 0 ? 0 : available / Math.max(kV, 1e-6);
    }

    @Override
    public void setBatteryVoltage(double batteryVoltage) {
        this.batteryVoltage = batteryVoltage;
    }

    @Override
    public void setLocalizationConfidence(double localizationConfidence) {
        this.localizationConfidence = localizationConfidence;
    }

    public RobotModel mass(double mass) {
        this.mass = mass;
        return this;
    }

    public RobotModel wheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
        return this;
    }

    public RobotModel motorFreeSpeed(double motorFreeSpeed) {
        this.motorFreeSpeed = motorFreeSpeed;
        return this;
    }

    public RobotModel gearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public RobotModel frictionCoefficient(double frictionCoefficient) {
        this.frictionCoefficient = frictionCoefficient;
        refreshDerivedLimits();
        return this;
    }

    public RobotModel maxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public RobotModel maxDeceleration(double maxDeceleration) {
        this.maxDeceleration = maxDeceleration;
        return this;
    }

    public RobotModel maxAngularVelocity(double maxAngularVelocity) {
        this.maxAngularVelocity = maxAngularVelocity;
        return this;
    }

    public RobotModel maxAngularAcceleration(double maxAngularAcceleration) {
        this.maxAngularAcceleration = maxAngularAcceleration;
        return this;
    }

    public RobotModel feedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    /**
     * Live view of motion limits. Values are read from the owning {@link RobotModel}
     * — do not store a parallel copy of limits elsewhere.
     */
    public static final class MotionConstraints {
        private final RobotModel model;

        MotionConstraints(RobotModel model) {
            this.model = model;
        }

        public double maxVelocity() {
            return model.motorLimitedVelocityIgnoringPathCeiling();
        }

        public double maxAcceleration() {
            return model.profileMaxAcceleration();
        }

        public double maxDeceleration() {
            return model.profileMaxDeceleration();
        }

        public double frictionCoefficient() {
            return model.frictionCoefficient;
        }

        public double maxLateralAcceleration() {
            return model.getMaxLateralAcceleration();
        }

        public RobotModel.MotionMode motionMode() {
            return model.motionMode;
        }
    }
}
