package com.pedropathing.model;

/**
 * Lumped physical model of the drivetrain used for constraint generation and feedforward.
 *
 * Motor voltage model (normalized to power in [-1, 1] when divided by battery voltage):
 * <pre>
 * V = kS * sgn(v) + kV * v + kA * a
 * </pre>
 *
 * Units are inches / seconds / radians unless noted.
 */
public class RobotModel {
    /** Robot mass (kg). Used for traction estimates and documentation; aMax may override. */
    public double mass = 12.0;

    /** Driven wheel radius (inches). */
    public double wheelRadius = 1.8898; // 48 mm

    /** Motor free speed (rad/s) at nominal voltage (e.g. 312 RPM ≈ 32.67 rad/s). */
    public double motorFreeSpeed = 32.67;

    /** Overall gear reduction from motor to wheel (motor revs / wheel revs). */
    public double gearRatio = 1.0;

    /** Nominal design voltage (V). */
    public double nominalVoltage = 12.0;

    /** Planning / last-known battery voltage (V). */
    public double batteryVoltage = 12.0;

    /** Carpet / tile coefficient of friction μ. */
    public double frictionCoefficient = 0.7;

    /** Gravity (in/s^2). */
    public double gravity = 386.09;

    /** Cap on longitudinal acceleration used by the profiler (in/s^2). */
    public double maxAcceleration = 80.0;

    /** Cap on longitudinal deceleration magnitude as a positive number (in/s^2). */
    public double maxDeceleration = 100.0;

    /** Cap on lateral (centripetal) acceleration (in/s^2). Defaults near μ*g. */
    public double maxLateralAcceleration = 0.7 * 386.09;

    /** Absolute max translation velocity if unconstrained by motors (in/s). 0 = derive from motors. */
    public double maxVelocityOverride = 0.0;

    /** Absolute max angular velocity (rad/s). */
    public double maxAngularVelocity = 6.0;

    /** Absolute max angular acceleration (rad/s^2). */
    public double maxAngularAcceleration = 20.0;

    /** Static friction / voltage intercept (V). */
    public double kS = 0.05;

    /** Velocity gain (V / (in/s)). */
    public double kV = 0.012;

    /** Acceleration gain (V / (in/s^2)). */
    public double kA = 0.002;

    /** Fraction of free-speed reachable under load before current limiting. */
    public double motorEfficiency = 0.85;

    /**
     * Phase A motion mode — scales profile velocity without changing English APIs.
     * CRUISE = full profile, LOADED = carrying, PRECISION = align / score approach.
     */
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

    /**
     * 1 = trust localization; lower slows profiles (fed by FusedLocalizer / EKF).
     * Clamped when applied.
     */
    public double localizationConfidence = 1.0;

    public RobotModel() {
        refreshDerivedLimits();
    }

    /** Recompute lateral accel from μ*g if the user only sets friction/mass. */
    public void refreshDerivedLimits() {
        maxLateralAcceleration = frictionCoefficient * gravity;
    }

    public void cruise() {
        motionMode = MotionMode.CRUISE;
    }

    public void loaded() {
        motionMode = MotionMode.LOADED;
    }

    public void precision() {
        motionMode = MotionMode.PRECISION;
    }

    private double contextScale() {
        double conf = localizationConfidence;
        if (conf < 0.4) conf = 0.4;
        if (conf > 1.0) conf = 1.0;
        return motionMode.velocityScale * conf;
    }

    /**
     * Theoretical motor-limited free-speed linear velocity at the current battery voltage,
     * scaled by {@link #motionMode} and {@link #localizationConfidence}.
     */
    public double motorLimitedVelocity() {
        double base;
        if (maxVelocityOverride > 0) base = maxVelocityOverride;
        else {
            double wheelOmega = (motorFreeSpeed / Math.max(gearRatio, 1e-6)) * motorEfficiency;
            double vAtNominal = wheelOmega * wheelRadius;
            base = vAtNominal * (batteryVoltage / Math.max(nominalVoltage, 1e-3));
        }
        return base * contextScale();
    }

    public double profileMaxAcceleration() {
        return maxAcceleration * motionMode.accelScale;
    }

    public double profileMaxDeceleration() {
        return maxDeceleration * motionMode.accelScale;
    }

    /**
     * Raw feedforward voltage from the linear motor model, then scaled to battery and
     * converted to normalized motor power relative to {@link #nominalVoltage}.
     */
    public double feedforwardPower(double velocity, double acceleration) {
        double rawVolts = kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
        double compensated = rawVolts * (nominalVoltage / Math.max(batteryVoltage, 1.0));
        return clamp(compensated / nominalVoltage, -1.0, 1.0);
    }

    /**
     * Max velocity that saturates feedforward at ±1 normalized power for the given accel.
     * Solved from: |kS + kV·v + kA·a| ≤ V_batt (approx using kV·v dominance).
     */
    public double velocityAtVoltageSaturation(double acceleration) {
        double available = batteryVoltage - kS - Math.abs(kA * acceleration);
        if (available <= 0) return 0;
        return available / Math.max(kV, 1e-6);
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

    public RobotModel batteryVoltage(double batteryVoltage) {
        this.batteryVoltage = batteryVoltage;
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

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
