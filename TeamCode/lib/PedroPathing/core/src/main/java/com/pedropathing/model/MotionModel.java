package com.pedropathing.model;

/**
 * Physics contract used by Pedro's time-optimal trajectory generator and predictive follower.
 *
 * <p>The concrete, team-tuned robot model belongs in TeamCode. Pedro depends only on this
 * interface so teams can own mass, acceleration, feedforward, voltage, and motion-context values.
 */
public interface MotionModel {
    double motorLimitedVelocity();

    double getMaxLateralAcceleration();

    double profileMaxAcceleration();

    double profileMaxDeceleration();

    double getMaxAngularVelocity();

    double getMaxAngularAcceleration();

    double feedforwardPower(double velocity, double acceleration);

    double velocityAtVoltageSaturation(double acceleration);

    void setBatteryVoltage(double batteryVoltage);

    void setLocalizationConfidence(double localizationConfidence);

    void cruise();

    void loaded();

    void precision();
}
