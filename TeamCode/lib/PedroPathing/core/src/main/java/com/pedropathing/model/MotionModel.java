package com.pedropathing.model;

/**
 * Physics contract for TeamCode {@code RobotModel}: velocity/accel limits and kS/kV/kA feedforward.
 * When set on {@link com.pedropathing.follower.Follower}, classic path following is model-led
 * (feedforward + light PID correction) so teams tune the model instead of Pedro PID gains.
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
