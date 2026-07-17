package com.pedropathing.ftc.localization.localizers;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.model.MotionModel;

/**
 * Simple 1D Kalman-style fuse of wheel/odo pose with Limelight AprilTag botpose.
 * Writes localization confidence to an attached {@link MotionModel}.
 */
public class FusedLocalizer implements Localizer {

    public static double ODO_VARIANCE = 0.05;
    public static double CAM_BASE_VARIANCE = 0.1;
    public static double CAM_DISTANCE_DECAY_RATE = 0.01;
    public static double MAX_VISION_DISTANCE = 72.0;

    private final Localizer odometry;
    private final AprilTagLocalizer vision;
    private final MotionModel motionModel;

    private Pose currentFusedPose;
    private double localizationConfidence = 1.0;

    public FusedLocalizer(Localizer odometry, AprilTagLocalizer vision) {
        this(odometry, vision, null);
    }

    public FusedLocalizer(Localizer odometry, AprilTagLocalizer vision, MotionModel motionModel) {
        this.odometry = odometry;
        this.vision = vision;
        this.motionModel = motionModel;
        this.currentFusedPose = odometry.getPose();
    }

    public double getLocalizationConfidence() {
        return localizationConfidence;
    }

    @Override
    public void setPose(Pose pose) {
        odometry.setPose(pose);
        currentFusedPose = pose;
    }

    @Override
    public Pose getPose() {
        return currentFusedPose;
    }

    @Override
    public void update() {
        odometry.update();
        Pose odoPose = odometry.getPose();

        AprilTagLocalizer.VisionMeasurement visionMeasurement = vision.getLatestMeasurement();

        if (visionMeasurement != null && visionMeasurement.distanceToTag <= MAX_VISION_DISTANCE) {
            double distanceSq = visionMeasurement.distanceToTag * visionMeasurement.distanceToTag;
            double camVariance = CAM_BASE_VARIANCE + (CAM_DISTANCE_DECAY_RATE * distanceSq);
            double kalmanGain = ODO_VARIANCE / (ODO_VARIANCE + camVariance);

            double fusedX = odoPose.getX()
                    + kalmanGain * (visionMeasurement.pose.getX() - odoPose.getX());
            double fusedY = odoPose.getY()
                    + kalmanGain * (visionMeasurement.pose.getY() - odoPose.getY());
            double headingError = MathFunctions.getSmallestAngleDifference(
                    visionMeasurement.pose.getHeading(), odoPose.getHeading())
                    * MathFunctions.getTurnDirection(
                            odoPose.getHeading(), visionMeasurement.pose.getHeading());
            double fusedHeading = odoPose.getHeading() + kalmanGain * headingError;

            currentFusedPose = new Pose(fusedX, fusedY, fusedHeading);
            odometry.setPose(currentFusedPose);

            // Higher gain (low cam variance) → higher confidence
            localizationConfidence = clamp(0.55 + 0.45 * (1.0 - Math.min(1.0, camVariance / 2.0)), 0.4, 1.0);
        } else {
            currentFusedPose = odoPose;
            localizationConfidence = clamp(localizationConfidence * 0.995, 0.55, 1.0);
        }

        if (motionModel != null) {
            motionModel.setLocalizationConfidence(localizationConfidence);
        }
    }

    @Override
    public Pose getVelocity() {
        return odometry.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return odometry.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odometry.setStartPose(setStart);
        currentFusedPose = setStart;
    }

    @Override
    public double getTotalHeading() {
        return odometry.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odometry.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odometry.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odometry.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        odometry.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return odometry.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentFusedPose.getX())
                || Double.isNaN(currentFusedPose.getY())
                || Double.isNaN(currentFusedPose.getHeading());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
