package com.pedropathing.localization;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * Localizer that does not own sensors. TeamCode (BrainSTEM / fused localizer) pushes
 * pose/velocity each loop via {@link #setState}; Pedro's Follower reads it.
 *
 * <p>Typical OpMode loop:
 * <pre>
 * lastVel = drive.localizer.update();
 * lastPose = drive.localizer.getPose();
 * robot.syncPose(
 *     lastPose.position.x, lastPose.position.y, lastPose.heading.toDouble(),
 *     lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel);
 * robot.update();
 * auto.update(); // OpMode owns the auton
 * telemetry.update();
 * </pre>
 */
public class ExternalPoseLocalizer implements Localizer {
    private Pose pose = new Pose();
    private Pose velocity = new Pose(); // x=vx, y=vy, heading=omega
    private Pose startPose = new Pose();
    private double totalHeading = 0;
    private double lastHeading = 0;
    private boolean hasHeadingSample = false;

    /**
     * Push the latest fused estimate (inches, radians, in/s, rad/s).
     */
    public void setState(double x, double y, double headingRad,
                         double vx, double vy, double omega) {
        if (hasHeadingSample) {
            totalHeading += wrapDelta(headingRad - lastHeading);
        }
        hasHeadingSample = true;
        lastHeading = headingRad;
        pose = Pose.fromField(x, y, headingRad);
        velocity = new Pose(vx, vy, omega);
    }

    /** Pose-only push (zeros velocity). */
    public void setPoseState(double x, double y, double headingRad) {
        setState(x, y, headingRad, 0, 0, 0);
    }

    /**
     * Push a pose already in Pedro coordinates (e.g. from PinpointLocalizer).
     * Prefer {@link #setState} when the source is FTC field coordinates.
     */
    public void setPedroState(double x, double y, double headingRad,
                              double vx, double vy, double omega) {
        if (hasHeadingSample) {
            totalHeading += wrapDelta(headingRad - lastHeading);
        }
        hasHeadingSample = true;
        lastHeading = headingRad;
        pose = new Pose(x, y, headingRad);
        velocity = new Pose(vx, vy, omega);
    }

    @Override
    public Pose getPose() {
        return pose.copy();
    }

    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    @Override
    public Vector getVelocityVector() {
        return velocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        Pose pedro = setStart.getAsCoordinateSystem(com.pedropathing.geometry.PedroCoordinates.INSTANCE);
        this.startPose = pedro.copy();
        this.pose = pedro.copy();
        this.lastHeading = pedro.getHeading();
        this.totalHeading = 0;
        this.hasHeadingSample = true;
    }

    @Override
    public void setPose(Pose setPose) {
        Pose pedro = setPose.getAsCoordinateSystem(com.pedropathing.geometry.PedroCoordinates.INSTANCE);
        this.pose = pedro.copy();
        this.lastHeading = pedro.getHeading();
        this.hasHeadingSample = true;
    }

    @Override
    public void update() {
        // BrainSTEM already updated sensors; nothing to read here.
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return 1;
    }

    @Override
    public double getLateralMultiplier() {
        return 1;
    }

    @Override
    public double getTurningMultiplier() {
        return 1;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        // no IMU owned
    }

    @Override
    public double getIMUHeading() {
        return pose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }

    private static double wrapDelta(double delta) {
        while (delta > Math.PI) delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;
        return delta;
    }
}
