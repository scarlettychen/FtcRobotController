package org.firstinspires.ftc.teamcode.brainstem.follower;

/**
 * Stable contract that autos, telemetry, and any future UI planner should depend on.
 * Nothing outside the follower/ package (plus robot construction) should import Pedro.
 */
public interface PathFollower {

    /** Begin following a path. {@code spec} is library-neutral. */
    void startPath(PathSpec spec);

    /** Call every loop with the robot's current field pose (inches, degrees). */
    FollowerOutput update(double poseX, double poseY, double poseHeadingDegrees);

    /** Convenience: read {@link #getFieldPose()} then {@link #update(double, double, double)}. */
    default FollowerOutput update() {
        double[] p = getFieldPose();
        return update(p[0], p[1], p[2]);
    }

    /** True once the current path is complete and the follower has settled. */
    boolean isFinished();

    /** True while a path (or manual chase) is active. */
    default boolean isBusy() {
        return !isFinished();
    }

    /** Stop following / manual drive. */
    void cancel();

    /** Current field pose {@code {x, y, headingDeg}} in FieldCoords. */
    double[] getFieldPose();

    /** Enter robot-centric manual drive (limelight chase, etc.). */
    void startManualDrive();

    /**
     * Robot-centric powers: forward +, strafe + = left (Pedro teleop convention), turn +.
     * Call {@link #update()} each loop after this.
     */
    void setManualDrive(double forward, double strafe, double turn);
}
