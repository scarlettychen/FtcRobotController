package org.firstinspires.ftc.teamcode.brainstem.follower;

/**
 * Immutable snapshot of what the follower wants this tick, in library-neutral units.
 * Returned by {@link PathFollower#update(double, double, double)}.
 */
public final class FollowerOutput {

    public final double correctiveX;
    public final double correctiveY;
    public final double headingPower;
    public final double pathCompletion;
    public final double crossTrackError;
    public final double curvature;

    /** Target cruise speed along path this tick (in/s). */
    public final double velocityReference;
    /** Target tangential acceleration this tick (in/s²). */
    public final double accelerationReference;
    /** Active velocity ceiling from {@link VelocityConstraint} (in/s). */
    public final double velocityLimit;
    /** Why {@link #velocityLimit} was chosen (e.g. CURVATURE, SEGMENT_CAP). */
    public final String curvatureLimitReason;

    public FollowerOutput(
            double correctiveX,
            double correctiveY,
            double headingPower,
            double pathCompletion,
            double crossTrackError,
            double curvature,
            double velocityReference,
            double accelerationReference,
            double velocityLimit,
            String curvatureLimitReason) {
        this.correctiveX = correctiveX;
        this.correctiveY = correctiveY;
        this.headingPower = headingPower;
        this.pathCompletion = pathCompletion;
        this.crossTrackError = crossTrackError;
        this.curvature = curvature;
        this.velocityReference = velocityReference;
        this.accelerationReference = accelerationReference;
        this.velocityLimit = velocityLimit;
        this.curvatureLimitReason =
                curvatureLimitReason == null ? VelocityConstraint.LimitReason.NONE.name() : curvatureLimitReason;
    }

    /** Backward-compatible ctor — velocity fields zero / NONE. */
    public FollowerOutput(
            double correctiveX,
            double correctiveY,
            double headingPower,
            double pathCompletion,
            double crossTrackError,
            double curvature) {
        this(correctiveX, correctiveY, headingPower, pathCompletion, crossTrackError, curvature,
                0, 0, 0, VelocityConstraint.LimitReason.NONE.name());
    }

    public static final FollowerOutput NONE =
            new FollowerOutput(0, 0, 0, 0, 0, 0, 0, 0, 0, VelocityConstraint.LimitReason.NONE.name());
}
