package com.pedropathing.trajectory;

/**
 * Time-parameterized state along a trajectory. Distances in inches, angles in radians.
 */
public final class TrajectoryState {
    public final double x;
    public final double y;
    public final double heading;
    public final double velocity;
    public final double acceleration;
    public final double angularVelocity;
    public final double angularAcceleration;
    public final double time;
    /** Arc length along the path (inches). */
    public final double s;
    /** Path curvature κ (1/inches). */
    public final double curvature;
    /** Parametric t on the owning Pedro Path segment when applicable. */
    public final double t;
    /** Index of the Path inside a PathChain (0 if single Path). */
    public final int pathIndex;

    public TrajectoryState(
            double x, double y, double heading,
            double velocity, double acceleration,
            double angularVelocity, double angularAcceleration,
            double time, double s, double curvature,
            double t, int pathIndex
    ) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.angularVelocity = angularVelocity;
        this.angularAcceleration = angularAcceleration;
        this.time = time;
        this.s = s;
        this.curvature = curvature;
        this.t = t;
        this.pathIndex = pathIndex;
    }

    public TrajectoryState withDynamics(
            double velocity, double acceleration,
            double angularVelocity, double angularAcceleration,
            double time
    ) {
        return new TrajectoryState(
                x, y, heading,
                velocity, acceleration,
                angularVelocity, angularAcceleration,
                time, s, curvature, t, pathIndex
        );
    }
}
