package com.pedropathing.trajectory;

/**
 * Immutable time-optimal trajectory: a dense array of {@link TrajectoryState} sorted by time/s.
 */
public class Trajectory {
    private final TrajectoryState[] states;
    private final double totalTime;
    private final double totalLength;

    public Trajectory(TrajectoryState[] states) {
        if (states == null || states.length == 0) {
            throw new IllegalArgumentException("Trajectory requires at least one state");
        }
        this.states = states;
        this.totalTime = states[states.length - 1].time;
        this.totalLength = states[states.length - 1].s;
    }

    public int size() {
        return states.length;
    }

    public TrajectoryState get(int index) {
        return states[index];
    }

    public TrajectoryState[] getStates() {
        return states;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public double getTotalLength() {
        return totalLength;
    }

    /** Binary search sample by trajectory time (seconds). */
    public TrajectoryState sampleByTime(double time) {
        if (time <= 0) return states[0];
        if (time >= totalTime) return states[states.length - 1];

        int lo = 0, hi = states.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (states[mid].time <= time) lo = mid;
            else hi = mid;
        }
        return interpolate(states[lo], states[hi],
                (time - states[lo].time) / Math.max(states[hi].time - states[lo].time, 1e-9));
    }

    /** Binary search sample by arc length (inches). */
    public TrajectoryState sampleByDistance(double s) {
        if (s <= 0) return states[0];
        if (s >= totalLength) return states[states.length - 1];

        int lo = 0, hi = states.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (states[mid].s <= s) lo = mid;
            else hi = mid;
        }
        return interpolate(states[lo], states[hi],
                (s - states[lo].s) / Math.max(states[hi].s - states[lo].s, 1e-9));
    }

    private static TrajectoryState interpolate(TrajectoryState a, TrajectoryState b, double alpha) {
        alpha = Math.max(0, Math.min(1, alpha));
        double heading = a.heading + wrap(b.heading - a.heading) * alpha;
        return new TrajectoryState(
                lerp(a.x, b.x, alpha),
                lerp(a.y, b.y, alpha),
                heading,
                lerp(a.velocity, b.velocity, alpha),
                lerp(a.acceleration, b.acceleration, alpha),
                lerp(a.angularVelocity, b.angularVelocity, alpha),
                lerp(a.angularAcceleration, b.angularAcceleration, alpha),
                lerp(a.time, b.time, alpha),
                lerp(a.s, b.s, alpha),
                lerp(a.curvature, b.curvature, alpha),
                lerp(a.t, b.t, alpha),
                alpha < 0.5 ? a.pathIndex : b.pathIndex
        );
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
