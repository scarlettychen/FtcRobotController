package com.pedropathing.trajectory;

/**
 * Geometry-only sample produced by {@link PathAnalyzer} before velocity profiling.
 */
public final class SampledPathPoint {
    public final double x;
    public final double y;
    public final double heading;
    public final double s;
    public final double curvature;
    public final double headingRate; // d(heading)/ds (rad / inch)
    public final double t;
    public final int pathIndex;

    public SampledPathPoint(
            double x, double y, double heading,
            double s, double curvature, double headingRate,
            double t, int pathIndex
    ) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.s = s;
        this.curvature = curvature;
        this.headingRate = headingRate;
        this.t = t;
        this.pathIndex = pathIndex;
    }
}
