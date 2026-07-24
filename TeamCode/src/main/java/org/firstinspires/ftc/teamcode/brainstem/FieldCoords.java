package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.geometry.Pose;

/**
 * Team field coordinates for all pose arrays {@code {x, y, headingDeg}}.
 *
 * <pre>
 *   +Y (into field from −Y wall) = 0°
 *    ^
 *    |
 *    +----&gt; +X     0° faces +Y (from the near horizontal wall into the field)
 *                  90° faces −X
 *                  180° faces −Y
 *                  −90° / 270° faces +X
 *                  increases counter-clockwise
 *
 * Origin = field center. Walls at ±72 in.
 * Pedro internals use corner-origin via {@link RoadRunnerCoordinates} / FTCCoordinates.
 * Do not hand-write Pedro numbers in OpModes.
 * </pre>
 *
 * <p>At heading 0, driving forward must increase field Y (into the field).
 */
public final class FieldCoords {
    private FieldCoords() {}

    public static final double WALL = 72.0;

    /** {@code {x, y, headingDegrees}} */
    public static double[] xyz(double x, double y, double headingDeg) {
        return new double[]{x, y, headingDeg};
    }

    /**
     * Heading (deg) from one field pose toward another.
     * Team convention: 0° = +Y, increases CCW.
     */
    public static double headingToward(double fromX, double fromY, double toX, double toY) {
        // atan2 from +X, then −90° so 0° is +Y
        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX)) - 90.0;
    }

    public static double headingToward(double[] from, double[] to) {
        return headingToward(from[0], from[1], to[0], to[1]);
    }

    /**
     * Radians of the forward axis in standard field frame (0=+X, CCW) for
     * {@code cos}/{@code sin} body→field math. Team 0° (+Y) → +90° here.
     */
    public static double ccwRadians(double teamHeadingRad) {
        return teamHeadingRad + Math.PI / 2;
    }

    /** Field pose → string for telemetry. */
    public static String format(double[] p) {
        if (p == null || p.length < 2) {
            return "(?)";
        }
        double h = p.length >= 3 ? p[2] : 0;
        return String.format("(%.0f, %.0f, %.0f°)", p[0], p[1], h);
    }

    public static String format(Pose fieldPose) {
        return String.format("(%.1f, %.1f, %.0f°)",
                fieldPose.getX(),
                fieldPose.getY(),
                Math.toDegrees(fieldPose.getHeading()));
    }
}
