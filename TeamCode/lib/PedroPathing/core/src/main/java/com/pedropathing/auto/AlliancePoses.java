package com.pedropathing.auto;

import com.pedropathing.geometry.Pose;

/**
 * Alliance field coordinates as {@code double[]{x, y, headingDegrees}} —
 * same shape your TeamCode autos already use with {@code createPose(...)}.
 *
 * <p>Numbers are interpreted via {@link Pose#fromField} (FTC field after
 * {@code PoseConverter.useFTCCoordinates()}) and converted to Pedro for following.
 *
 * <p>Keep <b>two subclasses</b>: one Blue, one Red. Inject both into
 * {@link ActionLibrary} and switch with {@link ActionLibrary#setAlliance(boolean)}
 * at OpMode start.
 *
 * <pre>
 * ActionLibrary bot = new RobotActions(drive, bluePoses, redPoses);
 * bot.setAlliance(isRed);
 * </pre>
 */
public abstract class AlliancePoses {

    public double[] start = xyz(0, 0, 0);

    public double[] lookAtOb = xyz(0, 0, 0);
    public double[] openGatePos = xyz(0, 0, 0);
    public double[] limelight = xyz(0, 0, 0);

    public double[] close1Shooting = xyz(0, 0, 0);

    public double[] collect1Pre = xyz(0, 0, 0);
    public double[] collect1Mid = xyz(0, 0, 0);
    public double[] firstSpikeEnd = xyz(0, 0, 0);
    public double[] strafePos = xyz(0, 0, 0);

    public double[] collect2Mid = xyz(0, 0, 0);
    public double[] collect2Pre = xyz(0, 0, 0);
    public double[] secondSpikeEnd = xyz(0, 0, 0);

    public double[] collect3Pre = xyz(0, 0, 0);
    public double[] collect3PrePass = xyz(0, 0, 0);
    public double[] thirdSpikeEnd = xyz(0, 0, 0);

    /** Convenience ctor helper: {@code {x, y, headingDegrees}}. */
    public static double[] xyz(double x, double y, double headingDegrees) {
        return new double[]{x, y, headingDegrees};
    }

    /** Matches TeamCode {@code createPose(double[])} — field coords → Pedro via {@link Pose#fromField}. */
    public static Pose toPose(double[] p) {
        if (p == null || p.length < 2) {
            throw new IllegalArgumentException("Pose array needs at least x,y");
        }
        double headingDeg = p.length >= 3 ? p[2] : 0;
        return Pose.fromFieldDegrees(p[0], p[1], headingDeg);
    }

    public Pose startPose() {
        return toPose(start);
    }
}
