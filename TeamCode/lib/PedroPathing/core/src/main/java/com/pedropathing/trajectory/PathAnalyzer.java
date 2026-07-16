package com.pedropathing.trajectory;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * Samples a Pedro {@link Path} or {@link PathChain} by arc length and extracts
 * curvature + heading rate for constraint generation.
 *
 * Generation is intended to run offline (OpMode init / path build), not every loop.
 */
public class PathAnalyzer {
    private final double sampleSpacing;

    /**
     * @param sampleSpacing inches between samples along the path (0.5–1.0 recommended for FTC)
     */
    public PathAnalyzer(double sampleSpacing) {
        this.sampleSpacing = Math.max(sampleSpacing, 0.1);
    }

    public PathAnalyzer() {
        this(0.75);
    }

    public List<SampledPathPoint> analyze(Path path) {
        List<SampledPathPoint> samples = new ArrayList<>();
        double length = path.length();
        if (length < 1e-6) {
            Pose p = path.getPose(0);
            samples.add(new SampledPathPoint(
                    p.getX(), p.getY(), path.getHeadingGoal(0),
                    0, 0, 0, 0, 0));
            return samples;
        }

        int n = Math.max(2, (int) Math.ceil(length / sampleSpacing) + 1);
        double prevHeading = path.getHeadingGoal(0);

        for (int i = 0; i < n; i++) {
            double completion = i / (double) (n - 1);
            double t = path.getTFromPathCompletion(completion);
            t = MathFunctions.clamp(t, 0, 1);
            Pose pose = path.getPose(t);
            double heading = path.getHeadingGoal(t);
            double s = completion * length;
            double curvature = Math.abs(path.getCurvature(t));

            double headingRate = 0;
            if (i > 0) {
                double ds = s - samples.get(i - 1).s;
                if (ds > 1e-9) {
                    headingRate = MathFunctions.normalizeAngleSigned(heading - prevHeading) / ds;
                }
            }
            samples.add(new SampledPathPoint(
                    pose.getX(), pose.getY(), heading,
                    s, curvature, headingRate, t, 0));
            prevHeading = heading;
        }
        return samples;
    }

    public List<SampledPathPoint> analyze(PathChain chain) {
        List<SampledPathPoint> samples = new ArrayList<>();
        if (chain.size() == 0) return samples;

        double sOffset = 0;
        double prevHeading = Double.NaN;

        for (int pathIndex = 0; pathIndex < chain.size(); pathIndex++) {
            Path path = chain.getPath(pathIndex);
            double length = path.length();
            if (length < 1e-6) continue;

            int n = Math.max(2, (int) Math.ceil(length / sampleSpacing) + 1);
            // Skip duplicate joint sample when stitching segments (except first segment).
            int start = samples.isEmpty() ? 0 : 1;

            for (int i = start; i < n; i++) {
                double completion = i / (double) (n - 1);
                double t = path.getTFromPathCompletion(completion);
                t = MathFunctions.clamp(t, 0, 1);
                Pose pose = path.getPose(t);
                double heading = path.getHeadingGoal(t);
                double s = sOffset + completion * length;
                double curvature = Math.abs(path.getCurvature(t));

                double headingRate = 0;
                if (!Double.isNaN(prevHeading) && !samples.isEmpty()) {
                    double ds = s - samples.get(samples.size() - 1).s;
                    if (ds > 1e-9) {
                        headingRate = MathFunctions.normalizeAngleSigned(heading - prevHeading) / ds;
                    }
                }
                samples.add(new SampledPathPoint(
                        pose.getX(), pose.getY(), heading,
                        s, curvature, headingRate, t, pathIndex));
                prevHeading = heading;
            }
            sOffset += length;
        }

        if (samples.isEmpty()) {
            Path path = chain.getPath(0);
            Pose p = path.getPose(0);
            samples.add(new SampledPathPoint(
                    p.getX(), p.getY(), path.getHeadingGoal(0),
                    0, 0, 0, 0, 0));
        }
        return samples;
    }
}
