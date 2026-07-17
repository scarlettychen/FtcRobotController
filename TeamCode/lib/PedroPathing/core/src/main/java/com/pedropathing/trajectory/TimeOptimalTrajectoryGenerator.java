package com.pedropathing.trajectory;

import com.pedropathing.model.MotionModel;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.List;

/**
 * Classic forward/backward pass time-optimal velocity profile along path arc length s.
 *
 * <p>Also the public generate entry (replaces the old TrajectoryFactory facade).
 * Complexity O(N) — run at path build time, not every loop.
 */
public class TimeOptimalTrajectoryGenerator {
    private final MotionModel model;
    private final PathAnalyzer analyzer;
    private boolean stopAtEnd = true;

    public TimeOptimalTrajectoryGenerator(MotionModel model) {
        this(model, new PathAnalyzer(0.75));
    }

    public TimeOptimalTrajectoryGenerator(MotionModel model, PathAnalyzer analyzer) {
        this.model = model;
        this.analyzer = analyzer;
    }

    public static Trajectory generate(Path path, MotionModel model) {
        return new TimeOptimalTrajectoryGenerator(model).generate(path);
    }

    public static Trajectory generate(PathChain chain, MotionModel model) {
        return new TimeOptimalTrajectoryGenerator(model).generate(chain);
    }

    public TimeOptimalTrajectoryGenerator stopAtEnd(boolean stopAtEnd) {
        this.stopAtEnd = stopAtEnd;
        return this;
    }

    public Trajectory generate(Path path) {
        return generate(analyzer.analyze(path));
    }

    public Trajectory generate(PathChain chain) {
        return generate(analyzer.analyze(chain));
    }

    public Trajectory generate(List<SampledPathPoint> samples) {
        int n = samples.size();
        if (n == 0) {
            throw new IllegalArgumentException("No path samples to profile");
        }

        double[] vMax = velocityLimits(samples);
        if (!stopAtEnd && n > 0) {
            SampledPathPoint last = samples.get(n - 1);
            double v = model.motorLimitedVelocity();
            if (Math.abs(last.curvature) > 1e-6) {
                v = Math.min(v, Math.sqrt(model.getMaxLateralAcceleration() / Math.abs(last.curvature)));
            }
            vMax[n - 1] = Math.max(v, 1e-3);
        }

        double[] v = new double[n];
        v[0] = 0;

        double aMax = model.profileMaxAcceleration();
        double aDec = model.profileMaxDeceleration();

        for (int i = 0; i < n - 1; i++) {
            double ds = samples.get(i + 1).s - samples.get(i).s;
            if (ds < 1e-9) {
                v[i + 1] = Math.min(v[i], vMax[i + 1]);
                continue;
            }
            double attainable = Math.sqrt(v[i] * v[i] + 2.0 * aMax * ds);
            v[i + 1] = Math.min(vMax[i + 1], attainable);

            double omega0 = samples.get(i).headingRate * v[i];
            double omega1Candidate = samples.get(i + 1).headingRate * v[i + 1];
            double dtEst = ds / Math.max(0.5 * (v[i] + v[i + 1]), 1e-3);
            if (dtEst > 1e-6) {
                double alphaNeeded = Math.abs(omega1Candidate - omega0) / dtEst;
                if (alphaNeeded > model.getMaxAngularAcceleration()
                        && Math.abs(samples.get(i + 1).headingRate) > 1e-6) {
                    double maxOmega = Math.abs(omega0) + model.getMaxAngularAcceleration() * dtEst;
                    v[i + 1] = Math.min(
                            v[i + 1], maxOmega / Math.abs(samples.get(i + 1).headingRate));
                }
            }
        }

        if (stopAtEnd) {
            v[n - 1] = 0;
        }
        for (int i = n - 2; i >= 0; i--) {
            double ds = samples.get(i + 1).s - samples.get(i).s;
            if (ds < 1e-9) {
                v[i] = Math.min(v[i], v[i + 1]);
                continue;
            }
            double attainable = Math.sqrt(v[i + 1] * v[i + 1] + 2.0 * aDec * ds);
            v[i] = Math.min(v[i], Math.min(vMax[i], attainable));
        }

        TrajectoryState[] states = new TrajectoryState[n];
        double time = 0;
        for (int i = 0; i < n; i++) {
            SampledPathPoint p = samples.get(i);

            // omega follows heading interpolation only. Do NOT use curvature*v:
            // that made constant-heading Bezier paths spin then unspin (κ changes sign)
            // while straight forwardDrive (κ≈0) looked fine.
            double omega = p.headingRate * v[i];

            double accel;
            if (i < n - 1) {
                double ds = samples.get(i + 1).s - p.s;
                accel = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * Math.max(ds, 1e-9));
            } else if (i > 0) {
                double ds = p.s - samples.get(i - 1).s;
                accel = (v[i] * v[i] - v[i - 1] * v[i - 1]) / (2.0 * Math.max(ds, 1e-9));
            } else {
                accel = 0;
            }
            accel = clamp(accel, -aDec, aMax);

            double alpha = 0;
            if (i < n - 1) {
                double omegaNext = samples.get(i + 1).headingRate * v[i + 1];
                double ds = samples.get(i + 1).s - p.s;
                double vAvg = 0.5 * (v[i] + v[i + 1]);
                double dt = ds / Math.max(vAvg, 1e-3);
                alpha = (omegaNext - omega) / Math.max(dt, 1e-6);
            }

            double pathTangent = p.heading;
            if (i < n - 1) {
                double dx = samples.get(i + 1).x - p.x;
                double dy = samples.get(i + 1).y - p.y;
                if (dx * dx + dy * dy > 1e-12) {
                    pathTangent = Math.atan2(dy, dx);
                }
            } else if (i > 0) {
                double dx = p.x - samples.get(i - 1).x;
                double dy = p.y - samples.get(i - 1).y;
                if (dx * dx + dy * dy > 1e-12) {
                    pathTangent = Math.atan2(dy, dx);
                }
            }

            states[i] = new TrajectoryState(
                    p.x, p.y, p.heading, pathTangent,
                    v[i], accel,
                    omega, alpha,
                    time, p.s, p.curvature,
                    p.t, p.pathIndex
            );

            if (i < n - 1) {
                double ds = samples.get(i + 1).s - p.s;
                double vAvg = 0.5 * (v[i] + v[i + 1]);
                time += ds / Math.max(vAvg, 1e-3);
            }
        }

        return new Trajectory(states);
    }

    /** Per-sample velocity ceiling from traction, motors, heading rate, and FF voltage. */
    private double[] velocityLimits(List<SampledPathPoint> samples) {
        int n = samples.size();
        double[] vMax = new double[n];
        double motorCap = model.motorLimitedVelocity();
        double ffCap = model.velocityAtVoltageSaturation(0);

        for (int i = 0; i < n; i++) {
            SampledPathPoint p = samples.get(i);
            double v = Math.min(motorCap, ffCap);

            double kappa = Math.abs(p.curvature);
            if (kappa > 1e-6) {
                // Holonomic: path curvature needs centripetal accel, not body yaw at κ·v.
                // (κ·v angular cap is for non-holonomic path-tangent following and made
                // constant-heading Bezier profiles artificially slow → ahead-of-schedule reverse.)
                v = Math.min(v, Math.sqrt(model.getMaxLateralAcceleration() / kappa));
            }

            // Cap speed from heading interpolation rate (turn-while-driving), not path κ.
            double headingRate = Math.abs(p.headingRate);
            if (headingRate > 1e-6) {
                v = Math.min(v, model.getMaxAngularVelocity() / headingRate);
            }

            vMax[i] = Math.max(v, 1e-3);
        }

        if (n > 0) {
            vMax[n - 1] = 0;
        }
        return vMax;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
