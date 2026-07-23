package com.pedropathing.trajectory;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.model.MotionModel;

/**
 * Predictive (feedforward + feedback) follower for a precomputed {@link Trajectory}.
 *
 * <p>Cruise: RobotModel FF along the time profile + gated closest-point cross-track.
 * <p>Settle (opt-in via {@link #follow(Trajectory, boolean)}): after the time profile, hold the
 * baked end (X, Y, heading) until tolerances stick. Leave settle off for mid-auton paths.
 */
public class PredictiveTrajectoryFollower {
    private final PoseTracker poseTracker;
    private final Drivetrain drivetrain;
    private final MotionModel model;

    private Trajectory trajectory;
    private long startNanos;
    private boolean running;
    private boolean finished;
    /** True once time profile is done and we are holding the end pose. */
    private boolean settling;
    private boolean settlingEnabledForThisPath;
    /** Nanos when settle tolerances were first continuously satisfied (0 = not yet). */
    private long settleOkSinceNanos;

    public double kP = 0.08;
    public double kD = 0.01;
    public double kPCross = 0.12;
    public double kPHeading = 0.8;
    public double kDHeading = 0.05;
    public double kVOmega = 0.15;
    public double kAAlpha = 0.02;

    /**
     * Closest-point search half-width behind the schedule (inches). Ahead width is tighter
     * ({@link #closestAheadInches}) so chord-cutting cannot snap progress toward the path end early.
     */
    public double closestWindowInches = 10.0;
    /** How far ahead of s*(t) closest-point may look (inches). Keep small to reduce chord-cutting. */
    public double closestAheadInches = 3.0;

    private double maxCrossPower = 0.45;
    /** Extra cross-track gain scale when near the end of the path (pull back onto the chord). */
    public double endCrossTrackBoost = 1.75;
    /** Remaining arc length (inches) under which end cross-track boost applies. */
    public double endCrossTrackDistance = 12.0;
    /** Minimum |corrective| when |eCross| exceeds this (inches) — overcomes static friction sideways. */
    public double crossMinPower = 0.18;
    public double crossMinErrorInches = 1.0;

    private double endPositionTolerance = 1.25;
    private double endVelocityTolerance = 2.0;
    private double endHeadingTolerance = Math.toRadians(8);
    /** How long (s) pose+vel must stay in tolerance before finishing settle. */
    public double settleHoldSeconds = 0.2;
    /** Give up settle after this many seconds (safety). */
    public double settleTimeoutSeconds = 1.5;
    /**
     * Only settle if within this distance of the baked end when the profile ends.
     * Prevents a long reverse chase if the end pose is wildly wrong.
     */
    public double settleMaxStartDistance = 18.0;

    private TrajectoryState lastSetpoint;
    private TrajectoryState lastClosest;
    private long settleStartNanos;

    public PredictiveTrajectoryFollower(PoseTracker poseTracker, Drivetrain drivetrain, MotionModel model) {
        this.poseTracker = poseTracker;
        this.drivetrain = drivetrain;
        this.model = model;
    }

    public void follow(Trajectory trajectory) {
        follow(trajectory, false);
    }

    /**
     * @param settleAtEnd if true, after the time profile hold end XY/heading until tolerances
     *                    (use on the last path of an auton). If false, stop when the profile ends
     *                    so mid-auton paths keep flowing.
     */
    public void follow(Trajectory trajectory, boolean settleAtEnd) {
        this.trajectory = trajectory;
        this.startNanos = System.nanoTime();
        this.running = true;
        this.finished = false;
        this.settling = false;
        this.settlingEnabledForThisPath = settleAtEnd;
        this.settleOkSinceNanos = 0;
        this.settleStartNanos = 0;
        this.lastSetpoint = trajectory.get(0);
        this.lastClosest = trajectory.get(0);
    }

    public boolean isBusy() {
        return running && !finished;
    }

    public boolean isFinished() {
        return finished;
    }

    public TrajectoryState getLastSetpoint() {
        return lastSetpoint;
    }

    /** Path completion from the time schedule (smooth 0→1). Settle still reports 1.0. */
    public double getPathCompletion() {
        if (trajectory == null) return finished ? 1 : 0;
        if (finished || settling) return 1;
        double len = trajectory.getTotalLength();
        if (len < 1e-6) return 0;
        if (lastSetpoint == null) return 0;
        return Math.max(0, Math.min(1, lastSetpoint.s / len));
    }

    public void cancel() {
        running = false;
        finished = true;
        settling = false;
        drivetrain.breakFollowing();
    }

    public void update() {
        update(true);
    }

    public void update(boolean updatePose) {
        if (!running || trajectory == null) return;

        if (updatePose) poseTracker.update();
        Pose pose = poseTracker.getPose();
        Vector velocity = poseTracker.getVelocity();

        if (drivetrain != null) {
            try {
                model.setBatteryVoltage(drivetrain.getVoltage());
            } catch (Exception ignored) {
                // some drivetrains may throw if unpowered in unit tests
            }
        }

        double t = (System.nanoTime() - startNanos) * 1e-9;
        TrajectoryState endState = trajectory.get(trajectory.size() - 1);

        // Profile done: settle only if this path opted in (typically last path of auton).
        if (t >= trajectory.getTotalTime()) {
            lastSetpoint = endState;
            lastClosest = endState;
            if (!settlingEnabledForThisPath) {
                // Mid-auton / normal paths: go with the flow — stop and hand off to the next command.
                finishNow();
                return;
            }
            if (!settling) {
                settling = true;
                settleStartNanos = System.nanoTime();
                settleOkSinceNanos = 0;
            }
        }

        if (settling) {
            updateSettle(pose, velocity, endState);
            return;
        }

        // --- Cruise: same structure as working commit 73f0394 ---
        TrajectoryState timeSetpoint = trajectory.sampleByTime(t);
        // Symmetric window like 73f0394 (ahead=behind) for forwardDrive stability.
        TrajectoryState closest = trajectory.findClosestNear(
                pose.getX(), pose.getY(), timeSetpoint.s, closestWindowInches);
        lastSetpoint = timeSetpoint;
        lastClosest = closest;

        double pathTheta = timeSetpoint.pathTangent;
        double cos = Math.cos(pathTheta);
        double sin = Math.sin(pathTheta);

        double ex = closest.x - pose.getX();
        double ey = closest.y - pose.getY();
        double eCross = -ex * sin + ey * cos;
        double eTheta = MathFunctions.normalizeAngleSigned(timeSetpoint.heading - pose.getHeading());
        double eLag = Math.max(0, timeSetpoint.s - closest.s);

        double vRef = timeSetpoint.velocity;
        double aRef = timeSetpoint.acceleration;
        double omegaRef = timeSetpoint.angularVelocity;
        double alphaRef = timeSetpoint.angularAcceleration;

        double vMeasAlong = velocity.getXComponent() * cos + velocity.getYComponent() * sin;
        double omegaMeas = poseTracker.getAngularVelocity();

        double ffDrive = model.feedforwardPower(vRef, aRef);
        double fbDrive = kP * eLag + kD * (vRef - vMeasAlong);
        double driveMag = clamp(ffDrive + fbDrive, 0.0, 1.0);

        double ffHeading = kVOmega * omegaRef + kAAlpha * alphaRef;
        double fbHeading = kPHeading * eTheta + kDHeading * (omegaRef - omegaMeas);
        double headingMag = clamp(ffHeading + fbHeading, -1.0, 1.0);

        double endPosErr = Math.hypot(endState.x - pose.getX(), endState.y - pose.getY());
        double remainingS = trajectory.getTotalLength() - timeSetpoint.s;
        if (endPosErr > 4.0) {
            headingMag = clamp(headingMag, -0.35, 0.35);
            if (driveMag < 0.2 && remainingS > 2.0 && vRef > 1.0) {
                driveMag = Math.max(driveMag, 0.2);
            }
        }
        if (Double.isNaN(driveMag)) driveMag = 0;
        if (Double.isNaN(headingMag)) headingMag = 0;

        // Cross-track (optional boosts kept for bezier/pathDrive; kPCross=0 disables).
        double crossGain = kPCross;
        double crossCap = maxCrossPower;
        if (remainingS < endCrossTrackDistance) {
            crossGain *= endCrossTrackBoost;
            crossCap = Math.min(0.7, maxCrossPower * endCrossTrackBoost);
        }
        double crossMag = clamp(crossGain * eCross, -crossCap, crossCap);
        if (Math.abs(eCross) > crossMinErrorInches && Math.abs(crossMag) < crossMinPower) {
            crossMag = Math.copySign(crossMinPower, eCross);
        }
        if (Double.isNaN(crossMag)) crossMag = 0;

        Vector pathing = new Vector(driveMag, pathTheta);
        Vector corrective = new Vector(crossMag, pathTheta + Math.PI / 2.0);
        Vector headingVec = new Vector(headingMag, pose.getHeading());
        drivetrain.runDrive(corrective, headingVec, pathing, pose.getHeading(), velocity);
    }

    /**
     * Hold the trajectory end pose. Drive direction is always toward end XY (never pathTangent),
     * so a flipped end tangent cannot reverse toward start. Finishes after tolerances hold.
     */
    private void updateSettle(Pose pose, Vector velocity, TrajectoryState end) {
        double endDx = end.x - pose.getX();
        double endDy = end.y - pose.getY();
        double endPosErr = Math.hypot(endDx, endDy);
        double eTheta = MathFunctions.normalizeAngleSigned(end.heading - pose.getHeading());
        double omegaMeas = poseTracker.getAngularVelocity();
        double settleElapsed = (System.nanoTime() - settleStartNanos) * 1e-9;

        // Too far from baked end when profile finished — stop rather than chase across the field.
        if (settleElapsed < 0.05 && endPosErr > settleMaxStartDistance) {
            finishNow();
            return;
        }

        double driveMag = 0;
        double pathTheta = pose.getHeading();
        double crossMag = 0;

        if (endPosErr >= endPositionTolerance) {
            // Translational PID toward end XY (along-track in the toward-end frame).
            pathTheta = Math.atan2(endDy, endDx);
            double vToward =
                    velocity.getXComponent() * Math.cos(pathTheta)
                            + velocity.getYComponent() * Math.sin(pathTheta);
            driveMag = clamp(kP * endPosErr - kD * vToward, 0.0, 1.0);
            if (driveMag < 0.2 && endPosErr > 2.0) {
                driveMag = 0.2; // min power to finish the last inches
            }
            // Also apply a small lateral component in field frame is unnecessary — pure
            // toward-end vector already corrects left/right off-chord error.
        }

        double headingMag = clamp(kPHeading * eTheta - kDHeading * omegaMeas, -0.5, 0.5);

        if (Double.isNaN(driveMag)) driveMag = 0;
        if (Double.isNaN(headingMag)) headingMag = 0;

        Vector pathing = new Vector(driveMag, pathTheta);
        Vector corrective = new Vector(crossMag, pathTheta + Math.PI / 2.0);
        Vector headingVec = new Vector(headingMag, pose.getHeading());
        drivetrain.runDrive(corrective, headingVec, pathing, pose.getHeading(), velocity);

        boolean posOk = endPosErr < endPositionTolerance;
        boolean velOk = velocity.getMagnitude() < endVelocityTolerance;
        boolean headOk = Math.abs(eTheta) < endHeadingTolerance;
        long now = System.nanoTime();
        if (posOk && velOk && headOk) {
            if (settleOkSinceNanos == 0) {
                settleOkSinceNanos = now;
            } else if ((now - settleOkSinceNanos) * 1e-9 >= settleHoldSeconds) {
                finishNow();
                return;
            }
        } else {
            settleOkSinceNanos = 0;
        }

        if (settleElapsed >= settleTimeoutSeconds) {
            finishNow();
        }
    }

    private void finishNow() {
        finished = true;
        running = false;
        settling = false;
        drivetrain.breakFollowing();
    }

    public PredictiveTrajectoryFollower setGains(double kP, double kD, double kPCross, double kPHeading, double kDHeading) {
        this.kP = kP;
        this.kD = kD;
        this.kPCross = kPCross;
        this.kPHeading = kPHeading;
        this.kDHeading = kDHeading;
        return this;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
