package com.pedropathing.trajectory;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.model.MotionModel;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Predictive (feedforward + feedback) follower for a Pedro-generated motion profile.
 *
 * <p>Pedro {@link Path} / {@link PathChain} are profiled once via
 * {@link TimeOptimalTrajectoryGenerator}, then each tick the reference
 * (pose, heading, v, a, ω, α, tangent) is sampled from that profile at the
 * robot's current path progress — not a single static end pose, and not
 * open-loop time.
 *
 * <p>Cruise: RobotModel FF along the profile + gated closest-point cross-track.
 * Settle (opt-in): after path progress reaches the end, hold end XY/heading.
 */
public class PredictiveTrajectoryFollower {
    private final PoseTracker poseTracker;
    private final Drivetrain drivetrain;
    private final MotionModel model;

    private Trajectory trajectory;
    private long startNanos;
    private boolean running;
    private boolean finished;
    /** True once path progress is at the end and we are holding the end pose. */
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
     * Closest-point search half-width behind the robot's last progress (inches).
     * Ahead width is {@link #closestAheadInches}.
     */
    public double closestWindowInches = 10.0;
    /** How far ahead of last progress closest-point may look (inches). */
    public double closestAheadInches = 3.0;
    /**
     * Sample the profile this far ahead of closest-point s (inches) for v/a/heading FF.
     * Keeps a small along-track lag term without open-loop time.
     */
    public double pathLookaheadInches = 3.0;
    /** Treat path as complete when closest s is within this of total length (inches). */
    public double endCompletionInches = 1.0;
    /** Safety: abandon cruise if wall-clock exceeds profile duration by this factor. */
    public double timeSafetyFactor = 2.0;

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

    // Debug: last centripetal / lateral FF tick (for OpMode telemetry)
    private double lastVelocityForLat;
    private double lastCurvature;
    private double lastALatUnclamped;
    private double lastALatClamped;
    private boolean lastALatWasClamped;
    private double lastLatPower;

    public PredictiveTrajectoryFollower(PoseTracker poseTracker, Drivetrain drivetrain, MotionModel model) {
        this.poseTracker = poseTracker;
        this.drivetrain = drivetrain;
        this.model = model;
    }

    public void follow(Trajectory trajectory) {
        follow(trajectory, false);
    }

    /**
     * @param settleAtEnd if true, after path progress reaches the end hold end XY/heading
     *                    until tolerances (use on the last path of an auton). If false, stop
     *                    when progress completes so mid-auton paths keep flowing.
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

    /**
     * Profile a Pedro {@link Path} with {@link TimeOptimalTrajectoryGenerator}, then follow.
     * Does not use Pedro's geometric {@code Follower} cruise — only path/profile generation.
     */
    public void follow(Path path) {
        follow(path, false);
    }

    public void follow(Path path, boolean settleAtEnd) {
        follow(TimeOptimalTrajectoryGenerator.generate(path, model), settleAtEnd);
    }

    /**
     * Profile a Pedro {@link PathChain} with {@link TimeOptimalTrajectoryGenerator}, then follow.
     */
    public void follow(PathChain chain) {
        follow(chain, false);
    }

    public void follow(PathChain chain, boolean settleAtEnd) {
        follow(TimeOptimalTrajectoryGenerator.generate(chain, model), settleAtEnd);
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

    /** Path completion from closest-point progress (smooth 0→1). Settle still reports 1.0. */
    public double getPathCompletion() {
        if (trajectory == null) return finished ? 1 : 0;
        if (finished || settling) return 1;
        double len = trajectory.getTotalLength();
        if (len < 1e-6) return 0;
        if (lastClosest == null) return 0;
        return Math.max(0, Math.min(1, lastClosest.s / len));
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

        TrajectoryState endState = trajectory.get(trajectory.size() - 1);
        double totalLength = trajectory.getTotalLength();
        double wallT = (System.nanoTime() - startNanos) * 1e-9;

        // Closest point on the Pedro-generated profile near last progress
        double sHint = lastClosest != null ? lastClosest.s : 0;
        TrajectoryState closest = trajectory.findClosestNear(
                pose.getX(), pose.getY(), sHint, closestWindowInches, closestAheadInches);

        // Live reference at robot progress (+ small lookahead) — replaces static end / open-loop time
        double sRef = Math.min(closest.s + pathLookaheadInches, totalLength);
        TrajectoryState ref = trajectory.sampleByDistance(sRef);
        lastClosest = closest;
        lastSetpoint = ref;

        boolean pathComplete = closest.s >= totalLength - endCompletionInches
                || wallT >= trajectory.getTotalTime() * timeSafetyFactor;

        if (pathComplete) {
            lastSetpoint = endState;
            lastClosest = endState;
            if (!settlingEnabledForThisPath) {
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

        // --- Cruise: FF/PID unchanged; refs come from progress-sampled profile ---
        double pathTheta = ref.pathTangent;
        double cos = Math.cos(pathTheta);
        double sin = Math.sin(pathTheta);

        double ex = closest.x - pose.getX();
        double ey = closest.y - pose.getY();
        double eCross = -ex * sin + ey * cos;
        double eTheta = MathFunctions.normalizeAngleSigned(ref.heading - pose.getHeading());
        double eLag = Math.max(0, ref.s - closest.s);

        double vRef = ref.velocity;
        double aRef = ref.acceleration;
        double omegaRef = ref.angularVelocity;
        double alphaRef = ref.angularAcceleration;

        double vMeasAlong = velocity.getXComponent() * cos + velocity.getYComponent() * sin;
        double omegaMeas = poseTracker.getAngularVelocity();

        double ffDrive = model.feedforwardPower(vRef, aRef);
        double fbDrive = kP * eLag + kD * (vRef - vMeasAlong);
        double driveMag = clamp(ffDrive + fbDrive, 0.0, 1.0);

        double ffHeading = kVOmega * omegaRef + kAAlpha * alphaRef;
        double fbHeading = kPHeading * eTheta + kDHeading * (omegaRef - omegaMeas);
        double headingMag = clamp(ffHeading + fbHeading, -1.0, 1.0);

        double endPosErr = Math.hypot(endState.x - pose.getX(), endState.y - pose.getY());
        double remainingS = totalLength - closest.s;
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

        // Lateral (centripetal) FF: a = v²κ, clamp to model max, map via feedforwardPower → power
        // Same field-frame normal as cross-track: pathTheta + π/2 (signed by turn direction).
        double vForLat = Math.abs(vMeasAlong);
        if (vForLat < 1.0) {
            vForLat = Math.abs(vRef);
        }
        // PathAnalyzer stores |κ| (1/inches); v in inches/s → a in inches/s² (matches RobotModel)
        double kappa = Math.abs(ref.curvature);
        double aLatUnclamped = vForLat * vForLat * kappa;
        double aLatMax = model.getMaxLateralAcceleration();
        double aLatClampedMag = Math.min(aLatUnclamped, aLatMax);
        boolean aLatWasClamped = aLatUnclamped > aLatMax + 1e-6;
        // Sign from path tangent turning along the profile (works for constant-heading holonomic
        // curves where omegaRef≈0). Fall back to scheduled yaw rate.
        double latSign = 0;
        TrajectoryState aheadForSign = trajectory.sampleByDistance(
                Math.min(ref.s + 0.75, totalLength));
        double dTangent = MathFunctions.normalizeAngleSigned(
                aheadForSign.pathTangent - ref.pathTangent);
        if (Math.abs(dTangent) > 1e-4) {
            latSign = Math.signum(dTangent);
        } else {
            latSign = Math.signum(omegaRef);
            if (Math.abs(latSign) < 1e-4) {
                latSign = Math.signum(alphaRef);
            }
        }
        if (Math.abs(latSign) < 1e-4 || kappa < 1e-8 || aLatClampedMag < 1e-6) {
            latSign = 0;
        }
        double aLatSigned = aLatClampedMag * latSign;
        // feedforwardPower(v,a): v in/s, a in/s² → normalized power [-1,1] via kS/kV/kA
        double latPower = (latSign == 0) ? 0.0 : model.feedforwardPower(0.0, aLatSigned);

        lastVelocityForLat = vForLat;
        lastCurvature = kappa;
        lastALatUnclamped = aLatUnclamped;
        lastALatClamped = aLatClampedMag;
        lastALatWasClamped = aLatWasClamped;
        lastLatPower = latPower;

        // Cross-track + centripetal share the path-normal axis (additive in field frame)
        double normalTheta = pathTheta + Math.PI / 2.0;
        Vector crossVec = new Vector(crossMag, normalTheta);
        Vector latVec = new Vector(latPower, normalTheta);
        Vector corrective = crossVec.plus(latVec);

        Vector pathing = new Vector(driveMag, pathTheta);
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

    public double getLastVelocityForLat() {
        return lastVelocityForLat;
    }

    public double getLastCurvature() {
        return lastCurvature;
    }

    /** Unclamped |a_lat| = v²κ (inches/s²). */
    public double getLastALatUnclamped() {
        return lastALatUnclamped;
    }

    /** Clamped |a_lat| after getMaxLateralAcceleration() (inches/s²). */
    public double getLastALatClamped() {
        return lastALatClamped;
    }

    public boolean wasLastALatClamped() {
        return lastALatWasClamped;
    }

    /** Power sent on path-normal after feedforwardPower(0, a_lat) ([-1,1]). */
    public double getLastLatPower() {
        return lastLatPower;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
