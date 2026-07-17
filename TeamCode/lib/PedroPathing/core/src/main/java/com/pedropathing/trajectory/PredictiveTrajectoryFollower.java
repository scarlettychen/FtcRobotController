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
 * <p>Feedforward uses the time profile ({@link MotionModel} / RobotModel). Cross-track uses a
 * closest point gated to the schedule arc length. When the time profile ends, motors stop
 * immediately — no post-profile "drive to end XY" chase (that was reversing back to start
 * whenever the recorded end disagreed with where the robot actually was).
 */
public class PredictiveTrajectoryFollower {
    private final PoseTracker poseTracker;
    private final Drivetrain drivetrain;
    private final MotionModel model;

    private Trajectory trajectory;
    private long startNanos;
    private boolean running;
    private boolean finished;

    public double kP = 0.08;
    public double kD = 0.01;
    public double kPCross = 0.12;
    public double kPHeading = 0.8;
    public double kDHeading = 0.05;
    public double kVOmega = 0.15;
    public double kAAlpha = 0.02;

    /** Half-width (inches) for closest-point search around the time-schedule arc length. */
    public double closestWindowInches = 10.0;

    private TrajectoryState lastSetpoint;
    private TrajectoryState lastClosest;
    private double maxCrossPower = 0.55;

    public PredictiveTrajectoryFollower(PoseTracker poseTracker, Drivetrain drivetrain, MotionModel model) {
        this.poseTracker = poseTracker;
        this.drivetrain = drivetrain;
        this.model = model;
    }

    public void follow(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.startNanos = System.nanoTime();
        this.running = true;
        this.finished = false;
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

    /** Path completion from the time schedule (smooth 0→1). */
    public double getPathCompletion() {
        if (trajectory == null) return finished ? 1 : 0;
        double len = trajectory.getTotalLength();
        if (len < 1e-6) return finished ? 1 : 0;
        if (lastSetpoint == null) return finished ? 1 : 0;
        if (finished) return 1;
        return Math.max(0, Math.min(1, lastSetpoint.s / len));
    }

    public void cancel() {
        running = false;
        finished = true;
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

        // Time profile complete → stop. Do not chase end XY (caused reverse-to-start after pathDone=1).
        if (t >= trajectory.getTotalTime()) {
            lastSetpoint = trajectory.get(trajectory.size() - 1);
            lastClosest = lastSetpoint;
            finished = true;
            running = false;
            drivetrain.breakFollowing();
            return;
        }

        TrajectoryState timeSetpoint = trajectory.sampleByTime(t);
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

        double eSchedule = timeSetpoint.s - closest.s;

        double vRef = timeSetpoint.velocity;
        double aRef = timeSetpoint.acceleration;
        double omegaRef = timeSetpoint.angularVelocity;
        double alphaRef = timeSetpoint.angularAcceleration;

        if (eSchedule < 0) {
            TrajectoryState atProgress = trajectory.sampleByDistance(closest.s);
            vRef = Math.min(vRef, atProgress.velocity);
            aRef = atProgress.acceleration;
            eSchedule = 0;
        }

        double vMeasAlong = velocity.getXComponent() * cos + velocity.getYComponent() * sin;
        double omegaMeas = poseTracker.getAngularVelocity();

        double ffDrive = model.feedforwardPower(vRef, aRef);
        double fbDrive = kP * eSchedule + kD * (vRef - vMeasAlong);
        double driveMag = clamp(ffDrive + fbDrive, -1.0, 1.0);
        // Never reverse during the profile.
        if (driveMag < 0) {
            driveMag = 0;
        }

        double ffHeading = kVOmega * omegaRef + kAAlpha * alphaRef;
        double fbHeading = kPHeading * eTheta + kDHeading * (omegaRef - omegaMeas);
        double headingMag = clamp(ffHeading + fbHeading, -1.0, 1.0);

        TrajectoryState endState = trajectory.get(trajectory.size() - 1);
        double endPosErr = Math.hypot(endState.x - pose.getX(), endState.y - pose.getY());
        double remainingS = trajectory.getTotalLength() - timeSetpoint.s;
        if (endPosErr > 4.0) {
            headingMag = clamp(headingMag, -0.35, 0.35);
            if (driveMag >= 0 && driveMag < 0.25 && (eSchedule > 1.0 || remainingS > 2.0)) {
                driveMag = 0.25;
            }
        }
        if (Double.isNaN(driveMag)) driveMag = 0;
        if (Double.isNaN(headingMag)) headingMag = 0;

        Vector pathing = new Vector(driveMag, pathTheta);
        Vector corrective = new Vector(
                clamp(kPCross * eCross, -maxCrossPower, maxCrossPower),
                pathTheta + Math.PI / 2.0);
        Vector headingVec = new Vector(headingMag, pose.getHeading());

        drivetrain.runDrive(corrective, headingVec, pathing, pose.getHeading(), velocity);
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
