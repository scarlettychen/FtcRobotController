package com.pedropathing.trajectory;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.model.RobotModel;

/**
 * Predictive (feedforward + feedback) follower for a precomputed {@link Trajectory}.
 *
 * <pre>
 * drivePower ≈ RobotModel.feedforward(v*, a*) + Kp·e_along + Kd·(v* - v_meas)
 * headingPower ≈ kVω·ω* + kAα·α* + Kp_θ·e_θ + Kd_θ·(ω* - ω_meas)
 * </pre>
 *
 * Uses Pedro {@link PoseTracker} / {@link Drivetrain}; does not replace geometric Path storage.
 * Call {@link #update()} once per OpMode loop (~50–80 Hz). Sampling is O(log N).
 */
public class PredictiveTrajectoryFollower {
    private final PoseTracker poseTracker;
    private final Drivetrain drivetrain;
    private final RobotModel model;

    private Trajectory trajectory;
    private long startNanos;
    private boolean running;
    private boolean finished;

    // Translational feedback (power / inch, power / (in/s))
    public double kP = 0.08;
    public double kD = 0.01;
    // Cross-track (lateral) feedback
    public double kPCross = 0.12;
    // Heading feedback
    public double kPHeading = 1.5;
    public double kDHeading = 0.05;
    public double kVOmega = 0.15;
    public double kAAlpha = 0.02;

    private TrajectoryState lastSetpoint;
    private double endVelocityTolerance = 2.0;
    private double endPositionTolerance = 1.5;
    private double endHeadingTolerance = Math.toRadians(3);

    public PredictiveTrajectoryFollower(PoseTracker poseTracker, Drivetrain drivetrain, RobotModel model) {
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

    /** Path completion in [0, 1] from trajectory arc length (0 if idle). */
    public double getPathCompletion() {
        if (trajectory == null || lastSetpoint == null) return 0;
        double len = trajectory.getTotalLength();
        if (len < 1e-6) return finished ? 1 : 0;
        return Math.max(0, Math.min(1, lastSetpoint.s / len));
    }

    public void cancel() {
        running = false;
        finished = true;
        drivetrain.breakFollowing();
    }

    /**
     * Advance time, sample trajectory, apply FF + FB, write wheel powers.
     * Updates localization first — use {@link #update(boolean)} if the pose was already refreshed this loop.
     */
    public void update() {
        update(true);
    }

    /**
     * @param updatePose if false, skips {@link PoseTracker#update()} (Pedro {@code Follower} already did)
     */
    public void update(boolean updatePose) {
        if (!running || trajectory == null) return;

        if (updatePose) poseTracker.update();
        Pose pose = poseTracker.getPose();
        Vector velocity = poseTracker.getVelocity();

        // Optionally refresh battery for voltage-scaled FF.
        if (drivetrain != null) {
            try {
                model.batteryVoltage = drivetrain.getVoltage();
            } catch (Exception ignored) {
                // some drivetrains may throw if unpowered in unit tests
            }
        }

        double t = (System.nanoTime() - startNanos) * 1e-9;
        TrajectoryState setpoint = trajectory.sampleByTime(t);
        lastSetpoint = setpoint;

        // Field-relative errors
        double ex = setpoint.x - pose.getX();
        double ey = setpoint.y - pose.getY();
        double eTheta = MathFunctions.normalizeAngleSigned(setpoint.heading - pose.getHeading());

        // Project pose error into path-aligned (along / cross) using trajectory heading.
        double cos = Math.cos(setpoint.heading);
        double sin = Math.sin(setpoint.heading);
        double eAlong = ex * cos + ey * sin;
        double eCross = -ex * sin + ey * cos;

        double vMeasAlong = velocity.getXComponent() * cos + velocity.getYComponent() * sin;
        double omegaMeas = poseTracker.getAngularVelocity();

        double ffDrive = model.feedforwardPower(setpoint.velocity, setpoint.acceleration);
        double fbDrive = kP * eAlong + kD * (setpoint.velocity - vMeasAlong);
        double driveMag = clamp(ffDrive + fbDrive, -1.0, 1.0);

        double ffHeading = kVOmega * setpoint.angularVelocity + kAAlpha * setpoint.angularAcceleration;
        double fbHeading = kPHeading * eTheta + kDHeading * (setpoint.angularVelocity - omegaMeas);
        double headingMag = clamp(ffHeading + fbHeading, -1.0, 1.0);

        // Pathing vector along desired heading; cross-track as left-normal corrective.
        Vector pathing = new Vector(driveMag, setpoint.heading);
        Vector corrective = new Vector(clamp(kPCross * eCross, -1.0, 1.0), setpoint.heading + Math.PI / 2.0);
        Vector headingVec = new Vector(headingMag, pose.getHeading());

        drivetrain.runDrive(corrective, headingVec, pathing, pose.getHeading(), velocity);

        if (t >= trajectory.getTotalTime()) {
            checkFinished(pose, velocity);
        }
    }

    private void checkFinished(Pose pose, Vector velocity) {
        TrajectoryState end = trajectory.get(trajectory.size() - 1);
        double posErr = Math.hypot(end.x - pose.getX(), end.y - pose.getY());
        double headErr = MathFunctions.getSmallestAngleDifference(end.heading, pose.getHeading());
        if (posErr < endPositionTolerance
                && headErr < endHeadingTolerance
                && velocity.getMagnitude() < endVelocityTolerance) {
            finished = true;
            running = false;
            drivetrain.breakFollowing();
        }
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
