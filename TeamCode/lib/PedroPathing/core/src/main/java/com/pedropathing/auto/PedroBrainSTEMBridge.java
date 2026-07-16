package com.pedropathing.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.pedropathing.util.Component;

/**
 * Pedro side of the BrainSTEM loop — pose feed + idle follower ticks.
 * Does <b>not</b> own or start an {@link AutoMode}; the OpMode constructs and runs the auton.
 *
 * <pre>
 * 1. update fused localizer
 * 2. syncPoseFromRobot(...)
 * 3. bridge.update()   // via TeamCode robot.update()
 * 4. auto.update()     // OpMode owns this while auton is active
 * 5. telemetry.update()
 * </pre>
 */
public class PedroBrainSTEMBridge implements Component {
    private final Follower follower;
    private final ExternalPoseLocalizer localizer;
    private boolean attachedToRobotLoop = true;

    public PedroBrainSTEMBridge(Follower follower, ExternalPoseLocalizer localizer) {
        this.follower = follower;
        this.localizer = localizer;
    }

    public Follower getFollower() {
        return follower;
    }

    public ExternalPoseLocalizer getPoseFeed() {
        return localizer;
    }

    /**
     * When true (default), BrainSTEMRobot syncs pose externally and drive commands
     * still call {@code follower.update()} while following — idle bridge ticks skip
     * a redundant follower update.
     */
    public void setAttachedToRobotLoop(boolean attached) {
        this.attachedToRobotLoop = attached;
    }

    public boolean isAttachedToRobotLoop() {
        return attachedToRobotLoop;
    }

    /**
     * Push BrainSTEM fused pose/velocity into Pedro (inches / radians / in/s / rad/s).
     * Call every loop after your localizer.update().
     */
    public void syncPoseFromRobot(double x, double y, double headingRad,
                                  double vx, double vy, double omega) {
        localizer.setState(x, y, headingRad, vx, vy, omega);
    }

    /**
     * Same as {@link #syncPoseFromRobot(double, double, double, double, double, double)}
     * + feeds localization confidence into the time-optimal profiler.
     */
    public void syncPoseFromRobot(double x, double y, double headingRad,
                                  double vx, double vy, double omega,
                                  double localizationConfidence) {
        syncPoseFromRobot(x, y, headingRad, vx, vy, omega);
        follower.getRobotModel().localizationConfidence = localizationConfidence;
    }

    /** Convenience when you already pack values like BrainSTEMRobot pose/vel arrays. */
    public void syncPoseFromRobot(double[] posVel) {
        if (posVel == null || posVel.length < 6) return;
        syncPoseFromRobot(posVel[0], posVel[1], posVel[2], posVel[3], posVel[4], posVel[5]);
    }

    @Override
    public void reset() {
        follower.breakFollowing();
    }

    @Override
    public void update() {
        // Path commands call follower.update() while busy. Idle: only tick when
        // Pedro owns the loop (not hosted inside BrainSTEMRobot).
        if (!attachedToRobotLoop && follower != null) {
            follower.update();
        }
    }

    @Override
    public String test() {
        return "PedroBrainSTEMBridge";
    }
}
