package com.pedropathing.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Component;

/**
 * Base class for english-like autons. Implements {@link Component}.
 *
 * <p>OpMode owns the auton — TeamCode constructs the robot + actions, then start/update/stop:
 * <pre>{@code
 * // TeamCode: BrainSTEMRobot + RobotActions (not in the Pedro library)
 * auto.setExternalLoop(true);
 * auto.setAlliance(isRed);
 * waitForStart();
 * auto.start();
 * while (opModeIsActive() && !auto.isFinished()) {
 *     robot.syncPose(...); // after localizer.update()
 *     robot.update();
 *     auto.update();
 * }
 * auto.stop();
 * }</pre>
 */
public abstract class AutoMode implements Component {
    protected final Follower follower;
    protected final PedroDrive drive;
    protected final ActionLibrary actions;
    protected final AutoScheduler scheduler = new AutoScheduler();

    private AutoCommand root;
    private boolean started;

    private boolean externalLoop;

    public AutoMode(Follower follower, ActionLibrary actions) {
        this.follower = follower;
        this.drive = actions.getDrive();
        this.actions = actions;
    }

    public abstract void run();

    /**
     * Starting field pose for this auton as {@code {x, y, headingDegrees}}.
     * Override per auton when it does not use the active alliance table's default start.
     */
    public double[] getStartPose() {
        return actions.poses().start;
    }

    public void setAlliance(boolean red) {
        actions.setAlliance(red);
    }

    public void setRed(boolean red) {
        setAlliance(red);
    }

    public boolean isRed() {
        return actions.isRed();
    }

    /**
     * When true, idle ticks skip redundant follower updates that re-read sensors;
     * BrainSTEMRobot syncs pose and path commands still call {@code follower.update()}.
     */
    public void setExternalLoop(boolean externalLoop) {
        this.externalLoop = externalLoop;
        drive.setExternalLoop(externalLoop);
    }

    public boolean isStarted() {
        return started;
    }

    public void start() {
        root = null;
        drive.setStartPose(getStartPose());
        run();
        if (root == null) {
            throw new IllegalStateException(
                    "AutoMode.run() must call sequence(...) (or schedule(...)).");
        }
        scheduler.schedule(root);
        started = true;
    }

    @Override
    public void update() {
        if (!started) return;
        if (scheduler.isRunning()) {
            scheduler.run();
        } else if (!externalLoop) {
            drive.update();
        }
    }

    @Override
    public void reset() {
        stop();
    }

    @Override
    public String test() {
        return getClass().getSimpleName();
    }

    public boolean isFinished() {
        return started && scheduler.isFinished();
    }

    public void stop() {
        scheduler.cancel();
        follower.breakFollowing();
        started = false;
    }

    protected void startPose(double x, double y, double headingDegrees) {
        drive.setStartPose(x, y, headingDegrees);
    }

    protected void startPose(double[] pose) {
        drive.setStartPose(pose);
    }

    protected void startPoseFromAlliance() {
        startPose(actions.poses().start);
    }

    protected void sequence(AutoCommand... commands) {
        root = actions.sequence(commands);
    }

    protected AutoCommand parallel(AutoCommand... commands) {
        return actions.parallel(commands);
    }

    protected void schedule(AutoCommand command) {
        root = command;
    }
}
