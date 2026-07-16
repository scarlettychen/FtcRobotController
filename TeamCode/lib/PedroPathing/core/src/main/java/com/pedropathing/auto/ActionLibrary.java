package com.pedropathing.auto;

import com.pedropathing.geometry.Pose;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Shared named actions for every auto.
 *
 * <p>Coordinates live in <b>two</b> {@link AlliancePoses} instances (Blue + Red).
 * Call {setAlliance(boolean)} (or {setRed(boolean)}) once at OpMode start;
 * all named drives then use the active side.
 *
 * <pre>
 * RobotActions bot = new RobotActions(drive, bluePoses, redPoses);
 * bot.setAlliance(isRed); // gamepad / config at OpMode init
 * auto.start();
 * </pre>
 */
public abstract class ActionLibrary {
    protected final PedroDrive drive;
    private final AlliancePoses bluePoses;
    private final AlliancePoses redPoses;
    private boolean red;

    protected ActionLibrary(PedroDrive drive, AlliancePoses bluePoses, AlliancePoses redPoses) {
        this(drive, bluePoses, redPoses, false);
    }

    protected ActionLibrary(
            PedroDrive drive,
            AlliancePoses bluePoses,
            AlliancePoses redPoses,
            boolean startRed
    ) {
        this.drive = drive;
        this.bluePoses = bluePoses;
        this.redPoses = redPoses;
        this.red = startRed;
    }

    /**
     * @param red {@code true} = Red poses, {@code false} = Blue poses
     */
    public void setAlliance(boolean red) {
        this.red = red;
    }

    /** Alias for {@link #setAlliance(boolean)}. */
    public void setRed(boolean red) {
        setAlliance(red);
    }

    public boolean isRed() {
        return red;
    }

    public boolean isBlue() {
        return !red;
    }

    /** Active alliance pose table (switches with {@link #setAlliance(boolean)}). */
    public AlliancePoses poses() {
        return red ? redPoses : bluePoses;
    }

    public PedroDrive getDrive() {
        return drive;
    }

    /** Active poses (same as {@link #poses()}). */
    public AlliancePoses getPoses() {
        return poses();
    }

    public AlliancePoses getBluePoses() {
        return bluePoses;
    }

    public AlliancePoses getRedPoses() {
        return redPoses;
    }

    // ---- Phase A motion context (scales time-optimal profiles) ----

    protected void cruise() {
        drive.getFollower().getRobotModel().cruise();
    }

    protected void loaded() {
        drive.getFollower().getRobotModel().loaded();
    }

    protected void precision() {
        drive.getFollower().getRobotModel().precision();
    }

    // ---- Low-level builders (resolve poses when the command starts) ----

    protected AutoCommand lineDrive(double[] pose, PedroDrive.Marker... markers) {
        return drive.lineDrive(pose, markers);
    }

    protected AutoCommand lineDrive(double x, double y, double headingDegrees, PedroDrive.Marker... markers) {
        return drive.lineDrive(x, y, headingDegrees, markers);
    }

    /**
     * Deferred line drive — reads the pose array when the command initializes,
     * so {@link #setAlliance(boolean)} can be called at OpMode start.
     */
    protected AutoCommand lineDrive(Supplier<double[]> poseSupplier, PedroDrive.Marker... markers) {
        return new DeferredDriveCommand(() -> drive.lineDrive(poseSupplier.get(), markers));
    }

    protected AutoCommand bezierDrive(double[]... poseArrays) {
        return drive.bezierDrive(poseArrays);
    }

    protected AutoCommand bezierDrive(Pose... controlAndEndPoses) {
        return drive.bezierDrive(controlAndEndPoses);
    }

    @SafeVarargs
    protected final AutoCommand bezierDrive(Supplier<double[]>... poseSuppliers) {
        return new DeferredDriveCommand(() -> {
            double[][] arrays = new double[poseSuppliers.length][];
            for (int i = 0; i < poseSuppliers.length; i++) {
                arrays[i] = poseSuppliers[i].get();
            }
            return drive.bezierDrive(arrays);
        });
    }

    protected AutoCommand pathDrive(double[]... waypoints) {
        return drive.pathDrive(waypoints);
    }

    protected AutoCommand pathDrive(PedroDrive.Marker[] markers, double[]... waypoints) {
        return drive.pathDrive(markers, waypoints);
    }

    /**
     * Deferred multi-waypoint drive — waypoints resolved from the active alliance at init.
     */
    @SafeVarargs
    protected final AutoCommand pathDrive(Supplier<double[]>... waypointSuppliers) {
        return new DeferredDriveCommand(() -> {
            double[][] arrays = new double[waypointSuppliers.length][];
            for (int i = 0; i < waypointSuppliers.length; i++) {
                arrays[i] = waypointSuppliers[i].get();
            }
            return drive.pathDrive(arrays);
        });
    }

    @SafeVarargs
    protected final AutoCommand pathDrive(PedroDrive.Marker[] markers, Supplier<double[]>... waypointSuppliers) {
        return new DeferredDriveCommand(() -> {
            double[][] arrays = new double[waypointSuppliers.length][];
            for (int i = 0; i < waypointSuppliers.length; i++) {
                arrays[i] = waypointSuppliers[i].get();
            }
            return drive.pathDrive(markers, arrays);
        });
    }

    protected AutoCommand turnTo(double headingDegrees) {
        return drive.turnTo(headingDegrees);
    }

    protected AutoCommand waitSeconds(double seconds) {
        return FunctionalCommand.waitSeconds(seconds);
    }

    protected AutoCommand run(Runnable action) {
        return FunctionalCommand.instant(action);
    }

    protected AutoCommand waitUntil(BooleanSupplier condition) {
        return FunctionalCommand.runUntil(() -> {}, condition);
    }

    protected AutoCommand runThenWait(Runnable start, BooleanSupplier finished) {
        return sequence(
                FunctionalCommand.instant(start),
                FunctionalCommand.runUntil(() -> {}, finished)
        );
    }

    public AutoCommand sequence(AutoCommand... commands) {
        return new CommandGroups.Sequential(commands);
    }

    public AutoCommand parallel(AutoCommand... commands) {
        return new CommandGroups.Parallel(commands);
    }

    /**
     * Builds the inner drive command only when this command initializes,
     * so alliance pose selection is always current.
     */
    private static final class DeferredDriveCommand extends BaseAutoCommand {
        private final Supplier<AutoCommand> factory;
        private AutoCommand inner;

        DeferredDriveCommand(Supplier<AutoCommand> factory) {
            this.factory = factory;
        }

        @Override
        public void initialize() {
            inner = factory.get();
            inner.initialize();
        }

        @Override
        public void execute() {
            if (inner != null) inner.execute();
        }

        @Override
        public boolean isFinished() {
            return inner == null || inner.isFinished();
        }

        @Override
        public void end() {
            if (inner != null) inner.end();
        }
    }
}
