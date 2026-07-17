package com.pedropathing.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Pedro wrapper for autonomous driving.
 *
 * <p><b>When are paths baked?</b> Not at OpMode start. Each drive command builds its
 * curve in {@code initialize()} using the robot's <em>current</em> pose as the start.
 * Destinations / control points are absolute field coordinates.
 *
 * <p>Phase A: time-optimal profiles are on by default; {@link Marker}s fire mid-path
 * by path completion. Motion speed follows the TeamCode-owned motion model.
 */
public class PedroDrive {
    private final Follower follower;
    private boolean useTimeOptimal = true;
    private boolean holdEnd = true;
    private boolean externalLoop = false;

    /**
     * Mid-path callback. {@code at} is path completion in [0, 1].
     * With time-optimal on, that fraction spans the whole trajectory/chain arc length
     * (current → end). Fired once when {@code follower.getPathCompletion() >= at}.
     *
     * <p>Waypoints shape geometry ({@link #pathDrive}); markers are callbacks along that path.
     */
    public static final class Marker {
        public final double at;
        public final Runnable action;

        public Marker(double at, Runnable action) {
            this.at = at;
            this.action = action;
        }
    }

    public PedroDrive(Follower follower) {
        this.follower = follower;
    }

    public Follower getFollower() {
        return follower;
    }

    public PedroDrive useTimeOptimal(boolean useTimeOptimal) {
        this.useTimeOptimal = useTimeOptimal;
        return this;
    }

    public boolean isTimeOptimal() {
        return useTimeOptimal;
    }

    public PedroDrive holdEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * When hosted by BrainSTEMRobot, pose is pushed via ExternalPoseLocalizer;
     * follower.update() still runs during path following so control loops tick.
     */
    public void setExternalLoop(boolean externalLoop) {
        this.externalLoop = externalLoop;
    }

    public void setStartPose(double x, double y, double headingDegrees) {
        follower.setStartingPose(Pose.fromFieldDegrees(x, y, headingDegrees));
    }

    /** {@code double[]{x, y, headingDegrees}} — same format as TeamCode pose arrays. */
    public void setStartPose(double[] pose) {
        Pose p = AlliancePoses.toPose(pose);
        follower.setStartingPose(p);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    /** Straight Bezier line to field (x, y). Keeps current heading. Field coords → Pedro. */
    public AutoCommand lineDrive(double x, double y, Marker... markers) {
        Pose pedro = Pose.fromFieldDegrees(x, y, 0);
        return new LineDriveCommand(this, pedro, false, markers);
    }

    /** Straight Bezier line to field (x, y) while rotating to headingDegrees. Field coords → Pedro. */
    public AutoCommand lineDrive(double x, double y, double headingDegrees, Marker... markers) {
        return new LineDriveCommand(this, Pose.fromFieldDegrees(x, y, headingDegrees), true, markers);
    }

    /**
     * Straight Bezier line to a TeamCode-style pose array {@code {x, y, headingDegrees}}.
     * Converted once via {@link AlliancePoses#toPose}. End heading is applied.
     */
    public AutoCommand lineDrive(double[] pose, Marker... markers) {
        return new LineDriveCommand(this, AlliancePoses.toPose(pose), true, markers);
    }

    /**
     * Drive {@code inches} forward along the robot's <em>current</em> Pedro heading.
     * Bakes from live pose at {@code initialize()} — use this for smoke tests so FTC
     * 90° heading vs Pinpoint 0° cannot turn "forward" into a strafe path.
     */
    public AutoCommand forwardDrive(double inches, Marker... markers) {
        return new ForwardDriveCommand(this, inches, markers);
    }

    /**
     * Curved Bezier ~{@code inches} forward along current heading, bulging
     * {@code sideOffsetInches} to the robot's left (negative = right).
     * Baked from live pose at {@code initialize()}.
     */
    public AutoCommand bezierForwardDrive(double inches, double sideOffsetInches, Marker... markers) {
        return new BezierForwardDriveCommand(this, inches, sideOffsetInches, markers);
    }

    /**
     * Curved Bezier from the robot's current pose through absolute control/end poses.
     * Pass Pedro poses (e.g. from {@link AlliancePoses#toPose} / {@link Pose#fromField}).
     */
    public AutoCommand bezierDrive(Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        return new BezierDriveCommand(this, Arrays.asList(poses), false, Double.NaN);
    }

    /** Forces end heading in field degrees (converted to Pedro for following). */
    public AutoCommand bezierDrive(double endHeadingDegrees, Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        double pedroHeading = Pose.fromFieldDegrees(0, 0, endHeadingDegrees).getHeading();
        return new BezierDriveCommand(this, Arrays.asList(poses), true, pedroHeading);
    }

    public AutoCommand bezierDrive(double[]... poseArrays) {
        Pose[] poses = new Pose[poseArrays.length];
        for (int i = 0; i < poseArrays.length; i++) {
            poses[i] = AlliancePoses.toPose(poseArrays[i]);
        }
        Pose end = poses[poses.length - 1];
        return new BezierDriveCommand(this, Arrays.asList(poses), true, end.getHeading());
    }

    public AutoCommand bezierDrive(Marker[] markers, Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        Pose end = poses[poses.length - 1];
        return new BezierDriveCommand(this, Arrays.asList(poses), true, end.getHeading(), markers);
    }

    public AutoCommand pathDrive(double[]... waypoints) {
        if (waypoints == null || waypoints.length == 0) {
            throw new IllegalArgumentException("pathDrive requires at least one waypoint");
        }
        return new PathDriveCommand(this, waypoints);
    }

    public AutoCommand pathDrive(Marker[] markers, double[]... waypoints) {
        if (waypoints == null || waypoints.length == 0) {
            throw new IllegalArgumentException("pathDrive requires at least one waypoint");
        }
        return new PathDriveCommand(this, markers, waypoints);
    }

    public AutoCommand turnTo(double headingDegrees) {
        return new TurnCommand(this, headingDegrees);
    }

    public void update() {
        follower.update();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    double pathCompletion() {
        return follower.getPathCompletion();
    }

    void startLine(Pose endPedro, boolean setHeading) {
        Pose start = follower.getPose();
        Pose end = setHeading
                ? endPedro
                : new Pose(endPedro.getX(), endPedro.getY(), start.getHeading());

        Path path = new Path(new BezierLine(start, end));
        if (setHeading) {
            path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        } else {
            path.setConstantHeadingInterpolation(start.getHeading());
        }
        follow(path);
    }

    void startBezier(List<Pose> absolutePoses, boolean setHeading, double endHeadingRadiansPedro) {
        Pose start = follower.getPose();
        List<Pose> controls = new ArrayList<>();
        controls.add(start.copy());
        for (Pose p : absolutePoses) {
            controls.add(p.copy());
        }

        Path path;
        if (controls.size() == 2) {
            path = new Path(new BezierLine(controls.get(0), controls.get(1)));
        } else {
            path = new Path(new BezierCurve(controls));
        }

        if (setHeading) {
            path.setLinearHeadingInterpolation(start.getHeading(), endHeadingRadiansPedro);
        } else {
            // Always hold start heading when not explicitly rotating.
            // (Previously end.heading != 0 triggered linear interp; default Path heading is tangent.)
            path.setConstantHeadingInterpolation(start.getHeading());
        }
        follow(path);
    }

    void startTurn(double headingDegrees) {
        // Field heading → Pedro heading via fromField at origin (rotation only still applies).
        Pose fieldHeading = Pose.fromFieldDegrees(0, 0, headingDegrees);
        follower.turnTo(fieldHeading.getHeading());
    }

    void startPath(double[]... waypoints) {
        Pose start = follower.getPose();
        Pose prev = start;
        com.pedropathing.paths.PathBuilder builder = follower.pathBuilder();

        for (double[] waypoint : waypoints) {
            Pose end = AlliancePoses.toPose(waypoint);
            Path path = new Path(new BezierLine(prev, end));
            path.setLinearHeadingInterpolation(prev.getHeading(), end.getHeading());
            builder.addPath(path);
            prev = end;
        }

        PathChain chain = builder.build();
        if (useTimeOptimal) {
            follower.followPathChainTimeOptimal(chain);
        } else {
            follower.followPath(chain, holdEnd);
        }
    }

    private void follow(Path path) {
        follow(path, useTimeOptimal);
    }

    private void follow(Path path, boolean timeOptimal) {
        PathChain chain = follower.pathBuilder()
                .addPath(path)
                .build();
        if (timeOptimal) {
            follower.followPathChainTimeOptimal(chain);
        } else {
            follower.followPath(chain, holdEnd);
        }
    }

    private abstract static class FollowDriveCommand extends BaseAutoCommand {
        final PedroDrive drive;
        final Marker[] markers;
        private boolean[] fired;

        FollowDriveCommand(PedroDrive drive, Marker... markers) {
            this.drive = drive;
            this.markers = markers == null || markers.length == 0 ? EMPTY : markers;
        }

        private static final Marker[] EMPTY = new Marker[0];

        @Override
        public void execute() {
            if (markers.length > 0) {
                if (fired == null) fired = new boolean[markers.length];
                double c = drive.pathCompletion();
                for (int i = 0; i < markers.length; i++) {
                    if (!fired[i] && c >= markers[i].at) {
                        fired[i] = true;
                        markers[i].action.run();
                    }
                }
            }
            drive.update();
        }

        @Override
        public boolean isFinished() {
            return !drive.isBusy();
        }
    }

    private static final class LineDriveCommand extends FollowDriveCommand {
        private final Pose end;
        private final boolean setHeading;

        LineDriveCommand(PedroDrive drive, Pose endPedro, boolean setHeading, Marker... markers) {
            super(drive, markers);
            this.end = endPedro;
            this.setHeading = setHeading;
        }

        @Override
        public void initialize() {
            drive.startLine(end, setHeading);
        }
    }

    private static final class ForwardDriveCommand extends FollowDriveCommand {
        private final double inches;

        ForwardDriveCommand(PedroDrive drive, double inches, Marker... markers) {
            super(drive, markers);
            this.inches = inches;
        }

        @Override
        public void initialize() {
            Pose start = drive.follower.getPose();
            double h = start.getHeading();
            Pose end = new Pose(
                    start.getX() + inches * Math.cos(h),
                    start.getY() + inches * Math.sin(h),
                    h
            );
            drive.startLine(end, false);
        }
    }

    private static final class BezierForwardDriveCommand extends FollowDriveCommand {
        private final double inches;
        private final double sideOffsetInches;

        BezierForwardDriveCommand(
                PedroDrive drive, double inches, double sideOffsetInches, Marker... markers) {
            super(drive, markers);
            this.inches = inches;
            this.sideOffsetInches = sideOffsetInches;
        }

        @Override
        public void initialize() {
            Pose start = drive.follower.getPose();
            double h = start.getHeading();
            double fx = Math.cos(h);
            double fy = Math.sin(h);
            // Robot-left unit vector in Pedro frame.
            double lx = -Math.sin(h);
            double ly = Math.cos(h);

            // Gentle cubic Bezier: both controls share the same side offset so the curve
            // is a smooth bow (start/end tangents stay mostly forward).
            Pose c1 = new Pose(
                    start.getX() + (1.0 / 3.0) * inches * fx + sideOffsetInches * lx,
                    start.getY() + (1.0 / 3.0) * inches * fy + sideOffsetInches * ly,
                    h
            );
            Pose c2 = new Pose(
                    start.getX() + (2.0 / 3.0) * inches * fx + sideOffsetInches * lx,
                    start.getY() + (2.0 / 3.0) * inches * fy + sideOffsetInches * ly,
                    h
            );
            Pose end = new Pose(
                    start.getX() + inches * fx,
                    start.getY() + inches * fy,
                    h
            );
            drive.startBezier(Arrays.asList(c1, c2, end), false, Double.NaN);
        }
    }

    private static final class BezierDriveCommand extends FollowDriveCommand {
        private final List<Pose> poses;
        private final boolean setHeading;
        private final double endHeadingRadiansPedro;

        BezierDriveCommand(
                PedroDrive drive,
                List<Pose> poses,
                boolean setHeading,
                double endHeadingRadiansPedro,
                Marker... markers
        ) {
            super(drive, markers);
            this.poses = poses;
            this.setHeading = setHeading;
            this.endHeadingRadiansPedro = endHeadingRadiansPedro;
        }

        @Override
        public void initialize() {
            drive.startBezier(poses, setHeading, endHeadingRadiansPedro);
        }
    }

    private static final class PathDriveCommand extends FollowDriveCommand {
        private final double[][] waypoints;

        PathDriveCommand(PedroDrive drive, double[]... waypoints) {
            this(drive, null, waypoints);
        }

        PathDriveCommand(PedroDrive drive, Marker[] markers, double[]... waypoints) {
            super(drive, markers);
            this.waypoints = waypoints;
        }

        @Override
        public void initialize() {
            drive.startPath(waypoints);
        }
    }

    private static final class TurnCommand extends BaseAutoCommand {
        private final PedroDrive drive;
        private final double headingDegrees;

        TurnCommand(PedroDrive drive, double headingDegrees) {
            this.drive = drive;
            this.headingDegrees = headingDegrees;
        }

        @Override
        public void initialize() {
            drive.startTurn(headingDegrees);
        }

        @Override
        public void execute() {
            drive.update();
        }

        @Override
        public boolean isFinished() {
            return !drive.isBusy();
        }
    }
}
