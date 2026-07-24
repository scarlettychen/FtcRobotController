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
 * Pedro path wrapper — start a path, then call {@link #update()} until {@link #isBusy()} is false.
 * No command framework; OpModes own the loop.
 */
public class PedroDrive {
    private final Follower follower;
    private boolean useTimeOptimal = true;
    private boolean holdEnd = true;
    private boolean settleEnd = false;
    private boolean externalLoop = false;

    private Marker[] activeMarkers = EMPTY_MARKERS;
    private boolean[] markersFired;
    private Boolean restoreSettleAfterPath;

    private static final Marker[] EMPTY_MARKERS = new Marker[0];

    public static final class Marker {
        public final double at;
        public final Runnable action;

        public Marker(double at, Runnable action) {
            this.at = at;
            this.action = action;
        }
    }

    public enum BezierHeading {
        HOLD_START,
        HOLD,
        TANGENT
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

    public PedroDrive settleEnd(boolean settleEnd) {
        this.settleEnd = settleEnd;
        return this;
    }

    public boolean isSettleEnd() {
        return settleEnd;
    }

    public void setExternalLoop(boolean externalLoop) {
        this.externalLoop = externalLoop;
    }

    public void setStartPose(double x, double y, double headingDegrees) {
        follower.setStartingPose(Pose.fromFieldDegrees(x, y, headingDegrees));
    }

    public void setStartPose(double[] pose) {
        follower.setStartingPose(fieldPose(pose));
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public static Pose fieldPose(double[] p) {
        if (p == null || p.length < 2) {
            throw new IllegalArgumentException("Pose array needs at least x,y");
        }
        double headingDeg = p.length >= 3 ? p[2] : 0;
        return Pose.fromFieldDegrees(p[0], p[1], headingDeg);
    }

    /** Straight line to field (x, y); holds live heading. */
    public void lineDrive(double x, double y, Marker... markers) {
        beginMarkers(markers);
        startLine(Pose.fromFieldDegrees(x, y, 0));
    }

    public void lineDrive(double x, double y, double headingDegrees, Marker... markers) {
        lineDrive(x, y, markers);
    }

    public void lineDrive(double[] pose, Marker... markers) {
        if (pose == null || pose.length < 2) {
            throw new IllegalArgumentException("lineDrive pose needs at least x,y");
        }
        lineDrive(pose[0], pose[1], markers);
    }

    /**
     * Forward along live Pedro heading (settle forced off for this path).
     */
    public void forwardDrive(double inches, Marker... markers) {
        beginMarkers(markers);
        restoreSettleAfterPath = settleEnd;
        settleEnd = false;
        Pose start = follower.getPose();
        double h = start.getHeading();
        startLine(new Pose(
                start.getX() + inches * Math.cos(h),
                start.getY() + inches * Math.sin(h),
                h
        ));
    }

    /**
     * Strafe along live Pedro heading (positive = left). Settle forced off for this path.
     */
    public void strafeDrive(double inches, Marker... markers) {
        beginMarkers(markers);
        restoreSettleAfterPath = settleEnd;
        settleEnd = false;
        Pose start = follower.getPose();
        double h = start.getHeading();
        // left is +90° from forward: (-sin h, cos h)
        startLine(new Pose(
                start.getX() + inches * -Math.sin(h),
                start.getY() + inches * Math.cos(h),
                h
        ));
    }

    public void bezierForwardDrive(double inches, double sideOffsetInches, Marker... markers) {
        beginMarkers(markers);
        Pose start = follower.getPose();
        double h = start.getHeading();
        double fx = Math.cos(h);
        double fy = Math.sin(h);
        double lx = -Math.sin(h);
        double ly = Math.cos(h);
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
        startBezier(Arrays.asList(c1, c2, end), BezierHeading.HOLD_START, Double.NaN);
    }

    public void bezierDrive(Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        beginMarkers();
        startBezier(Arrays.asList(poses), BezierHeading.HOLD_START, Double.NaN);
    }

    public void bezierDrive(double endHeadingDegrees, Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        beginMarkers();
        double pedroHeading = Pose.fromFieldDegrees(0, 0, endHeadingDegrees).getHeading();
        startBezier(Arrays.asList(poses), BezierHeading.HOLD, pedroHeading);
    }

    public void bezierDrive(double[]... poseArrays) {
        if (poseArrays == null || poseArrays.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        Pose[] poses = new Pose[poseArrays.length];
        for (int i = 0; i < poseArrays.length; i++) {
            poses[i] = fieldPose(poseArrays[i]);
        }
        beginMarkers();
        Pose end = poses[poses.length - 1];
        startBezier(Arrays.asList(poses), BezierHeading.HOLD, end.getHeading());
    }

    public void bezierDriveTangent(double[]... poseArrays) {
        if (poseArrays == null || poseArrays.length == 0) {
            throw new IllegalArgumentException("bezierDriveTangent requires at least an end pose");
        }
        Pose[] poses = new Pose[poseArrays.length];
        for (int i = 0; i < poseArrays.length; i++) {
            poses[i] = fieldPose(poseArrays[i]);
        }
        beginMarkers();
        startBezier(Arrays.asList(poses), BezierHeading.TANGENT, Double.NaN);
    }

    public void bezierDrive(Marker[] markers, Pose... poses) {
        if (poses == null || poses.length == 0) {
            throw new IllegalArgumentException("bezierDrive requires at least an end pose");
        }
        beginMarkers(markers);
        Pose end = poses[poses.length - 1];
        startBezier(Arrays.asList(poses), BezierHeading.HOLD, end.getHeading());
    }

    public void pathDrive(double[]... waypoints) {
        pathDrive(null, waypoints);
    }

    public void pathDrive(Marker[] markers, double[]... waypoints) {
        if (waypoints == null || waypoints.length == 0) {
            throw new IllegalArgumentException("pathDrive requires at least one waypoint");
        }
        beginMarkers(markers);
        startPath(waypoints);
    }

    public void turnTo(double headingDegrees) {
        beginMarkers();
        startTurn(headingDegrees);
    }

    /** Alias for {@link #pathDrive(double[]...)}. */
    public void startPathDrive(double[]... waypoints) {
        pathDrive(waypoints);
    }

    public void startLineDrive(double[] pose) {
        lineDrive(pose);
    }

    public boolean isFollowingPath() {
        return follower.isBusy();
    }

    public void stopPathAndResumeTeleop() {
        clearPathState();
        follower.breakFollowing();
        follower.startTeleopDrive();
    }

    public void update() {
        fireMarkers();
        follower.update();
        if (!follower.isBusy()) {
            finishPathState();
        }
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public double pathCompletion() {
        return follower.getPathCompletion();
    }

    private void beginMarkers(Marker... markers) {
        activeMarkers = markers == null || markers.length == 0 ? EMPTY_MARKERS : markers;
        markersFired = activeMarkers.length == 0 ? null : new boolean[activeMarkers.length];
    }

    private void fireMarkers() {
        if (activeMarkers.length == 0 || markersFired == null) {
            return;
        }
        double c = follower.getPathCompletion();
        for (int i = 0; i < activeMarkers.length; i++) {
            if (!markersFired[i] && c >= activeMarkers[i].at) {
                markersFired[i] = true;
                activeMarkers[i].action.run();
            }
        }
    }

    private void finishPathState() {
        if (restoreSettleAfterPath != null) {
            settleEnd = restoreSettleAfterPath;
            restoreSettleAfterPath = null;
        }
        activeMarkers = EMPTY_MARKERS;
        markersFired = null;
    }

    private void clearPathState() {
        restoreSettleAfterPath = null;
        activeMarkers = EMPTY_MARKERS;
        markersFired = null;
    }

    private void startLine(Pose endPedro) {
        Pose start = follower.getPose();
        Pose end = new Pose(endPedro.getX(), endPedro.getY(), start.getHeading());
        Path path = new Path(new BezierLine(start, end));
        path.setConstantHeadingInterpolation(start.getHeading());
        follow(path);
    }

    private void startBezier(
            List<Pose> absolutePoses, BezierHeading headingMode, double holdHeadingRadiansPedro) {
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

        if (headingMode == BezierHeading.TANGENT) {
            path.setTangentHeadingInterpolation();
        } else if (headingMode == BezierHeading.HOLD
                && !Double.isNaN(holdHeadingRadiansPedro)) {
            path.setConstantHeadingInterpolation(holdHeadingRadiansPedro);
        } else {
            path.setConstantHeadingInterpolation(start.getHeading());
        }
        follow(path);
    }

    private void startTurn(double headingDegrees) {
        Pose fieldHeading = Pose.fromFieldDegrees(0, 0, headingDegrees);
        follower.turnTo(fieldHeading.getHeading());
    }

    private void startPath(double[]... waypoints) {
        Pose start = follower.getPose();
        Pose prev = start;
        Double prevWaypointHeading = null;
        com.pedropathing.paths.PathBuilder builder = follower.pathBuilder();

        for (double[] waypoint : waypoints) {
            Pose end = fieldPose(waypoint);
            Path path = new Path(new BezierLine(prev, end));
            if (prevWaypointHeading == null
                    || Math.abs(com.pedropathing.math.MathFunctions.normalizeAngleSigned(
                            end.getHeading() - prevWaypointHeading)) < Math.toRadians(1.0)) {
                path.setConstantHeadingInterpolation(prev.getHeading());
            } else {
                path.setLinearHeadingInterpolation(prev.getHeading(), end.getHeading());
            }
            builder.addPath(path);
            prevWaypointHeading = end.getHeading();
            prev = end;
        }

        PathChain chain = builder.build();
        if (useTimeOptimal) {
            follower.followPathChainTimeOptimal(chain, settleEnd);
        } else {
            follower.followPath(chain, holdEnd);
        }
    }

    private void follow(Path path) {
        PathChain chain = follower.pathBuilder()
                .addPath(path)
                .build();
        if (useTimeOptimal) {
            follower.followPathChainTimeOptimal(chain, settleEnd);
        } else {
            follower.followPath(chain, holdEnd);
        }
    }
}
