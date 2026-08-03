package org.firstinspires.ftc.teamcode.brainstem.follower;

import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Library-neutral path. Java autos and (eventually) a UI planner build this;
 * {@link PathSpecConverter} turns it into the active follower's internal type.
 */
public final class PathSpec {

    public enum HeadingMode {
        HOLD_START,
        HOLD,
        TANGENT
    }

    public static final class Waypoint {
        public final double x;
        public final double y;
        public final Double headingDegrees;

        public Waypoint(double x, double y) {
            this(x, y, null);
        }

        public Waypoint(double x, double y, Double headingDegrees) {
            this.x = x;
            this.y = y;
            this.headingDegrees = headingDegrees;
        }
    }

    public static final class Segment {
        public final List<Waypoint> controlPoints;
        public final HeadingMode headingMode;
        /**
         * in/s. {@code 0} = dynamic limit from {@link VelocityConstraint} (robot + curvature).
         * Nonzero = hard upper cap: {@code min(cap, dynamic)}.
         */
        public final double maxVelocity;

        public Segment(List<Waypoint> controlPoints, HeadingMode headingMode, double maxVelocity) {
            if (controlPoints == null || controlPoints.size() < 2) {
                throw new IllegalArgumentException("Segment needs at least 2 control points");
            }
            this.controlPoints = Collections.unmodifiableList(new ArrayList<>(controlPoints));
            this.headingMode = headingMode == null ? HeadingMode.HOLD_START : headingMode;
            this.maxVelocity = maxVelocity;
        }
    }

    public final List<Segment> segments;
    public final String name;

    public PathSpec(String name, List<Segment> segments) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("PathSpec needs at least one segment");
        }
        this.name = name == null ? "" : name;
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
    }

    // ---- factories (FieldCoords inches / degrees) ----

    /** Straight line; holds start heading. */
    public static PathSpec line(String name, double x0, double y0, double x1, double y1) {
        return line(name, x0, y0, x1, y1, HeadingMode.HOLD_START, null);
    }

    /** Straight line to (x1,y1) with optional hold heading at end. */
    public static PathSpec line(
            String name,
            double x0,
            double y0,
            double x1,
            double y1,
            HeadingMode mode,
            Double holdHeadingDeg) {
        List<Waypoint> pts = Arrays.asList(
                new Waypoint(x0, y0, holdHeadingDeg),
                new Waypoint(x1, y1, holdHeadingDeg));
        return new PathSpec(name, Collections.singletonList(new Segment(pts, mode, 0)));
    }

    /**
     * Line from live start pose to field target.
     * {@code HOLD_START} ignores target heading; {@code HOLD} uses target[2].
     */
    public static PathSpec lineTo(String name, double[] startPose, double[] target, HeadingMode mode) {
        double x0 = startPose[0];
        double y0 = startPose[1];
        double x1 = target[0];
        double y1 = target[1];
        Double hold = target.length >= 3 ? target[2] : null;
        if (mode == HeadingMode.HOLD && hold == null) {
            hold = startPose.length >= 3 ? startPose[2] : 0.0;
        }
        return line(name, x0, y0, x1, y1, mode, hold);
    }

    /** Forward (+) / back (−) along current FieldCoords heading. */
    public static PathSpec forward(String name, double[] startPose, double inches) {
        double hRad = FieldCoords.ccwRadians(Math.toRadians(startPose[2]));
        return line(
                name,
                startPose[0],
                startPose[1],
                startPose[0] + inches * Math.cos(hRad),
                startPose[1] + inches * Math.sin(hRad),
                HeadingMode.HOLD_START,
                startPose[2]);
    }

    /** Strafe left (+) / right (−) along current FieldCoords heading. */
    public static PathSpec strafe(String name, double[] startPose, double inchesLeft) {
        double hRad = FieldCoords.ccwRadians(Math.toRadians(startPose[2]));
        // left = +90° from forward
        double lx = Math.cos(hRad + Math.PI / 2);
        double ly = Math.sin(hRad + Math.PI / 2);
        return line(
                name,
                startPose[0],
                startPose[1],
                startPose[0] + inchesLeft * lx,
                startPose[1] + inchesLeft * ly,
                HeadingMode.HOLD_START,
                startPose[2]);
    }

    /** Cubic/quadratic Bezier: control points are absolute field waypoints (2+). */
    public static PathSpec bezier(
            String name, HeadingMode mode, Double holdHeadingDeg, Waypoint... controls) {
        if (controls == null || controls.length < 2) {
            throw new IllegalArgumentException("bezier needs >= 2 control points");
        }
        List<Waypoint> pts = new ArrayList<>();
        for (Waypoint w : controls) {
            pts.add(new Waypoint(w.x, w.y, holdHeadingDeg != null ? holdHeadingDeg : w.headingDegrees));
        }
        return new PathSpec(name, Collections.singletonList(new Segment(pts, mode, 0)));
    }

    // ---- JSON (UI planner contract) ----

    public String toJson() {
        try {
            JSONObject root = new JSONObject();
            root.put("name", name);
            JSONArray segs = new JSONArray();
            for (Segment s : segments) {
                JSONObject sj = new JSONObject();
                sj.put("headingMode", s.headingMode.name());
                sj.put("maxVelocity", s.maxVelocity);
                JSONArray cps = new JSONArray();
                for (Waypoint w : s.controlPoints) {
                    JSONObject wj = new JSONObject();
                    wj.put("x", w.x);
                    wj.put("y", w.y);
                    if (w.headingDegrees != null) {
                        wj.put("headingDegrees", w.headingDegrees);
                    }
                    cps.put(wj);
                }
                sj.put("controlPoints", cps);
                segs.put(sj);
            }
            root.put("segments", segs);
            return root.toString(2);
        } catch (JSONException e) {
            throw new IllegalStateException("PathSpec toJson failed", e);
        }
    }

    public static PathSpec fromJson(String json) {
        try {
            JSONObject root = new JSONObject(json);
            String name = root.optString("name", "");
            JSONArray segs = root.getJSONArray("segments");
            List<Segment> segments = new ArrayList<>();
            for (int i = 0; i < segs.length(); i++) {
                JSONObject sj = segs.getJSONObject(i);
                HeadingMode mode = HeadingMode.valueOf(sj.optString("headingMode", "HOLD_START"));
                double maxV = sj.optDouble("maxVelocity", 0);
                JSONArray cps = sj.getJSONArray("controlPoints");
                List<Waypoint> pts = new ArrayList<>();
                for (int j = 0; j < cps.length(); j++) {
                    JSONObject wj = cps.getJSONObject(j);
                    Double h = wj.has("headingDegrees") ? wj.getDouble("headingDegrees") : null;
                    pts.add(new Waypoint(wj.getDouble("x"), wj.getDouble("y"), h));
                }
                segments.add(new Segment(pts, mode, maxV));
            }
            return new PathSpec(name, segments);
        } catch (JSONException e) {
            throw new IllegalArgumentException("Invalid PathSpec JSON", e);
        }
    }
}
