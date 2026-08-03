package org.firstinspires.ftc.teamcode.brainstem.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * Converts library-neutral {@link PathSpec} into Pedro {@link PathChain}.
 * Only classes inside follower/ should call this.
 */
final class PathSpecConverter {

    private PathSpecConverter() {}

    static PathChain toPedroPathChain(PathSpec spec, Follower follower) {
        if (spec == null || spec.segments.isEmpty()) {
            throw new IllegalArgumentException("PathSpec has no segments");
        }

        PathBuilder builder = follower.pathBuilder();
        // Pedro radians — used for HOLD_START across segments
        double segmentStartHeading = follower.getPose().getHeading();

        for (PathSpec.Segment segment : spec.segments) {
            List<Pose> controls = new ArrayList<>(segment.controlPoints.size());
            for (PathSpec.Waypoint wp : segment.controlPoints) {
                double headingDeg = wp.headingDegrees != null
                        ? wp.headingDegrees
                        : 0.0;
                controls.add(Pose.fromFieldDegrees(wp.x, wp.y, headingDeg));
            }

            Path path;
            if (controls.size() == 2) {
                path = new Path(new BezierLine(controls.get(0), controls.get(1)));
            } else {
                path = new Path(new BezierCurve(controls));
            }

            segmentStartHeading = applyHeading(path, segment, segmentStartHeading);
            builder.addPath(path);
        }

        return builder.build();
    }

    /** @return Pedro heading radians to use as HOLD_START for the next segment */
    private static double applyHeading(
            Path path,
            PathSpec.Segment segment,
            double segmentStartHeadingPedro) {
        switch (segment.headingMode) {
            case TANGENT:
                path.setTangentHeadingInterpolation();
                // next HOLD_START falls back to previous start heading
                return segmentStartHeadingPedro;
            case HOLD: {
                Double holdDeg = null;
                for (int i = segment.controlPoints.size() - 1; i >= 0; i--) {
                    Double h = segment.controlPoints.get(i).headingDegrees;
                    if (h != null) {
                        holdDeg = h;
                        break;
                    }
                }
                double hold = holdDeg != null
                        ? Pose.fromFieldDegrees(0, 0, holdDeg).getHeading()
                        : segmentStartHeadingPedro;
                path.setConstantHeadingInterpolation(hold);
                return hold;
            }
            case HOLD_START:
            default:
                path.setConstantHeadingInterpolation(segmentStartHeadingPedro);
                return segmentStartHeadingPedro;
        }
    }
}
