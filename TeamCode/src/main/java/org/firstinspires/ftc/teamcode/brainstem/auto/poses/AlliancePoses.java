package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

import com.pedropathing.auto.PedroDrive;
import com.pedropathing.geometry.Pose;

// alliance field coords as {x, y, headingDeg}
// keep blue + red subclasses. switch w/ robotactions.setAlliance
public abstract class AlliancePoses {

    public double[] start = xyz(0, 0, 0);

    public static double[] xyz(double x, double y, double headingDegrees) {
        return new double[]{x, y, headingDegrees};
    }

    public static Pose toPose(double[] p) {
        return PedroDrive.fieldPose(p);
    }

    public Pose startPose() {
        return toPose(start);
    }
}
