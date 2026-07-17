package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

import com.pedropathing.auto.AlliancePoses;

/**
 * Team-owned named field locations.
 *
 * <p>Add every new named pose needed by {@code RobotActions} here, then fill it in for each
 * alliance subclass. Pedro's generic {@link AlliancePoses} intentionally knows only the start.
 */
public abstract class RobotPoses extends AlliancePoses {
    public double[] lookAtOb = xyz(0, 0, 0);
    public double[] openGatePos = xyz(0, 0, 0);
    public double[] limelight = xyz(0, 0, 0);
    public double[] close1Shooting = xyz(0, 0, 0);
    public double[] collect1Pre = xyz(0, 0, 0);
    public double[] collect1Mid = xyz(0, 0, 0);
    public double[] firstSpikeEnd = xyz(0, 0, 0);
    public double[] strafePos = xyz(0, 0, 0);
    public double[] collect2Mid = xyz(0, 0, 0);
    public double[] collect2Pre = xyz(0, 0, 0);
    public double[] secondSpikeEnd = xyz(0, 0, 0);
    public double[] collect3Pre = xyz(0, 0, 0);
    public double[] collect3PrePass = xyz(0, 0, 0);
    public double[] thirdSpikeEnd = xyz(0, 0, 0);
}
