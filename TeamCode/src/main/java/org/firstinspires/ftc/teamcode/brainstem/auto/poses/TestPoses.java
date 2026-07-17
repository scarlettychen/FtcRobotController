package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

/** Smoke-test coordinates: origin start, +5 inches in X. */
public class TestPoses extends RobotPoses {
    public final double[] forwardFive = xyz(5, 0, 0);

    public TestPoses() {
        start = xyz(0, 0, 0);
        close1Shooting = forwardFive;
    }
}
