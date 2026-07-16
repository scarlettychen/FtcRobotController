package org.firstinspires.ftc.teamcode.auto.poses;

import com.pedropathing.auto.AlliancePoses;

/**
 * Smoke-test field poses: origin start, +5 in X as the "goal".
 */
public class TestPoses extends AlliancePoses {
    /** (5, 0, 0°) — used by {@link org.firstinspires.ftc.teamcode.auto.RobotActions#driveForwardFive()}. */
    public double[] forwardFive = xyz(5, 0, 0);

    public TestPoses() {
        start = xyz(0, 0, 0);
        // Treat the 5" mark as the close shoot / goal for the smoke test.
        close1Shooting = forwardFive;
    }
}
