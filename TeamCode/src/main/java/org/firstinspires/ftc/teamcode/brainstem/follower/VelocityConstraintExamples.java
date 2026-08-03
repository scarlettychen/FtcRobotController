package org.firstinspires.ftc.teamcode.brainstem.follower;

import org.firstinspires.ftc.teamcode.brainstem.RobotModel;

/**
 * Lightweight examples / assertions for {@link VelocityConstraint}.
 * Call {@link #runSelfCheck()} from a test OpMode or unit harness.
 */
public final class VelocityConstraintExamples {

    private VelocityConstraintExamples() {}

    public static void runSelfCheck() {
        // Shared geometry: gentle vs tight curvature (1/in)
        final double kappaStraight = 0.0;
        final double kappaTight = 0.15; // radius ≈ 6.7 in

        RobotModel cruise = new RobotModel()
                .maxAcceleration(80)
                .maxDeceleration(100)
                .frictionCoefficient(0.7)
                .feedforward(0.05, 0.18, 0.003);
        cruise.maxVelocityOverride = 60.0;
        cruise.cruise();

        // 1) Straight → robot top speed
        VelocityConstraint.Result straight =
                VelocityConstraint.getMaxVelocity(kappaStraight, cruise, 0);
        assertTrue(
                "straight should be ROBOT_TOP_SPEED",
                straight.reason == VelocityConstraint.LimitReason.ROBOT_TOP_SPEED);
        assertNear("straight v ≈ robot top", straight.maxVelocity, cruise.motorLimitedVelocityIgnoringPathCeiling(), 0.5);

        // 2) Tight curve → slower, CURVATURE
        VelocityConstraint.Result tight =
                VelocityConstraint.getMaxVelocity(kappaTight, cruise, 0);
        assertTrue("tight should be CURVATURE", tight.reason == VelocityConstraint.LimitReason.CURVATURE);
        assertTrue("tight slower than straight", tight.maxVelocity < straight.maxVelocity);
        double expectedCurv = Math.sqrt(cruise.getMaxLateralAcceleration() / kappaTight);
        assertNear("tight matches √(a_lat/κ)", tight.maxVelocity, expectedCurv, 0.01);

        // 3) User segment cap
        VelocityConstraint.Result capped =
                VelocityConstraint.getMaxVelocity(kappaStraight, cruise, 20.0);
        assertTrue("cap binds on straight", capped.reason == VelocityConstraint.LimitReason.SEGMENT_CAP);
        assertNear("cap = 20", capped.maxVelocity, 20.0, 0.01);

        VelocityConstraint.Result cappedOnCurve =
                VelocityConstraint.getMaxVelocity(kappaTight, cruise, 50.0);
        // curvature limit ~√(0.7*386/0.15) ≈ 42; cap 50 does not bind
        assertTrue(
                "curve still CURVATURE when cap is high",
                cappedOnCurve.reason == VelocityConstraint.LimitReason.CURVATURE);
        assertTrue("never exceeds segment cap", cappedOnCurve.maxVelocity <= 50.0 + 1e-6);

        VelocityConstraint.Result lowCapOnCurve =
                VelocityConstraint.getMaxVelocity(kappaTight, cruise, 10.0);
        assertTrue("low cap binds on curve", lowCapOnCurve.reason == VelocityConstraint.LimitReason.SEGMENT_CAP);
        assertNear("low cap = 10", lowCapOnCurve.maxVelocity, 10.0, 0.01);

        // 4) Different models → different limits on same κ
        RobotModel sticky = new RobotModel().frictionCoefficient(1.0);
        sticky.maxVelocityOverride = 60.0;
        sticky.cruise();
        RobotModel slippery = new RobotModel().frictionCoefficient(0.35);
        slippery.maxVelocityOverride = 60.0;
        slippery.cruise();

        double vSticky = VelocityConstraint.getMaxVelocity(kappaTight, sticky, 0).maxVelocity;
        double vSlip = VelocityConstraint.getMaxVelocity(kappaTight, slippery, 0).maxVelocity;
        assertTrue("higher μ → higher curve speed", vSticky > vSlip);

        RobotModel slowBot = new RobotModel();
        slowBot.maxVelocityOverride = 25.0;
        slowBot.frictionCoefficient(0.7);
        slowBot.cruise();
        double vSlowStraight =
                VelocityConstraint.getMaxVelocity(kappaStraight, slowBot, 0).maxVelocity;
        assertNear("slow bot top speed", vSlowStraight, 25.0, 0.5);
    }

    private static void assertTrue(String msg, boolean cond) {
        if (!cond) {
            throw new AssertionError(msg);
        }
    }

    private static void assertNear(String msg, double actual, double expected, double tol) {
        if (Math.abs(actual - expected) > tol) {
            throw new AssertionError(msg + ": expected " + expected + " got " + actual);
        }
    }
}
