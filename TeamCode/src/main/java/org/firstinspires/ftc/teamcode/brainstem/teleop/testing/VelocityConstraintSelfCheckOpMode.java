package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.follower.VelocityConstraintExamples;

/**
 * Runs {@link VelocityConstraintExamples#runSelfCheck()} on the RC.
 * Expect telemetry "PASS" — fails loudly if assertions break.
 */
@TeleOp(name = "VelocityConstraint Self-Check", group = "SysId")
public class VelocityConstraintSelfCheckOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("VelocityConstraint self-check");
        telemetry.addLine("Press start to run assertions");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            VelocityConstraintExamples.runSelfCheck();
            telemetry.addLine("PASS — straight / curve / cap / model cases OK");
        } catch (AssertionError e) {
            telemetry.addLine("FAIL");
            telemetry.addLine(e.getMessage());
        }
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }
}
