package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// blocker only via commands. a = open, b = close
@TeleOp(name = "Test Blocker", group = "Test")
public class BlockerTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Blocker blocker;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        blocker = new Blocker(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            if (gp1.isFirstA()) {
                run(OpmodeCommands.openBlocker(blocker));
            }
            if (gp1.isFirstB()) {
                run(OpmodeCommands.closeBlocker(blocker));
            }

            Scheduler.execute();
            blocker.update();

            telemetry.addData("state", blocker.getState());
            telemetry.update();
        }

        Scheduler.reset();
    }

    private void run(Command command) {
        Scheduler.schedule(command);
    }
}
