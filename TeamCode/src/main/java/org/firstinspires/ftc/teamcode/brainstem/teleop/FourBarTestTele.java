package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// four bar only via commands. a = down, x = low, y = high
@TeleOp(name = "Test Lift", group = "Test")
public class FourBarTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private FourBarLinkage lift;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        lift = new FourBarLinkage(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            if (gp1.isFirstA()) {
                run(OpmodeCommands.setLiftDown(lift));
            }
            if (gp1.isFirstX()) {
                run(OpmodeCommands.setLiftLow(lift));
            }
            if (gp1.isFirstY()) {
                run(OpmodeCommands.setLiftHigh(lift));
            }


            Scheduler.execute();
            lift.update();

            telemetry.addData("state", lift.getState());
            telemetry.addData("pos", lift.getPosition());
            telemetry.addData("atTarget", lift.atTarget());
            telemetry.update();
        }

        Scheduler.reset();
    }

    private void run(Command command) {
        Scheduler.schedule(command);
    }
}
