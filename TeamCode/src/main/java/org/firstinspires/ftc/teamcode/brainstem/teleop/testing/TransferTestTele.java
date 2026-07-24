package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// transfer only via commands. a = on, b = off, x = reverse
@TeleOp(name = "Test Transfer", group = "Test")
public class TransferTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Transfer transfer;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        transfer = new Transfer(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            if (gp1.isFirstA()) {
                run(OpmodeCommands.turnOnTransfer(transfer));
            }
            if (gp1.isFirstB()) {
                run(OpmodeCommands.turnOffTransfer(transfer));
            }
            if (gp1.isFirstX()) {
                run(OpmodeCommands.reverseTransfer(transfer));
            }

            Scheduler.execute();
            transfer.update();

            telemetry.addData("state", transfer.getTransferState());
            telemetry.update();
        }

        Scheduler.reset();
    }

    private void run(Command command) {
        Scheduler.schedule(command);
    }
}
