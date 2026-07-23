package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// intake only via commands. a = on, b = off, x = reverse
@TeleOp(name = "intake + transfer commands", group = "Test")
public class IntakeTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Intake intake;
    private Transfer transfer;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap,telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            if (gp1.isFirstA()) {
               run(OpmodeCommands.turnOnIntakeAndTransfer(intake, transfer));
            }
            if (gp1.isFirstB()) {
                run(OpmodeCommands.turnOffIntake(intake));
                run(OpmodeCommands.turnOffTransfer(transfer));
            }
            if (gp1.isFirstX()) {
                run(OpmodeCommands.reverseIntake(intake));
            }

            Scheduler.execute();
            intake.update();
            transfer.update();

            telemetry.addData("state", intake.getIntakeState());
            telemetry.update();
        }

        Scheduler.reset();
    }

    private void run(Command command) {
        Scheduler.schedule(command);
    }
}
