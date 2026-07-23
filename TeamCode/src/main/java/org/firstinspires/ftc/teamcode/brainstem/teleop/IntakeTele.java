package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// intake only. a = out, else off
@TeleOp(name = "intake + transfer non commands", group = "Test")
public class IntakeTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Intake intake;
    private Transfer transfer;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.setIntakeState(Intake.IntakeState.IN);
                transfer.setTransferState(Transfer.TransferState.IN);
            } else  if (gamepad1.b){
                intake.setIntakeState(Intake.IntakeState.OUT);
            } else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }

            intake.update();
            transfer.update();

            telemetry.addData("state", intake.getIntakeState());
            telemetry.update();

            gp1.update();
            gp2.update();
        }
    }
}
