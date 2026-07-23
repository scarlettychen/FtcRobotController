package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// four bar testing mode — check motors dont fight
// a = left @ 0.2  b = right @ 0.2  (hold; release = off)
// encoders should both move the same way when both on

@TeleOp(name = "lift motor test", group = "Test")
public class FourBarMotorTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private FourBarLinkage lift;

    @Override
    public void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        lift = new FourBarLinkage(hardwareMap, telemetry);
        lift.setState(FourBarLinkage.LinkState.TESTING);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift.setState(FourBarLinkage.LinkState.TESTING);
            FourBarLinkage.rightMotorOn = gamepad1.b;

            lift.update();

            telemetry.addData("right on (b)", FourBarLinkage.rightMotorOn);
            telemetry.addData("right enc", lift.getRightPosition());
            telemetry.addData("hint", "both on → encoders should match direction");
            telemetry.update();

            gp1.update();
            gp2.update();
        }
    }
}
