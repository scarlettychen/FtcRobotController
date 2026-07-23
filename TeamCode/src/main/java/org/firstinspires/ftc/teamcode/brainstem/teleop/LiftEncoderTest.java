package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// four bar testing mode — check motors dont fight
// a = left @ 0.2  b = right @ 0.2  (hold; release = off)
// encoders should both move the same way when both on

@TeleOp(name = "Lift encoder test", group = "Test")
public class LiftEncoderTest extends LinearOpMode {


    private FourBarLinkage lift;

    @Override
    public void runOpMode() {

        lift = new FourBarLinkage(hardwareMap, telemetry);
        lift.setState(FourBarLinkage.LinkState.OFF);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift.setState(FourBarLinkage.LinkState.TESTING);



            telemetry.addData("encoder", lift.getRightPosition());
            telemetry.update();

            lift.update();


        }
    }
}
