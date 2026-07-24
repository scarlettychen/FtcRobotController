package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;

// hold a = spin lift @ testing power, watch encoder direction
@TeleOp(name = "lift motor test", group = "Test")
public class FourBarMotorTestTele extends LinearOpMode {

    private FourBarLinkage lift;

    @Override
    public void runOpMode() {
        lift = new FourBarLinkage(hardwareMap, telemetry);
        lift.setState(FourBarLinkage.LinkState.OFF);

        telemetry.addLine("hold a = testing power. enc should rise for raise");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                lift.setState(FourBarLinkage.LinkState.TESTING);
            } else {
                lift.setState(FourBarLinkage.LinkState.OFF);
            }

            lift.update();

            telemetry.addData("enc", lift.getPosition());
            telemetry.addData("power", lift.getLastPower());
            telemetry.addData("MOTOR_SIGN", FourBarLinkage.MOTOR_SIGN);
            telemetry.addLine("if a raises enc the wrong way, flip MOTOR_SIGN to -1");
            telemetry.update();
        }

        lift.setState(FourBarLinkage.LinkState.OFF);
        lift.update();
    }
}
