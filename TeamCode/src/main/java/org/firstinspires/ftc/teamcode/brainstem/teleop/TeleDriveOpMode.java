package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Mecanum teleop via Pedro: left stick drive, right stick turn.
 *
 * <ul>
 *   <li>Left stick Y — forward / back</li>
 *   <li>Left stick X — strafe</li>
 *   <li>Right stick X — turn</li>
 *   <li>Right bumper — slow mode (40%)</li>
 *   <li>A — robot-centric (default)</li>
 *   <li>B — field-centric</li>
 *   <li>Options / start — zero heading for field-centric</li>
 * </ul>
 */
@TeleOp(name = "Pedro Tele Drive", group = "Pedro")
public class TeleDriveOpMode extends LinearOpMode {

    private static final double SLOW_SCALE = 0.4;

    private boolean robotCentric = true;
    private boolean aWasPressed;
    private boolean bWasPressed;
    private boolean optionsWasPressed;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        Follower follower = robot.follower;

        telemetry.addLine("Pedro Tele Drive");
        telemetry.addLine("LS drive | RS turn | RB slow");
        telemetry.addLine("A robot-centric | B field-centric");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.a && !aWasPressed) {
                robotCentric = true;
            }
            if (gamepad1.b && !bWasPressed) {
                robotCentric = false;
            }
            if (optionsPressed() && !optionsWasPressed) {
                // Absolute stamp — setStartPose rebases and corrupts XY on repeat presses.
                Pose zeroed = new Pose(robot.pinpoint.getPose().getX(), robot.pinpoint.getPose().getY(), 0);
                robot.pinpoint.setPose(zeroed);
                robot.follower.setStartingPose(zeroed);
            }
            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;
            optionsWasPressed = optionsPressed();

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            double forward = -gamepad1.left_stick_y * scale;
            double strafe = -gamepad1.left_stick_x * scale;
            double turn = -gamepad1.right_stick_x * scale;

            follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
            follower.update();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

            telemetry.addData("mode", robotCentric ? "robot-centric" : "field-centric");
            telemetry.addData("slow", gamepad1.right_bumper);
            telemetry.addData("fwd", "%.2f", forward);
            telemetry.addData("str", "%.2f", strafe);
            telemetry.addData("turn", "%.2f", turn);
            telemetry.addData("fieldX", "%.2f", field.getX());
            telemetry.addData("fieldY", "%.2f", field.getY());
            telemetry.addData("fieldH deg", "%.1f", Math.toDegrees(field.getHeading()));
            telemetry.update();
        }
    }

    private boolean optionsPressed() {
        return gamepad1.options || gamepad1.start;
    }
}
