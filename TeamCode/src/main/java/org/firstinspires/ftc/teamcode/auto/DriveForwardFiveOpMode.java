package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoScheduler;
import com.pedropathing.auto.PedroDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;

/**
 * Smoke test: start at (0, 0, 0°) and drive forward 5 inches via a named action.
 * No raw {@code lineDrive(double[])} — use {@link RobotActions#driveForwardFive()}.
 */
@Autonomous(name = "Pedro Drive Forward 5in", group = "Pedro")
public class DriveForwardFiveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions actions = RobotActions.forSmokeTest(new PedroDrive(robot.follower));
        actions.getDrive().setExternalLoop(true);
        actions.getDrive().setStartPose(actions.poses().start);

        AutoScheduler scheduler = new AutoScheduler();
        AutoCommand move = actions.driveForwardFive();

        telemetry.addLine("Pedro: driveForwardFive (0,0,0) → +5 in X");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        scheduler.schedule(move);
        while (opModeIsActive() && scheduler.isRunning()) {
            // optional: robot.syncPose from RR/Pinpoint
            robot.update();
            scheduler.run();
            telemetry.update();
        }
        scheduler.cancel();
        robot.follower.breakFollowing();
    }
}
