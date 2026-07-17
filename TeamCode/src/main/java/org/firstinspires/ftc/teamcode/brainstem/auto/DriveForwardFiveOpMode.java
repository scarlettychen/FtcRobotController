package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoScheduler;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Smoke test: {@link RobotActions#driveForwardFive()} drives 5″ along the current robot heading.
 */
@Autonomous(name = "Pedro Drive Forward 5in", group = "Pedro")
public class DriveForwardFiveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions actions = RobotActions.forSmokeTest(robot);
        actions.getDrive().setExternalLoop(true);

        AutoScheduler scheduler = new AutoScheduler();
        AutoCommand move = actions.driveForwardFive();
        AutoCommand side = actions.side();

        telemetry.addLine("Pedro + Pinpoint: forwardDrive 5in along robot heading");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        robot.update();
        Pose bakeStart = robot.follower.getPose();
//        scheduler.schedule(move);
        scheduler.schedule(side);
        while (opModeIsActive() && scheduler.isRunning()) {
            robot.update();
            scheduler.run();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(com.pedropathing.ftc.FTCCoordinates.INSTANCE);
            telemetry.addData("bake start", "(%.1f, %.1f, %.0f°)",
                    bakeStart.getX(), bakeStart.getY(), Math.toDegrees(bakeStart.getHeading()));
            telemetry.addData("field x", field.getX());
            telemetry.addData("field y", field.getY());
            telemetry.addData("pedro x", pedro.getX());
            telemetry.addData("pedro y", pedro.getY());
            telemetry.addData("pedroH deg", Math.toDegrees(pedro.getHeading()));
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.update();
        }

        scheduler.cancel();
        robot.follower.breakFollowing();
    }
}
