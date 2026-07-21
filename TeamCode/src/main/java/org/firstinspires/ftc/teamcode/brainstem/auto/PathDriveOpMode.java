package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * OpMode for {@link PathDriveAuto}: pathDrive through a few points.
 *
 * <pre>
 * (0,0,0°) → (24,0,0°) → (24,16,90°) → (40,16,90°)
 * </pre>
 */
@Autonomous(name = "Pedro Path Drive", group = "Pedro")
public class PathDriveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions actions = RobotActions.forSmokeTest(robot);
        PathDriveAuto auto = new PathDriveAuto(robot, actions);
        auto.setExternalLoop(true);
        actions.getDrive().settleEnd(false);

        telemetry.addLine("Pedro Path Drive");
        telemetry.addLine("START (0,0,0) → A(24,0,0) → B(24,16,90) → C(40,16,90)");
        telemetry.addLine("Place robot at field origin, facing field +X (0°)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(auto.getStartPose());
        auto.start();

        while (opModeIsActive() && !auto.isFinished()) {
            robot.update();
            auto.update();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            telemetry.addData("field", "(%.1f, %.1f, %.0f°)",
                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
            telemetry.addData("pedro", "(%.1f, %.1f, %.0f°)",
                    pedro.getX(), pedro.getY(), Math.toDegrees(pedro.getHeading()));
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.addData("finished", auto.isFinished());
            telemetry.update();
        }

        auto.stop();
        robot.follower.breakFollowing();

        Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        telemetry.addLine("Done — target C (40, 16, 90°)");
        telemetry.addData("final field", "(%.1f, %.1f, %.0f°)",
                field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
        telemetry.update();
        sleep(1000);
    }
}
