package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoScheduler;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Mild Bezier: ~48″ forward with a small leftward curve (constant heading).
 * Time-optimal + RobotModel FF.
 */
@Autonomous(name = "Pedro Bezier 48in", group = "Pedro")
public class BezierSplineOpMode extends LinearOpMode {

    /** Chord length along robot forward (inches). */
    public static final double DISTANCE_INCHES = 48.0;
    /** Mid-path offset to the left (inches). Keep small for a gentle spline. */
    public static final double BULGE_INCHES = 4.0;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);

        AutoScheduler scheduler = new AutoScheduler();
        AutoCommand move = drive.bezierForwardDrive(DISTANCE_INCHES, BULGE_INCHES);

        telemetry.addLine("Bezier spline ~" + DISTANCE_INCHES + " in forward");
        telemetry.addLine("Bulge " + BULGE_INCHES + " in left");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Fresh Pinpoint → Pedro before the command bakes its Bezier from live pose.
        robot.update();
        Pose bakeStart = robot.follower.getPose();
        double h = bakeStart.getHeading();
        Pose bakeEnd = new Pose(
                bakeStart.getX() + DISTANCE_INCHES * Math.cos(h),
                bakeStart.getY() + DISTANCE_INCHES * Math.sin(h),
                h
        );

        scheduler.schedule(move);
        while (opModeIsActive() && scheduler.isRunning()) {
            robot.update();
            scheduler.run();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

            telemetry.addData("bake start", "(%.1f, %.1f, %.0f°)",
                    bakeStart.getX(), bakeStart.getY(), Math.toDegrees(bakeStart.getHeading()));
            telemetry.addData("bake end", "(%.1f, %.1f)", bakeEnd.getX(), bakeEnd.getY());
            telemetry.addData("delta in", "%.1f",
                    Math.hypot(pedro.getX() - bakeStart.getX(), pedro.getY() - bakeStart.getY()));
            telemetry.addData("fieldX", "%.2f", field.getX());
            telemetry.addData("fieldY", "%.2f", field.getY());
            telemetry.addData("pedroX", "%.2f", pedro.getX());
            telemetry.addData("pedroY", "%.2f", pedro.getY());
            telemetry.addData("pedroH deg", "%.1f", Math.toDegrees(pedro.getHeading()));
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.update();
        }

        scheduler.cancel();
        robot.follower.breakFollowing();

        Pose pedro = robot.pinpoint.getPose();
        telemetry.addLine("Done");
        telemetry.addData("bake end", "(%.1f, %.1f)", bakeEnd.getX(), bakeEnd.getY());
        telemetry.addData("final pedro", "(%.1f, %.1f)", pedro.getX(), pedro.getY());
        telemetry.addData("travel in", "%.1f",
                Math.hypot(pedro.getX() - bakeStart.getX(), pedro.getY() - bakeStart.getY()));
        telemetry.update();
        sleep(1500);
    }
}
