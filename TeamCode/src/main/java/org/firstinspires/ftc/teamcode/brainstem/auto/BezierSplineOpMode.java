package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoScheduler;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Field cubic S-curve Bezier from {@link #START} {@code (0,0,0°)} to {@link #END} {@code (48,0,0°)}.
 *
 * <p>Gentle S: {@link #C1} left (+Y), {@link #C2} right (−Y). Large opposite bulges on a short
 * chord fold the cubic so the path tangent points backward (looks like reverse). Keep offsets
 * small (~2–3″). Heading follows tangent; {@code turnTo(0)} finishes facing field +X.
 */
@Autonomous(name = "Pedro Bezier 0 to 48,0", group = "Pedro")
public class BezierSplineOpMode extends LinearOpMode {

    public static final double[] START = {0, 0, 0};
    /** ~1/3 along +X, mild left bulge. */
    public static final double[] C1 = {16, 2.5, 0};
    /** ~2/3 along +X, mild right bulge (opposite sign = S-curve). */
    public static final double[] C2 = {32, -2.5, 0};
    public static final double[] END = {48, 0, 0};

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions actions = RobotActions.forSmokeTest(robot);
        PedroDrive drive = actions.getDrive();
        drive.setExternalLoop(true);
        // Settle can yank the robot back if it cut the curve — off for this smoke test.
        drive.settleEnd(false);

        AutoCommand move = actions.sequence(
                drive.bezierDriveTangent(C1, C2, END),
                drive.turnTo(0)
        );

        AutoScheduler scheduler = new AutoScheduler();

        telemetry.addLine("Gentle S (0,0) → C1(16,2.5) → C2(32,-2.5) → (48,0)");
        telemetry.addLine("Tangent heading, then turnTo(0°). settleEnd=false");
        telemetry.addLine("Place robot at field origin, facing field +X (0°)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(START);
        robot.update();

        Pose bakeStart = robot.follower.getPose();
        Pose bakeEnd = AlliancePoses.toPose(END);
        double dx = bakeEnd.getX() - bakeStart.getX();
        double dy = bakeEnd.getY() - bakeStart.getY();
        double h = bakeStart.getHeading();
        double forward = dx * Math.cos(h) + dy * Math.sin(h);
        double right = dx * Math.sin(h) - dy * Math.cos(h);

        scheduler.schedule(move);
        while (opModeIsActive() && scheduler.isRunning()) {
            robot.update();
            scheduler.run();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

            telemetry.addData("bake start pedro", "(%.1f, %.1f, %.0f°)",
                    bakeStart.getX(), bakeStart.getY(), Math.toDegrees(bakeStart.getHeading()));
            telemetry.addData("bake end pedro", "(%.1f, %.1f, %.0f°)",
                    bakeEnd.getX(), bakeEnd.getY(), Math.toDegrees(bakeEnd.getHeading()));
            telemetry.addData("chord robotFwd", "%.1f in", forward);
            telemetry.addData("chord robotRight", "%.1f in", right);
            telemetry.addData("fieldX", "%.2f", field.getX());
            telemetry.addData("fieldY", "%.2f", field.getY());
            telemetry.addData("fieldH deg", "%.1f", Math.toDegrees(field.getHeading()));
            telemetry.addData("pedroH deg", "%.1f", Math.toDegrees(pedro.getHeading()));
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.update();
        }

        scheduler.cancel();
        robot.follower.breakFollowing();

        Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        telemetry.addLine("Done");
        telemetry.addData("final field", "(%.1f, %.1f, %.0f°)",
                field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
        telemetry.addData("target field", "(%.0f, %.0f, 0°)", END[0], END[1]);
        telemetry.update();
        sleep(1500);
    }
}
