package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;

/**
 * Red line score. Poses are {@link FieldCoords}: 0° = into field from −Y wall (+Y), CCW+.
 * <p>
 * Start sits on the −Y wall facing into the field (0°), drives to goal,
 * then to the ball pile. Use {@code driveTo} (not {@code lineTo}) when the
 * heading in the pose array must be applied.
 */
@Configurable
@Autonomous(name = "Red Line Score", group = "Auto")
public class RedLineScoreAuto extends LinearOpMode {

    // −Y wall, x≈−24; face into field (+Y = 0°)
    public static double[] START = FieldCoords.xyz(-24, -72 + 9, 0);
    // score area; face +Y
    public static double[] GOAL = FieldCoords.xyz(-20, 20, 0);
    // ball pile; face −Y for intake (180°). driveTo applies this heading.
    public static double[] BALLS = FieldCoords.xyz(-12, -12, 180);
    public static double STRAFE_LEFT_IN = 1.5;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(true);
        robot.setStartPose(START);

        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);
        drive.settleEnd(false);
        drive.useTimeOptimal(true);

        robot.blocker.setDown();
        robot.transfer.setTransferState(Transfer.TransferState.OFF);

        Command auto = Groups.sequential(
                OpmodeCommands.closeBlocker(robot.blocker),
                Groups.parallel(
                        OpmodeCommands.lineTo(drive, GOAL),
                        OpmodeCommands.setLiftHigh(robot.lift)
                ),
                OpmodeCommands.strafeLeft(drive, STRAFE_LEFT_IN),
                OpmodeCommands.turnOnTransferAndOpenBlocker(robot.transfer, robot.blocker, robot.intake),
                Groups.parallel(
                        OpmodeCommands.driveTo(drive, BALLS),
                        OpmodeCommands.resetAndCollect(
                                robot.intake, robot.transfer, robot.lift, robot.blocker)
                ),
                OpmodeCommands.collectBallsThenBackOff(
                        drive, robot.limelight, robot.intake, robot.transfer,
                        robot.intakeGate, true, 5, Limelight.COLLECT_BACK_OFF_IN, 10)
        );

        telemetry.addLine("Red Line Score");
        telemetry.addLine("FieldCoords: 0°=+Y (into field)  CCW+  walls±72");
        telemetry.addData("start", FieldCoords.format(START));
        telemetry.addData("goal", FieldCoords.format(GOAL));
        telemetry.addData("balls", FieldCoords.format(BALLS));
        telemetry.addData("strafe left in", STRAFE_LEFT_IN);
        telemetry.update();

        for (int i = 0; i < 10 && opModeInInit(); i++) {
            robot.update();
            Pose field = robot.pinpoint.getPose()
                    .getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            telemetry.addData("field now", FieldCoords.format(field));
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(START);
        robot.update();

        Scheduler.reset();
        Scheduler.schedule(auto);

        while (opModeIsActive() && Scheduler.isScheduled(auto)) {
            robot.update();
            Scheduler.execute();

            Pose field = robot.pinpoint.getPose()
                    .getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            telemetry.addData("field", FieldCoords.format(field));
            telemetry.addData("balls tgt", FieldCoords.format(BALLS));
            telemetry.addData("lift", "%s pos=%d atTarget=%s",
                    robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
            telemetry.addData("balls in robot", OpmodeCommands.getEstimatedBallsInRobot());
            robot.intakeGate.addTelemetry();
            telemetry.addData("busy", drive.isBusy());
            telemetry.update();
        }

        Scheduler.reset();
        robot.follower.breakFollowing();
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
