package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;

// score at goal → reset/collect + lineTo balls → smart collect
@Configurable
@Autonomous(name = "wierd auton", group = "Auto")
public class Strafe extends LinearOpMode {
    public static double DRIVE_FORWARD =  - 86;
    public static double STRAFE_LEFT = 12;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(true);

        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);
        drive.settleEnd(false);
        drive.useTimeOptimal(true);

        robot.blocker.setDown();
        robot.transfer.setTransferState(Transfer.TransferState.OFF);

        Command auto = Groups.sequential(
                Groups.parallel(
                        OpmodeCommands.driveForward(drive, DRIVE_FORWARD),
                        OpmodeCommands.raiseAndScoreHigh(robot.intake, robot.transfer, robot.lift, robot.blocker)
                ),

                OpmodeCommands.strafeRight(drive, STRAFE_LEFT),
                Commands.waitMs(500),
                OpmodeCommands.driveForward(drive, -1.5),
                OpmodeCommands.turnOnTransferAndOpenBlocker(robot.transfer, robot.blocker, robot.intake),
                Commands.waitMs(1500),
                OpmodeCommands.driveForward(drive, 5),
                Commands.waitMs(1500),

                OpmodeCommands.resetAll(robot.intake, robot.transfer, robot.lift, robot.blocker)

        );

        telemetry.addLine("Red Line Score");
        telemetry.update();

//        for (int i = 0; i < 10 && opModeInInit(); i++) {
//            robot.update();
//            Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
//            telemetry.addData("field now", "(%.1f, %.1f, %.0f°)",
//                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
//            telemetry.update();
//            sleep(50);
//        }

        waitForStart();
        if (isStopRequested()) return;

        robot.update();

        Scheduler.reset();
        Scheduler.schedule(auto);

        while (opModeIsActive() && Scheduler.isScheduled(auto)) {
            robot.update();
            Scheduler.execute();

            Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            telemetry.addData("field", "(%.1f, %.1f, %.0f°)",
                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
            telemetry.addData("lift", "%s pos=%d atTarget=%s",
                    robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
            telemetry.addData("balls in robot", OpmodeCommands.getEstimatedBallsInRobot());
            telemetry.addData("busy", drive.isBusy());
            telemetry.update();
        }

        Scheduler.reset();
        robot.follower.breakFollowing();
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
