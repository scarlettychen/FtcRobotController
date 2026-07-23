package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.PedroDrive;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;

// red line: start → end while raising lift, then transfer + open blocker
// poses are road runner / ftc field coords
@Autonomous(name = "Red Line Score", group = "Auto")
public class RedLineScoreAuto extends LinearOpMode {

    private static final double[] START = {-24, 9, 270};
    private static final double[] END = {60, 60, 270};

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(true);
        robot.setStartPose(START);

        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);
        drive.settleEnd(false);

        robot.blocker.setDown();
        robot.transfer.setTransferState(Transfer.TransferState.OFF);

        Command auto = Groups.sequential(
                OpmodeCommands.closeBlocker(robot.blocker),
                Groups.parallel(
                        OpmodeCommands.driveTo(drive, END),
                        Groups.sequential(
                                OpmodeCommands.setLiftHigh(robot.lift)
//                                , OpmodeCommands.waitLiftAtTarget(robot.lift)
                        )
                ),
                OpmodeCommands.turnOnTransfer(robot.transfer),
                OpmodeCommands.openBlocker(robot.blocker)
        );

        telemetry.addLine("Red Line Score");
        telemetry.addData("start", "(%.0f, %.0f, %.0f°)", START[0], START[1], START[2]);
        telemetry.addData("end", "(%.0f, %.0f, %.0f°)", END[0], END[1], END[2]);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Scheduler.reset();
        Scheduler.schedule(auto);

        while (opModeIsActive() && Scheduler.isScheduled(auto)) {
            robot.update();
            Scheduler.execute();

            Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            telemetry.addData("field", "(%.1f, %.1f, %.0f°)",
                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
            telemetry.addData("lift", robot.lift.getState());
            telemetry.addData("busy", drive.isBusy());
            telemetry.update();
        }

        Scheduler.reset();
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
