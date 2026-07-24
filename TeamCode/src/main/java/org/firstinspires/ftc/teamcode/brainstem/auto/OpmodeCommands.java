package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.PedroDrive;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;

public final class OpmodeCommands {
    private OpmodeCommands() {}

    // ---- intake ----
    public static Command turnOnIntake(Intake intake) {
        return setIntake(intake, Intake.IntakeState.IN);
    }

    public static Command turnOffIntake(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OFF);
    }

    public static Command reverseIntake(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OUT);
    }

    public static Command setIntake(Intake intake, Intake.IntakeState state) {
        return Commands.instant(() -> intake.setIntakeState(state)).requiring(intake);
    }

    // ---- transfer ----
    public static Command turnOnTransfer(Transfer transfer) {
        return setTransfer(transfer, Transfer.TransferState.IN);
    }

    public static Command turnOffTransfer(Transfer transfer) {
        return setTransfer(transfer, Transfer.TransferState.OFF);
    }

    public static Command reverseTransfer(Transfer transfer) {
        return setTransfer(transfer, Transfer.TransferState.OUT);
    }

    public static Command setTransfer(Transfer transfer, Transfer.TransferState state) {
        return Commands.instant(() -> transfer.setTransferState(state)).requiring(transfer);
    }

    // ---- blocker ----
    public static Command openBlocker(Blocker blocker) {
        return Commands.instant(blocker::setOpen).requiring(blocker);
    }

    public static Command closeBlocker(Blocker blocker) {
        return Commands.instant(blocker::setDown).requiring(blocker);
    }

    public static Command setBlocker(Blocker blocker, Blocker.ServoState state) {
        return Commands.instant(() -> blocker.setState(state)).requiring(blocker);
    }

    // ---- lift ----
    public static Command setLiftDown(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.DOWN);
    }

    public static Command setLiftLow(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command setLiftHigh(FourBarLinkage lift) {
        return setLift(lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command setLift(FourBarLinkage lift, FourBarLinkage.LinkState state) {
        return Commands.instant(() -> lift.setState(state)).requiring(lift);
    }

    public static Command waitLiftAtTarget(FourBarLinkage lift) {
        return Commands.waitUntil(lift::atTarget).requiring(lift);
    }

    public static Command waitLiftAtTarget(
            FourBarLinkage lift, FourBarLinkage.LinkState expected) {
        return Commands.waitUntil(
                () -> lift.getState() == expected && lift.atTarget()
        ).requiring(lift);
    }

    // intake + transfer on together
    public static Command turnOnIntakeAndTransfer(Intake intake, Transfer transfer) {
        return Commands.instant(() -> {
            intake.setIntakeState(Intake.IntakeState.IN);
            transfer.setTransferState(Transfer.TransferState.IN);
        }).requiring(intake, transfer);
    }

    public static Command turnOffIntakeAndTransfer(Intake intake, Transfer transfer) {
        return Commands.instant(() -> {
            intake.setIntakeState(Intake.IntakeState.OFF);
            transfer.setTransferState(Transfer.TransferState.OFF);
        }).requiring(intake, transfer);
    }

    // go to {x, y, headingDeg} via pedro pathdrive
    public static Command driveTo(PedroDrive drive, double[] fieldPose) {
        return Command.build()
                .setStart(() -> drive.pathDrive(fieldPose))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(endCondition -> {
                    if (drive.isBusy()) {
                        drive.getFollower().breakFollowing();
                    }
                })
                .requiring(drive);
    }

    public static Command driveTo(PedroDrive drive, double x, double y, double headingDegrees) {
        return driveTo(drive, new double[]{x, y, headingDegrees});
    }

    // chase closest limelight ball w/ pid on tx + range (teleop drive, no path stuff)
    // done when close+centered or we lost the ball
    public static Command driveToClosestBall(PedroDrive drive, Limelight limelight) {
        final Follower follower = drive.getFollower();
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final int[] lostFrames = {0};
        final boolean[] done = {false};

        return Command.build()
                .setStart(() -> {
                    follower.startTeleopDrive();
                    turnPid.reset();
                    rangePid.reset();
                    lostFrames[0] = 0;
                    done[0] = false;
                })
                .setExecute(() -> {
                    limelight.update();
                    turnPid.setGains(
                            Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
                    rangePid.setGains(
                            Limelight.CHASE_KP_RANGE,
                            Limelight.CHASE_KI_RANGE,
                            Limelight.CHASE_KD_RANGE);
                    turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
                    rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);

                    Limelight.BallDetection ball = limelight.getClosestBall();
                    if (ball == null) {
                        lostFrames[0]++;
                        follower.setTeleOpDrive(0, 0, 0, true);
                        follower.update();
                        if (lostFrames[0] >= Limelight.CHASE_LOST_FRAMES) {
                            done[0] = true;
                        }
                        return;
                    }
                    lostFrames[0] = 0;

                    double range = limelight.estimateRangeInches(ball.tyDeg);
                    // +tx = ball right → turn right (neg in tele stick land)
                    double turn = -turnPid.calculate(ball.txDeg, 0.0);
                    double forward = 0.0;
                    if (!Double.isNaN(range)) {
                        // range - stop > 0 means we still gotta go
                        forward = rangePid.calculate(
                                Limelight.CHASE_STOP_RANGE_IN, range);
                        if (forward < 0) {
                            forward = 0;
                        }
                    }

                    follower.setTeleOpDrive(forward, 0, turn, true);
                    follower.update();

                    boolean aimed = Math.abs(ball.txDeg) <= Limelight.CHASE_TX_TOL_DEG;
                    boolean close = !Double.isNaN(range) && range <= Limelight.CHASE_STOP_RANGE_IN;
                    if (aimed && close) {
                        done[0] = true;
                    }
                })
                .setDone(() -> done[0])
                .setEnd(endCondition -> {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    follower.update();
                })
                .requiring(drive, limelight);
    }

    // intake off, transfer on, lift high. doesnt open blocker
    public static Command setLiftToHigh(Intake intake, Transfer transfer, FourBarLinkage lift) {
        return raiseLift(intake, transfer, lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    // same but lift low
    public static Command setLiftToLow(Intake intake, Transfer transfer, FourBarLinkage lift) {
        return raiseLift(intake, transfer, lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    // raise + transfer, open blocker once lift hits target (no goal-region check)
    public static Command raiseAndScore(
            Intake intake,
            Transfer transfer,
            FourBarLinkage lift,
            FourBarLinkage.LinkState scoreHeight
    ) {
        // one instant — ivy parallel-of-instants was skipping lift/blocker side effects
        return Groups.sequential(
                Commands.instant(() -> {
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    transfer.setTransferState(Transfer.TransferState.IN);
                    lift.setState(scoreHeight);
                }).requiring(intake, transfer, lift),
                waitLiftAtTarget(lift, scoreHeight)
        );
    }

    public static Command raiseAndScoreHigh(
            Intake intake, Transfer transfer, FourBarLinkage lift, Blocker blocker) {
        return raiseAndScore(
                intake, transfer, lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command raiseAndScoreLow(
            Intake intake, Transfer transfer, FourBarLinkage lift, Blocker blocker) {
        return raiseAndScore(
                intake, transfer, lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    // wait til blocker is over the goal box then yeet it open
    public static Command openBlockerWhenOverGoal(
            PedroDrive drive,
            Blocker blocker,
            double goalX,
            double goalY,
            double halfWidth,
            double halfHeight
    ) {
        return Groups.sequential(
                Commands.waitUntil(() -> blocker.isOverRegion(
                        drive.getPose(), goalX, goalY, halfWidth, halfHeight)),
                openBlocker(blocker)
        ).requiring(blocker);
    }

    // drive + raise at the same time, open blocker only when its over the goal
    public static Command driveRaiseAndScore(
            PedroDrive drive,
            Intake intake,
            Transfer transfer,
            FourBarLinkage lift,
            Blocker blocker,
            double[] fieldPose,
            FourBarLinkage.LinkState scoreHeight,
            double goalX,
            double goalY,
            double halfWidth,
            double halfHeight
    ) {
        return Groups.sequential(
                Groups.parallel(
                        driveTo(drive, fieldPose),
                        raiseLift(intake, transfer, lift, scoreHeight)
                ),
                openBlockerWhenOverGoal(drive, blocker, goalX, goalY, halfWidth, halfHeight)
        );
    }

    public static Command driveRaiseAndScoreHigh(
            PedroDrive drive,
            Intake intake,
            Transfer transfer,
            FourBarLinkage lift,
            Blocker blocker,
            double[] fieldPose,
            double goalX,
            double goalY,
            double halfWidth,
            double halfHeight
    ) {
        return driveRaiseAndScore(
                drive, intake, transfer, lift, blocker, fieldPose,
                FourBarLinkage.LinkState.SCORE_HIGH,
                goalX, goalY, halfWidth, halfHeight);
    }

    public static Command driveRaiseAndScoreLow(
            PedroDrive drive,
            Intake intake,
            Transfer transfer,
            FourBarLinkage lift,
            Blocker blocker,
            double[] fieldPose,
            double goalX,
            double goalY,
            double halfWidth,
            double halfHeight
    ) {
        return driveRaiseAndScore(
                drive, intake, transfer, lift, blocker, fieldPose,
                FourBarLinkage.LinkState.SCORE_LOW,
                goalX, goalY, halfWidth, halfHeight);
    }

    // everything chill: intake/transfer off, lift down, blocker down
    public static Command resetAll(
            Intake intake, Transfer transfer, FourBarLinkage lift, Blocker blocker) {
        return Commands.instant(() -> {
            intake.setIntakeState(Intake.IntakeState.OFF);
            transfer.setTransferState(Transfer.TransferState.OFF);
            lift.setState(FourBarLinkage.LinkState.DOWN);
            blocker.setDown();
        }).requiring(intake, transfer, lift, blocker);
    }

    // collect vibe: lift down, suck on, blocker down
    public static Command resetAndCollect(
            Intake intake, Transfer transfer, FourBarLinkage lift, Blocker blocker) {
        return Commands.instant(() -> {
            lift.setState(FourBarLinkage.LinkState.DOWN);
            intake.setIntakeState(Intake.IntakeState.IN);
            transfer.setTransferState(Transfer.TransferState.IN);
            blocker.setDown();
        }).requiring(intake, transfer, lift, blocker);
    }

    private static Command raiseLift(
            Intake intake,
            Transfer transfer,
            FourBarLinkage lift,
            FourBarLinkage.LinkState scoreState
    ) {
        return Groups.sequential(
                Commands.instant(() -> {
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    transfer.setTransferState(Transfer.TransferState.IN);
                    lift.setState(scoreState);
                }).requiring(intake, transfer, lift),
                waitLiftAtTarget(lift, scoreState)
        );
    }
}
