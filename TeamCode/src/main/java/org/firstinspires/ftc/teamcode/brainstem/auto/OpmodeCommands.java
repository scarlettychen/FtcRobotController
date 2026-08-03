package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathSpec;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.IntakeBeamBreak;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;

import java.util.concurrent.atomic.AtomicBoolean;

public final class OpmodeCommands {
    private OpmodeCommands() {}

    /** Balls counted by intake beam breaks (synced from {@link IntakeBeamBreak}). */
    private static int estimatedBallsInRobot = 0;

    public static int getEstimatedBallsInRobot() {
        return estimatedBallsInRobot;
    }

    public static void resetEstimatedBallsInRobot() {
        estimatedBallsInRobot = 0;
    }

    public static void syncBallCount(IntakeBeamBreak intakeGate) {
        if (intakeGate != null) {
            estimatedBallsInRobot = intakeGate.getBallCount();
        }
    }

    // ---- intake ----
    public static Command turnOnIntake(Intake intake, FourBarLinkage lift) {

        if (lift.state == FourBarLinkage.LinkState.DOWN){
            return setIntake(intake, Intake.IntakeState.IN);
        } else {
            return setIntake(intake, Intake.IntakeState.OFF);
        }

    }
    public static Command turnOnIntakeEx(Intake intake) {
        return setIntake(intake, Intake.IntakeState.OUT);
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

    // optional: wait until encoder is at the requested state (setLift alone is usually enough)
    public static Command holdLift(FourBarLinkage lift, FourBarLinkage.LinkState state) {
        return Command.build()
                .setStart(() -> lift.setState(state))
                .setExecute(() -> lift.setState(state))
                .setDone(() -> lift.getState() == state && lift.atTarget())
                .requiring(lift);
    }

    public static Command holdLiftHigh(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.SCORE_HIGH);
    }

    public static Command holdLiftLow(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.SCORE_LOW);
    }

    public static Command holdLiftDown(FourBarLinkage lift) {
        return holdLift(lift, FourBarLinkage.LinkState.DOWN);
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

    // transfer + open blocker in one instant (avoid parallel-of-instants)
    public static Command turnOnTransferAndOpenBlocker(Transfer transfer, Blocker blocker, Intake intake) {
        return Commands.instant(() -> {
            transfer.setTransferState(Transfer.TransferState.IN);
            blocker.setOpen();
            intake.setIntakeState(Intake.IntakeState.OUT);
        }).requiring(transfer, blocker, intake);
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

    public static Command extakeIntakeAndTransfer(Intake intake, Transfer transfer) {
        return Commands.instant(() -> {
            intake.setIntakeState(Intake.IntakeState.OUT);
            transfer.setTransferState(Transfer.TransferState.OUT);
        }).requiring(intake, transfer);
    }

    // ---- drive (PathFollower / PathSpec — no Pedro types) ----

    /** Go to {x, y, headingDeg}; applies end heading (HOLD). */
    public static Command driveTo(PathFollower drive, double[] fieldPose) {
        return Command.build()
                .setStart(() -> drive.startPath(PathSpec.lineTo(
                        "driveTo", drive.getFieldPose(), fieldPose, PathSpec.HeadingMode.HOLD)))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(end -> { if (drive.isBusy()) drive.cancel(); })
                .requiring(drive);
    }

    public static Command driveTo(PathFollower drive, double x, double y, double headingDegrees) {
        return driveTo(drive, new double[]{x, y, headingDegrees});
    }

    /** Straight line to field x,y — holds start heading. */
    public static Command lineTo(PathFollower drive, double[] fieldPose) {
        return Command.build()
                .setStart(() -> drive.startPath(PathSpec.lineTo(
                        "lineTo", drive.getFieldPose(), fieldPose, PathSpec.HeadingMode.HOLD_START)))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(end -> { if (drive.isBusy()) drive.cancel(); })
                .requiring(drive);
    }

    public static Command lineTo(PathFollower drive, double x, double y, double headingDegrees) {
        return lineTo(drive, new double[]{x, y, headingDegrees});
    }

    public static Command driveForward(PathFollower drive, double inches) {
        return Command.build()
                .setStart(() -> drive.startPath(PathSpec.forward("forward", drive.getFieldPose(), inches)))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(end -> { if (drive.isBusy()) drive.cancel(); })
                .requiring(drive);
    }

    public static Command driveBack(PathFollower drive, double inches) {
        return driveForward(drive, -Math.abs(inches));
    }

    /** Robot-relative strafe (positive = left). */
    public static Command strafe(PathFollower drive, double inches) {
        return Command.build()
                .setStart(() -> drive.startPath(PathSpec.strafe("strafe", drive.getFieldPose(), inches)))
                .setExecute(drive::update)
                .setDone(() -> !drive.isBusy())
                .setEnd(end -> { if (drive.isBusy()) drive.cancel(); })
                .requiring(drive);
    }

    public static Command strafeLeft(PathFollower drive, double inches) {
        return strafe(drive, Math.abs(inches));
    }

    public static Command strafeRight(PathFollower drive, double inches) {
        return strafe(drive, -Math.abs(inches));
    }

    public static Command driveToClosestBall(
            PathFollower drive, Limelight limelight, boolean red) {
        return driveToClosestBall(drive, limelight, red, true, Limelight.MAX_CHASE_TIME_MS, null);
    }

    public static Command driveToClosestBall(
            PathFollower drive, Limelight limelight, boolean red, boolean finishOnLost) {
        return driveToClosestBall(drive, limelight, red, finishOnLost, Limelight.MAX_CHASE_TIME_MS, null);
    }

    public static Command driveToClosestBall(
            PathFollower drive, Limelight limelight, boolean red, boolean finishOnLost, double maxChaseTimeMs) {
        return driveToClosestBall(drive, limelight, red, finishOnLost, maxChaseTimeMs, null);
    }

    public static Command driveToClosestBall(
            PathFollower drive,
            Limelight limelight,
            boolean red,
            boolean finishOnLost,
            double maxChaseTimeMs,
            AtomicBoolean reachedBall) {
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final int[] lostFrames = {0};
        final boolean[] done = {false};
        final long[] startMs = {0};

        return Command.build()
                .setStart(() -> {
                    drive.startManualDrive();
                    turnPid.reset();
                    rangePid.reset();
                    lostFrames[0] = 0;
                    done[0] = false;
                    startMs[0] = System.currentTimeMillis();
                    if (reachedBall != null) reachedBall.set(false);
                })
                .setExecute(() -> {
                    if (done[0]) return;
                    if (System.currentTimeMillis() - startMs[0] >= maxChaseTimeMs) {
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        done[0] = true;
                        return;
                    }
                    double[] field = drive.getFieldPose();
                    if (Limelight.isOutOfBounds(field[0], field[1], red)) {
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        done[0] = true;
                        return;
                    }
                    limelight.update();
                    turnPid.setGains(Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
                    rangePid.setGains(Limelight.CHASE_KP_RANGE, Limelight.CHASE_KI_RANGE, Limelight.CHASE_KD_RANGE);
                    turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
                    rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);
                    Limelight.BallDetection ball = limelight.getClosestBall();
                    if (ball == null) {
                        lostFrames[0]++;
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        if (finishOnLost && lostFrames[0] >= Limelight.CHASE_LOST_FRAMES) done[0] = true;
                        return;
                    }
                    lostFrames[0] = 0;
                    double range = limelight.estimateRangeInches(ball.tyDeg);
                    double turn = -turnPid.calculate(ball.txDeg, 0.0);
                    double forward = 0.0;
                    if (!Double.isNaN(range)) {
                        forward = rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, range);
                        if (forward < 0) forward = 0;
                    }
                    drive.setManualDrive(forward, 0, turn);
                    drive.update();
                    boolean aimed = Math.abs(ball.txDeg) <= Limelight.CHASE_TX_TOL_DEG;
                    boolean close = !Double.isNaN(range) && range <= Limelight.CHASE_STOP_RANGE_IN;
                    if (aimed && close) {
                        if (reachedBall != null) reachedBall.set(true);
                        done[0] = true;
                    }
                })
                .setDone(() -> done[0])
                .setEnd(end -> {
                    drive.setManualDrive(0, 0, 0);
                    drive.update();
                    drive.cancel();
                })
                .requiring(drive, limelight);
    }

    public static Command collectBallsThenBackOff(
            PathFollower drive, Limelight limelight, Intake intake, Transfer transfer,
            IntakeBeamBreak intakeGate, boolean red) {
        return collectBallsThenBackOff(
                drive, limelight, intake, transfer, intakeGate, red,
                Limelight.COLLECT_COUNT, Limelight.COLLECT_BACK_OFF_IN,
                Limelight.MAX_CHASE_TIME_MS / 1000.0);
    }

    public static Command collectBallsThenBackOff(
            PathFollower drive, Limelight limelight, Intake intake, Transfer transfer,
            IntakeBeamBreak intakeGate, boolean red, int ballCount, double backInches,
            double timeoutSeconds) {
        return Groups.sequential(
                smartCollect(drive, limelight, intake, transfer, intakeGate, ballCount, timeoutSeconds),
                Groups.race(driveBack(drive, backInches), Commands.waitMs(2500))
        );
    }

    public static Command huntAndVerifyBall(
            PathFollower drive, Limelight limelight, int[] possessedBalls) {
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final int[] lostFrames = {0};
        final boolean[] done = {false};
        final boolean[] wasInStrikeZone = {false};

        return Command.build()
                .setStart(() -> {
                    drive.startManualDrive();
                    turnPid.reset();
                    rangePid.reset();
                    lostFrames[0] = 0;
                    done[0] = false;
                    wasInStrikeZone[0] = false;
                })
                .setExecute(() -> {
                    limelight.update();
                    turnPid.setGains(Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
                    rangePid.setGains(Limelight.CHASE_KP_RANGE, Limelight.CHASE_KI_RANGE, Limelight.CHASE_KD_RANGE);
                    turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
                    rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);
                    Limelight.BallDetection ball = limelight.getClosestBall();
                    if (ball == null) {
                        lostFrames[0]++;
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        if (wasInStrikeZone[0]) {
                            possessedBalls[0]++;
                            done[0] = true;
                            return;
                        }
                        if (lostFrames[0] >= Limelight.CHASE_LOST_FRAMES) done[0] = true;
                        return;
                    }
                    lostFrames[0] = 0;
                    double range = limelight.estimateRangeInches(ball.tyDeg);
                    boolean aimed = Math.abs(ball.txDeg) <= Limelight.CHASE_TX_TOL_DEG;
                    boolean close = !Double.isNaN(range) && range <= (Limelight.CHASE_STOP_RANGE_IN + 2.0);
                    wasInStrikeZone[0] = aimed && close;
                    double turn = -turnPid.calculate(ball.txDeg, 0.0);
                    double forward = 0.0;
                    if (!Double.isNaN(range)) {
                        forward = rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, range);
                        if (forward < 0) forward = 0;
                    }
                    drive.setManualDrive(forward, 0, turn);
                    drive.update();
                })
                .setDone(() -> done[0])
                .setEnd(end -> {
                    drive.setManualDrive(0, 0, 0);
                    drive.update();
                    drive.cancel();
                })
                .requiring(drive, limelight);
    }

    public static Command smartCollect(
            PathFollower drive, Limelight limelight, Intake intake, Transfer transfer,
            IntakeBeamBreak intakeGate, int targetCount, double timeoutSeconds) {
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final long[] startMs = {0};
        final boolean[] finished = {false};
        final double[] lastForward = {0.15};
        final long timeoutMs = Math.max(1L, (long) (timeoutSeconds * 1000.0));

        Command chase = Command.build()
                .setStart(() -> {
                    drive.startManualDrive();
                    turnPid.reset();
                    rangePid.reset();
                    intakeGate.resetCount();
                    estimatedBallsInRobot = 0;
                    startMs[0] = System.currentTimeMillis();
                    finished[0] = false;
                    lastForward[0] = 0.15;
                    intake.setIntakeState(Intake.IntakeState.IN);
                    transfer.setTransferState(Transfer.TransferState.IN);
                })
                .setExecute(() -> {
                    if (finished[0]) return;
                    syncBallCount(intakeGate);
                    long elapsed = System.currentTimeMillis() - startMs[0];
                    if (intakeGate.getBallCount() >= targetCount || elapsed >= timeoutMs) {
                        finished[0] = true;
                        drive.setManualDrive(0, 0, 0);
                        drive.update();
                        return;
                    }
                    limelight.update();
                    Limelight.BallDetection ball = limelight.getClosestBall();
                    if (ball == null) {
                        double creep = Math.max(0.15, Math.min(0.30, lastForward[0]));
                        drive.setManualDrive(creep, 0, 0);
                        drive.update();
                        return;
                    }
                    double currentRange = limelight.estimateRangeInches(ball.tyDeg);
                    double tx = ball.txDeg - Limelight.CHASE_TX_OFFSET_DEG;
                    turnPid.setGains(Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
                    rangePid.setGains(Limelight.CHASE_KP_RANGE, Limelight.CHASE_KI_RANGE, Limelight.CHASE_KD_RANGE);
                    turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
                    rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);
                    double forward = 0.0;
                    if (!Double.isNaN(currentRange)) {
                        forward = rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, currentRange);
                        if (forward < 0) forward = 0;
                    }
                    boolean near = !Double.isNaN(currentRange) && currentRange <= Limelight.CHASE_STRAFE_RANGE_IN;
                    double turn;
                    double strafe = 0.0;
                    if (near) {
                        strafe = Math.max(-Limelight.CHASE_MAX_STRAFE,
                                Math.min(Limelight.CHASE_MAX_STRAFE, -Limelight.CHASE_KP_STRAFE * tx));
                        turn = -turnPid.calculate(tx, 0.0) * Limelight.CHASE_NEAR_TURN_SCALE;
                        forward = Math.max(forward, Limelight.CHASE_MIN_FORWARD);
                    } else {
                        turn = -turnPid.calculate(tx, 0.0);
                        if (Math.abs(tx) < 15.0) {
                            forward = Math.max(forward, Limelight.CHASE_MIN_FORWARD * 0.7);
                        }
                    }
                    lastForward[0] = forward;
                    drive.setManualDrive(forward, strafe, turn);
                    drive.update();
                })
                .setDone(() -> finished[0]
                        || (startMs[0] != 0 && System.currentTimeMillis() - startMs[0] >= timeoutMs))
                .setEnd(end -> {
                    finished[0] = true;
                    syncBallCount(intakeGate);
                    drive.setManualDrive(0, 0, 0);
                    drive.update();
                    drive.cancel();
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    transfer.setTransferState(Transfer.TransferState.OFF);
                })
                .requiring(drive, limelight, intake, transfer, intakeGate);

        return Groups.race(chase, Commands.waitMs(timeoutMs));
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
//                    transfer.setTransferState(Transfer.TransferState.IN);
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
            PathFollower drive,
            Blocker blocker,
            double goalX,
            double goalY,
            double halfWidth,
            double halfHeight
    ) {
        return Groups.sequential(
                Commands.waitUntil(() -> blocker.isOverRegionField(
                        drive.getFieldPose(), goalX, goalY, halfWidth, halfHeight)),
                openBlocker(blocker)
        ).requiring(blocker);
    }

    // drive + raise at the same time, open blocker only when its over the goal
    public static Command driveRaiseAndScore(
            PathFollower drive,
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
            PathFollower drive,
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
            PathFollower drive,
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
            if (lift.state == FourBarLinkage.LinkState.DOWN) {
                intake.setIntakeState(Intake.IntakeState.IN);
            }
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
                holdLift(lift, scoreState)
        );
    }
}
