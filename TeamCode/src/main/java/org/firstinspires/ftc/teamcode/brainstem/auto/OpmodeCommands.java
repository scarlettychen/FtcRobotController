package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.PedroDrive;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
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
//            transfer.setTransferState(Transfer.TransferState.OUT);
        }).requiring(intake, transfer);
    }

    // go to {x, y, headingDeg} via pedro pathdrive
    public static Command driveTo(PedroDrive drive, double[] fieldPose) {
        final boolean[] ran = {false};
        return Command.build()
                .setStart(() -> {
                    ran[0] = false;
                    drive.pathDrive(fieldPose);
                })
                .setExecute(() -> {
                    drive.update();
                    ran[0] = true;
                })
                .setDone(() -> ran[0] && !drive.isBusy())
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

    // straight line to field x,y — holds start heading (rr/ftc field coords)
    public static Command lineTo(PedroDrive drive, double[] fieldPose) {
        final boolean[] ran = {false};
        return Command.build()
                .setStart(() -> {
                    ran[0] = false;
                    drive.lineDrive(fieldPose);
                })
                .setExecute(() -> {
                    drive.update();
                    ran[0] = true;
                })
                .setDone(() -> ran[0] && !drive.isBusy())
                .setEnd(endCondition -> {
                    if (drive.isBusy()) {
                        drive.getFollower().breakFollowing();
                    }
                })
                .requiring(drive);
    }

    public static Command lineTo(PedroDrive drive, double x, double y, double headingDegrees) {
        return lineTo(drive, new double[]{x, y, headingDegrees});
    }

    // robot-relative forward along live heading (inches)
    public static Command driveForward(PedroDrive drive, double inches) {
        final boolean[] ran = {false};
        return Command.build()
                .setStart(() -> {
                    ran[0] = false;
                    drive.forwardDrive(inches);
                })
                .setExecute(() -> {
                    drive.update();
                    ran[0] = true;
                })
                .setDone(() -> ran[0] && !drive.isBusy())
                .setEnd(endCondition -> {
                    if (drive.isBusy()) {
                        drive.getFollower().breakFollowing();
                    }
                })
                .requiring(drive);
    }

    // robot-relative reverse (inches, positive = back)
    public static Command driveBack(PedroDrive drive, double inches) {
        return driveForward(drive, -Math.abs(inches));
    }

    // robot-relative strafe (positive = left)
    public static Command strafe(PedroDrive drive, double inches) {
        final boolean[] ran = {false};
        return Command.build()
                .setStart(() -> {
                    ran[0] = false;
                    drive.strafeDrive(inches);
                })
                .setExecute(() -> {
                    drive.update();
                    ran[0] = true;
                })
                .setDone(() -> ran[0] && !drive.isBusy())
                .setEnd(endCondition -> {
                    if (drive.isBusy()) {
                        drive.getFollower().breakFollowing();
                    }
                })
                .requiring(drive);
    }

    public static Command strafeLeft(PedroDrive drive, double inches) {
        return strafe(drive, Math.abs(inches));
    }

    public static Command strafeRight(PedroDrive drive, double inches) {
        return strafe(drive, -Math.abs(inches));
    }

    // chase closest limelight ball w/ pid on tx + range (teleop drive, no path stuff)
    public static Command driveToClosestBall(
            PedroDrive drive, Limelight limelight, boolean red) {
        return driveToClosestBall(
                drive, limelight, red, true, Limelight.MAX_CHASE_TIME_MS, null);
    }

    public static Command driveToClosestBall(
            PedroDrive drive, Limelight limelight, boolean red, boolean finishOnLost) {
        return driveToClosestBall(
                drive, limelight, red, finishOnLost, Limelight.MAX_CHASE_TIME_MS, null);
    }

    public static Command driveToClosestBall(
            PedroDrive drive,
            Limelight limelight,
            boolean red,
            boolean finishOnLost,
            double maxChaseTimeMs) {
        return driveToClosestBall(drive, limelight, red, finishOnLost, maxChaseTimeMs, null);
    }

    /**
     * PID chase to closest limelight ball.
     * Ends on: reached ball, timeout, alliance out-of-bounds, or (optional) lost target.
     * If {@code reachedBall} is non-null, set true only when aimed+close.
     */
    public static Command driveToClosestBall(
            PedroDrive drive,
            Limelight limelight,
            boolean red,
            boolean finishOnLost,
            double maxChaseTimeMs,
            AtomicBoolean reachedBall) {
        final Follower follower = drive.getFollower();
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final int[] lostFrames = {0};
        final boolean[] done = {false};
        final long[] startMs = {0};

        return Command.build()
                .setStart(() -> {
                    follower.startTeleopDrive();
                    turnPid.reset();
                    rangePid.reset();
                    lostFrames[0] = 0;
                    done[0] = false;
                    startMs[0] = System.currentTimeMillis();
                    if (reachedBall != null) {
                        reachedBall.set(false);
                    }
                })
                .setExecute(() -> {
                    if (done[0]) {
                        return;
                    }

                    if (System.currentTimeMillis() - startMs[0] >= maxChaseTimeMs) {
                        follower.setTeleOpDrive(0, 0, 0, true);
                        follower.update();
                        done[0] = true;
                        return;
                    }

                    Pose field = drive.getPose().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
                    if (Limelight.isOutOfBounds(field.getX(), field.getY(), red)) {
                        follower.setTeleOpDrive(0, 0, 0, true);
                        follower.update();
                        done[0] = true;
                        return;
                    }

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
                        if (finishOnLost && lostFrames[0] >= Limelight.CHASE_LOST_FRAMES) {
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
                        if (reachedBall != null) {
                            reachedBall.set(true);
                        }
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

    // chase + beam-break count, then back off for clearance (not a substitute for collecting)
    public static Command collectBallsThenBackOff(
            PedroDrive drive,
            Limelight limelight,
            Intake intake,
            Transfer transfer,
            IntakeBeamBreak intakeGate,
            boolean red) {
        return collectBallsThenBackOff(
                drive,
                limelight,
                intake,
                transfer,
                intakeGate,
                red,
                Limelight.COLLECT_COUNT,
                Limelight.COLLECT_BACK_OFF_IN,
                Limelight.MAX_CHASE_TIME_MS / 1000.0);
    }

    public static Command collectBallsThenBackOff(
            PedroDrive drive,
            Limelight limelight,
            Intake intake,
            Transfer transfer,
            IntakeBeamBreak intakeGate,
            boolean red,
            int ballCount,
            double backInches,
            double timeoutSeconds) {
        return Groups.sequential(
                smartCollect(
                        drive, limelight, intake, transfer, intakeGate,
                        ballCount, timeoutSeconds),
                driveBack(drive, backInches)
        );
    }

    public static Command huntAndVerifyBall(
            PedroDrive drive, Limelight limelight, int[] possessedBalls) {

        final Follower follower = drive.getFollower();
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);
        final int[] lostFrames = {0};
        final boolean[] done = {false};
        final boolean[] wasInStrikeZone = {false};

        return Command.build()
                .setStart(() -> {
                    follower.startTeleopDrive();
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

                    // If we don't see a ball...
                    if (ball == null) {
                        lostFrames[0]++;
                        follower.setTeleOpDrive(0, 0, 0, true);
                        follower.update();

                        // INFERENCE: If we lost it, but we were right on top of it a split second ago, it went in!
                        if (wasInStrikeZone[0]) {
                            possessedBalls[0]++;
                            done[0] = true;
                            return;
                        }

                        // Otherwise, we just lost tracking. Abort after too many lost frames.
                        if (lostFrames[0] >= Limelight.CHASE_LOST_FRAMES) {
                            done[0] = true;
                        }
                        return;
                    }

                    lostFrames[0] = 0;

                    double range = limelight.estimateRangeInches(ball.tyDeg);

                    // Are we in the Strike Zone?
                    boolean aimed = Math.abs(ball.txDeg) <= Limelight.CHASE_TX_TOL_DEG;
                    boolean close = !Double.isNaN(range) && range <= (Limelight.CHASE_STOP_RANGE_IN + 2.0); // Added slight buffer

                    // Save this state for the NEXT frame, in case it disappears under the bumper
                    wasInStrikeZone[0] = (aimed && close);

                    // Drive math
                    double turn = -turnPid.calculate(ball.txDeg, 0.0);
                    double forward = 0.0;
                    if (!Double.isNaN(range)) {
                        forward = rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, range);
                        if (forward < 0) forward = 0;
                    }

                    follower.setTeleOpDrive(forward, 0, turn, true);
                    follower.update();
                })
                .setDone(() -> done[0])
                .setEnd(endCondition -> {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    follower.update();
                })
                .requiring(drive, limelight);
    }

    /**
     * Limelight chase + intake; ball count from dual beam breaks at the intake gate
     * (not limelight heuristics). Stops at {@code targetCount} or timeout.
     * Lost limelight does <b>not</b> abort — keeps intake running and creeps forward
     * so balls under the intake still trip the gate (old path backed off instead).
     */
    public static Command smartCollect(
            PedroDrive drive,
            Limelight limelight,
            Intake intake,
            Transfer transfer,
            IntakeBeamBreak intakeGate,
            int targetCount,
            double timeoutSeconds
    ) {
        final Follower follower = drive.getFollower();
        final PIDController turnPid = new PIDController(0, 0, 0);
        final PIDController rangePid = new PIDController(0, 0, 0);

        final ElapsedTime chaseTimer = new ElapsedTime();
        final boolean[] done = {false};
        final double[] lastForward = {0.15};

        return Command.build()
                .setStart(() -> {
                    follower.startTeleopDrive();
                    turnPid.reset();
                    rangePid.reset();
                    intakeGate.resetCount();
                    estimatedBallsInRobot = 0;
                    chaseTimer.reset();
                    done[0] = false;
                    lastForward[0] = 0.15;

                    intake.setIntakeState(Intake.IntakeState.IN);
                    transfer.setTransferState(Transfer.TransferState.IN);
                })
                .setExecute(() -> {
                    // gate.count updates in robot.update() before Scheduler.execute
                    syncBallCount(intakeGate);
                    if (intakeGate.getBallCount() >= targetCount
                            || chaseTimer.seconds() > timeoutSeconds) {
                        done[0] = true;
                        return;
                    }

                    limelight.update();
                    Limelight.BallDetection ball = limelight.getClosestBall();

                    // No vision: keep creeping / last forward so intake can still eat
                    if (ball == null) {
                        double creep = Math.max(0.12, Math.min(0.25, lastForward[0]));
                        follower.setTeleOpDrive(creep, 0, 0, true);
                        follower.update();
                        return;
                    }

                    double currentRange = limelight.estimateRangeInches(ball.tyDeg);

                    turnPid.setGains(Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
                    rangePid.setGains(Limelight.CHASE_KP_RANGE, Limelight.CHASE_KI_RANGE, Limelight.CHASE_KD_RANGE);
                    turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
                    rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);

                    double turn = -turnPid.calculate(ball.txDeg, 0.0);
                    double forward = 0.0;
                    if (!Double.isNaN(currentRange)) {
                        forward = rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, currentRange);
                        if (forward < 0) forward = 0;
                    }
                    // close enough: still nudge in so the gate sees the ball
                    if (forward < 0.12
                            && !Double.isNaN(currentRange)
                            && currentRange <= Limelight.CHASE_STOP_RANGE_IN + 4.0) {
                        forward = 0.12;
                    }
                    lastForward[0] = forward;

                    follower.setTeleOpDrive(forward, 0, turn, true);
                    follower.update();
                })
                .setDone(() -> done[0])
                .setEnd(endCondition -> {
                    syncBallCount(intakeGate);
                    follower.setTeleOpDrive(0, 0, 0, true);
                    follower.update();
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    transfer.setTransferState(Transfer.TransferState.OFF);
                })
                .requiring(drive, limelight, intake, transfer, intakeGate);
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
