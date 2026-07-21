package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.ActionLibrary;
import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.PedroDrive;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.BlueClosePoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.RedClosePoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.RobotPoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.TestPoses;

/**
 * Team-editable named drive and subsystem commands.
 *
 * <p>Match autos should call high-level helpers ({@link #tryCollect()}, {@link #tryScore()},
 * {@link #safeAlign()}, {@link #recoverLocalization()}) — not raw {@code retry}/{@code validate}
 * trees. Recovery stays local to the action that can fail.
 */
public class RobotActions extends ActionLibrary {
    protected final BrainSTEMRobot robot;
    private final RobotPoses bluePoses;
    private final RobotPoses redPoses;

    /** Default collect/score retry budget (deterministic, fixed). */
    public static final int DEFAULT_RETRY_ATTEMPTS = 2;
    /** Shooter spin-up / align wait timeout (seconds). */
    public static final double DEFAULT_VALIDATE_TIMEOUT_S = 1.5;

    public RobotActions(BrainSTEMRobot robot) {
        this(robot, new PedroDrive(robot.follower), new BlueClosePoses(), new RedClosePoses());
    }

    public RobotActions(
            BrainSTEMRobot robot,
            PedroDrive drive,
            RobotPoses blue,
            RobotPoses red
    ) {
        super(drive, blue, red);
        this.robot = robot;
        this.bluePoses = blue;
        this.redPoses = red;
    }

    /** Smoke-test actions: start (0,0,0), goal at +5 in X. */
    public static RobotActions forSmokeTest(BrainSTEMRobot robot) {
        TestPoses poses = new TestPoses();
        return new RobotActions(robot, new PedroDrive(robot.follower), poses, poses);
    }

    public RobotPoses robotPoses() {
        return isRed() ? redPoses : bluePoses;
    }

    // ---- Sensor stubs (wire to real hardware) ----

    /** True when a game piece is held (beam-break / color / current). */
    public boolean hasGamePiece() {
        return false;
    }

    /** True when shooter is at target velocity. */
    public boolean isShooterAtSpeed() {
        return true;
    }

    /** True when turret / heading is within align tolerance. */
    public boolean isAligned() {
        return true;
    }

    /** True when Pinpoint pose is finite / trusted enough to keep pathing. */
    public boolean isLocalizationReasonable() {
        return robot.pinpoint != null && !robot.pinpoint.isNAN();
    }

    // ---- High-level resilient actions (prefer these in match autos) ----

    /**
     * Collect with local retry: up to {@link #DEFAULT_RETRY_ATTEMPTS} fresh collect attempts
     * until {@link #hasGamePiece()}. Always finishes so the auton stays deterministic.
     */
    public AutoCommand tryCollect() {
        return retry(this::collect, this::hasGamePiece, DEFAULT_RETRY_ATTEMPTS);
    }

    /**
     * Score only if a piece is present; otherwise run local intake recovery (no global tree).
     * Scoring itself waits for shooter speed with a fixed timeout.
     */
    public AutoCommand tryScore() {
        return validate(this::hasGamePiece, score(), recoverIntake());
    }

    /**
     * Align with local retry until {@link #isAligned()} or attempts exhausted.
     */
    public AutoCommand safeAlign() {
        return retry(this::align, this::isAligned, DEFAULT_RETRY_ATTEMPTS);
    }

    /**
     * Local localization recovery: brief settle, then validate pose is still usable.
     * Does not change pathing gains or Pedro drive math.
     */
    public AutoCommand recoverLocalization() {
        return sequence(
                run(() -> { /* optional: pause vision fusion / hold last pose */ }),
                waitSeconds(0.15),
                validate(
                        this::isLocalizationReasonable,
                        run(() -> { /* pose OK — continue */ }),
                        run(() -> {
                            // Last resort stub: re-stamp current Pinpoint reading into Pedro.
                            // Replace with Limelight / known-tag reseat when available.
                            if (robot.pinpoint != null && !robot.pinpoint.isNAN()) {
                                robot.follower.setStartingPose(robot.pinpoint.getPose());
                            }
                        })
                )
        );
    }

    // ---- Single-attempt primitives (used by try* via Supplier) ----

    /** One collect attempt: collector on → first-spike path → collector off. */
    public AutoCommand collect() {
        return sequence(
                setCollectorOn(),
                collectFirstSpike(),
                setCollectorOff()
        );
    }

    /**
     * One score attempt: drive to shoot, spin up, wait for speed, fire, idle.
     * Path geometry unchanged — only wraps existing named drives.
     */
    public AutoCommand score() {
        return sequence(
                driveToCloseShoot(),
                shooterTurnOnClose(),
                waitUntilValidated(this::isShooterAtSpeed, DEFAULT_VALIDATE_TIMEOUT_S),
                validate(
                        this::isShooterAtSpeed,
                        moveSpindexer360(),
                        run(() -> { /* skip fire if never at speed */ })
                ),
                shooterIdle()
        );
    }

    /** One align attempt: precision line to look-at pose. */
    public AutoCommand align() {
        return driveToLookAtObelisk();
    }

    /** Brief intake recovery (local to collect/score failure paths). */
    public AutoCommand recoverIntake() {
        return sequence(
                setCollectorOn(),
                waitSeconds(0.4),
                setCollectorOff()
        );
    }

    // ---- Named drives (coordinates resolve when each command initializes) ----

    public AutoCommand driveForwardFive() {
        cruise();
        return getDrive().forwardDrive(5.0);
    }

    public AutoCommand side() {
        cruise();
        return getDrive().turnTo(90);
    }

    /**
     * Smoke-test polyline from field origin: forward → left/turn → continue.
     * Place robot at (0,0) facing field +X (0°).
     */
    public AutoCommand driveSmokePath() {
        return pathDriveFewPoints(
                AlliancePoses.xyz(24, 0, 0),
                AlliancePoses.xyz(24, 16, 90),
                AlliancePoses.xyz(40, 16, 90)
        );
    }

    /** Straight-segment path through the given field waypoints {@code {x,y,headingDegrees}}. */
    public AutoCommand pathDriveFewPoints(double[]... waypoints) {
        cruise();
        return pathDrive(waypoints);
    }

    public AutoCommand driveToGoal() {
        return driveToCloseShoot();
    }

    public AutoCommand driveToCloseShoot() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().close1Shooting;
        });
    }

    public AutoCommand driveToLookAtObelisk() {
        return lineDrive(() -> {
            precision();
            return robotPoses().lookAtOb;
        });
    }

    public AutoCommand driveToOpenGate() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().openGatePos;
        });
    }

    public AutoCommand driveOffLine() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().strafePos;
        });
    }

    public AutoCommand collectFirstSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect1Pre;
                },
                () -> robotPoses().firstSpikeEnd
        );
    }

    public AutoCommand collectSecondSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect2Mid;
                },
                () -> robotPoses().collect2Pre,
                () -> robotPoses().secondSpikeEnd
        );
    }

    public AutoCommand collectThirdSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect3Pre;
                },
                () -> robotPoses().thirdSpikeEnd
        );
    }

    public AutoCommand driveToShootViaPass() {
        return pathDrive(
                () -> {
                    precision();
                    return robotPoses().collect3PrePass;
                },
                () -> robotPoses().close1Shooting
        );
    }

    // ---- Named subsystem commands: connect these to fields on BrainSTEMRobot ----

    public AutoCommand shooterTurnOnClose() {
        return run(() -> { /* robot.shooter.setShooterShootClose(); */ });
    }

    public AutoCommand shooterIdle() {
        return run(() -> { /* robot.shooter.setShooterIdle(); */ });
    }

    public AutoCommand setCollectorOn() {
        return run(() -> { /* robot.collector.auto(); */ });
    }

    public AutoCommand setCollectorOff() {
        return run(() -> { /* robot.collector.off(); */ });
    }

    public AutoCommand rampUp() {
        return run(() -> { /* robot.ramp.up(); */ });
    }

    public AutoCommand rampDown() {
        return run(() -> { /* robot.ramp.down(); */ });
    }

    public AutoCommand moveSpindexer360() {
        return run(() -> { /* robot.spindexer.spin360(); */ });
    }
}
